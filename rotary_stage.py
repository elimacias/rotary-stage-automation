import time
import serial


class RTLA:
    COUNTS_PER_MOTOR_REV = 50000
    GEAR_RATIO = 144
    DRIVER_MOTOR_COUNTS_UNITS_VELOCITY = 0.1
    DRIVER_MOTOR_COUNTS_UNITS_ACCELERATION = 10
    DRIVER_MOTOR_COUNTS_UNITS_DECELERATION = 10
    DEGREES_PER_REVOLUTION = 360
    ROUNDING_DECIMAL = 2
    BYTE_LENGTH_TO_READ = 100
    COMMAND_SLEEP_TIME = 1
    DEGREES_AFTER_HOMING = 0
    DIRECTION_AFTER_HOMING = "cc"
    ROTATION_TYPE_AFTER_HOMING = "absolute"

    EVENT_STATUS_REGISTER = {
        "register_bits": 32,
        "motion_ongoing": {"query_bit": 27, "set": "1"},
    }
    TRAJECTORY_STATUS_REGISTER = {
        "register_bits": 16,
        "homing_success": {"query_bit": 12, "set": "1"},
        "homing_ongoing": {"query_bit": 13, "set": "1"},
        "move_aborted": {"query_bit": 14, "set": "1"},
    }

    EVENT_STATUS = "r0xa0"
    TRAJECTORY_STATUS = "r0xc9"
    POSITION_HEX = "r0xca"
    VELOCITY_HEX = "r0xcb"
    ACCELERATION_HEX = "r0xcc"
    DECELERATION_HEX = "r0xcd"
    DRIVER_MODE = "r0xc8"
    DRIVER_ENABLE = "r0x24"
    BAUDRATE_SETTING = "r0x90"
    FAULT_STATUS = "r0xa4"
    ENCODER = "enc"

    FAULT_REGISTER_CLEAR = "0xffff"

    SET_INPUT = "s"
    GET_OUTPUT = "g"
    MOTION_SETTER = "t"

    ROTATING_MOTION = "1"
    HOMING_MOTION = "2"
    STOP_MOTION = "0"

    DRIVER_ENABLE_ON = "31"
    DRIVER_ENABLE_OFF = "0"

    ROTATION_SIGN = {"cc": 1, "cw": -1}
    ROTATION_TYPES = {"absolute": "0", "relative": "256"}

    """The RTLA rotary stage class.

    Attributes:
        _dev_path (str): Path to device file.
        _baudrate (float): Device baud rate.
        _timeout (float): Communication timeout.
        _velocity (float): Rotational speed (degrees/s).
        _acceleration (float): Rotational acceleration (degrees/s^2).
        _deceleration (float): Rotational deceleration (degrees/s^2).
        _ser (object): The serial communication object.
        _counts_per_motor_rev (int): Programmed counts per motor rev.
        _status (dict): Stores rotary stage status until next move.
        _driver_units_velocity (float): Driver velocity units motor-counts/s.
        _driver_units_acceleration (int): Driver acceleration units motor-counts/s^2.
        _driver_units_deceleration (int): Driver deceleration units motor-counts/s^2

    Keyword Args:
        name (str): Rotary stage user-defined name.
        device-path (str): Path to device file.
        baudrate (float): Device baud rate.
        timeout (float): Communication timeout.
        velocity (float): Rotational speed (degrees/s).
        acceleration (float): Rotational acceleration (degrees/s^2).
        deceleration (float): Rotational deceleration (degrees/s^2).

    """

    def __init__(self, **kwargs):
        self.dev_path = kwargs.get("dev-path", "/dev/ttyUSB0")
        self.baudrate = kwargs.get("baudrate", 9615)
        self.timeout = kwargs.get("timeout", 0.1)
        self.velocity = kwargs.get("velocity", 1.35)
        self.acceleration = kwargs.get("acceleration", 100)
        self.deceleration = kwargs.get("deceleration", 100)
        self.ser = None
        self.status = {
            "position": self.DEGREES_AFTER_HOMING,
            "direction": "None",
            "rotation_type": "None",
            "degrees_rotated": self.DEGREES_AFTER_HOMING,
        }

    def setup(self):
        """Sets up connection with the rotary stage."""

        self.ser = serial.Serial(
            self.dev_path, baudrate=self.baudrate, timeout=self.timeout
        )

        # Make sure baudrate is correct
        if self._get_baudrate() != self.baudrate:
            self._set_baudrate()

        # Clear latched faults
        self._clear_faults()

        # Set initial parameters
        self._set_velocity()
        self._set_acceleration()
        self._set_deceleration()
        self._enable_drive()
        self.home()

    def set_rotation(self, degrees, direction, rotation_type):
        """Sets the angle to which the rotary stage will rotate. The
        type of rotation can be relative to current position or absolute, meaning
        that the angle is relative to the home position.

        Args:
            rotation_type (str): "relative" or "absolute" rotation.
            degrees (float): Rotation angle.
            direction (str): Rotating "cw" (clock-wise) or "cc" (counter-clockwise).
        """
        self._command(
            self.SET_INPUT, self.DRIVER_MODE, val=self.ROTATION_TYPES[rotation_type]
        )
        self._command(
            self.SET_INPUT,
            self.POSITION_HEX,
            val=str(self.ROTATION_SIGN[direction] * self._deg_to_counts(degrees)),
        )
        print(
            f"setting rotation to {degrees} {rotation_type} "
            f"degrees in the {direction} direction."
        )
        self._status_update(degrees, direction, rotation_type)

    def run(self):
        """Moves the rotary stage."""
        self._command(self.MOTION_SETTER, self.ROTATING_MOTION)
        print("Rotary stage moving.")
        rotating = True
        while rotating:
            register_output = self._register_parser(
                self.EVENT_STATUS, self.EVENT_STATUS_REGISTER["register_bits"]
            )
            if not self._is_set(
                register_output, self.EVENT_STATUS_REGISTER["motion_ongoing"]
            ):
                rotating = False
                print("Rotary stage stopped moving.")
        self._check_aborted()

    def reset(self, **kwargs):
        """Resets velocity, acceleration, or deceleration. If a parameters is
        not passed, it keeps current value.

        Keyword Args:
            velocity (float): Rotary stage velocity (degrees/s).
            acceleration (float): Rotary stage acceleration (degrees/s^2).
            deceleration (float): Rotary stage deceleration (degrees/s^2).
        """
        self.velocity = kwargs.get("velocity", self.velocity)
        self.acceleration = kwargs.get("acceleration", self.acceleration)
        self.deceleration = kwargs.get("deceleration", self.deceleration)
        self._set_velocity()
        self._set_acceleration()
        self._set_deceleration()

    def stop(self):
        """Stop the rotating motion."""
        self._command(self.MOTION_SETTER, self.STOP_MOTION)

    def home(self):
        """Returns rotary stage to home position."""

        # Ensures that homing process does not rotate cw while negative
        print("homing")
        if self.status["position"] < 0:
            self.set_rotation(abs(self.status["position"]), "cc", "relative")
            self.run()

        # Send homing command to the drive
        self._command(self.MOTION_SETTER, self.HOMING_MOTION)
        rotating = True
        while rotating:
            register_output = self._register_parser(
                self.TRAJECTORY_STATUS, self.TRAJECTORY_STATUS_REGISTER["register_bits"]
            )
            if not self._is_set(
                register_output, self.TRAJECTORY_STATUS_REGISTER["homing_ongoing"]
            ):
                rotating = False
                if self._is_set(
                    register_output, self.TRAJECTORY_STATUS_REGISTER["homing_success"]
                ):
                    print("Homing success.")
                else:
                    self._check_aborted()
        self._status_update(
            self.DEGREES_AFTER_HOMING,
            self.DIRECTION_AFTER_HOMING,
            self.ROTATION_TYPE_AFTER_HOMING,
        )

    def cleanup(self):
        """Closes connection with Rotary stage drive. First, makes sure that motor
        is stopped, the rotary stage is homed , and drive disabled.
        """
        self.stop()
        self.home()
        self._disable_drive()
        self.ser.close()

    def _is_set(self, register_output, query_dict):
        """Checks register outputs to see if in-motion bit is set."""
        return register_output[query_dict["query_bit"]] == query_dict["set"]

    def _check_aborted(self):
        """Checks if move is aborted."""
        register_output = self._register_parser(
            self.TRAJECTORY_STATUS, self.TRAJECTORY_STATUS_REGISTER["register_bits"]
        )
        if self._is_set(
            register_output, self.TRAJECTORY_STATUS_REGISTER["homing_ongoing"]
        ):
            raise SystemError

    def _register_parser(self, register_to_query, register_bits):
        """Parses the drive output message and converts it to a string"""
        output = list(
            format(
                int(self._command(self.GET_OUTPUT, register_to_query).strip("v ")),
                f"0{register_bits}b",
            )
        )
        return list(reversed(output))

    def _status_update(self, degrees, direction, rotation_type):
        """Updates position, direction, rotation-type status, and degrees rotated."""
        self.status["direction"] = direction
        if rotation_type == "relative":
            self.status["degrees_rotated"] = degrees
            self.status["position"] += self.ROTATION_SIGN[direction] * degrees
        else:
            self.status["degrees_rotated"] = abs(self.status["position"] - degrees)
            self.status["position"] = self.ROTATION_SIGN[direction] * degrees
        self.status["rotation_type"] = rotation_type

    def _set_velocity(self):
        """Sets rotary stage velocity. Velocity is in rotary stage degrees and is
        converted to motor counts.
        """
        self._command(
            self.SET_INPUT,
            self.VELOCITY_HEX,
            val=str(
                int(
                    self._deg_to_counts(self.velocity)
                    / self.DRIVER_MOTOR_COUNTS_UNITS_VELOCITY
                )
            ),
        )

    def _set_acceleration(self):
        """Sets rotary stage acceleration. Acceleration is in rotary stage degrees and
        is converted to motor counts.
        """
        self._command(
            self.SET_INPUT,
            self.ACCELERATION_HEX,
            val=str(
                int(
                    self._deg_to_counts(self.acceleration)
                    / self.DRIVER_MOTOR_COUNTS_UNITS_ACCELERATION
                )
            ),
        )

    def _set_deceleration(self):
        """Sets rotary stage deceleration. Deceleration is in rotary stage degrees and
        is converted to motor counts.
        """
        self._command(
            self.SET_INPUT,
            self.DECELERATION_HEX,
            val=str(
                int(
                    self._deg_to_counts(self.deceleration)
                    / self.DRIVER_MOTOR_COUNTS_UNITS_DECELERATION
                )
            ),
        )

    def _set_baudrate(self, **kwargs):
        """Sets connection baudrate."""
        self._command(
            self.SET_INPUT,
            self.BAUDRATE_SETTING,
            val=str(kwargs.get("baudrate", self._get_baudrate())),
        )

    def _get_baudrate(self):
        """Gets drive baudrate.

        return: Baudrate as an integer.
        """
        return int(self._command(self.GET_OUTPUT, self.BAUDRATE_SETTING).strip("v "))

    def get_settings(self):
        """Get drive settings."""
        return self.ser.get_settings()

    def _run_time(self, degrees):
        """Estimates moving rotary run time.

        return: Time in seconds as float.
        """
        return abs(degrees) / self.velocity

    def _deg_to_counts(self, degrees):
        """Converts rotary stage degrees to motor counts.

        Args:
            degrees (float): Rotary stage degrees.

        return: Integer representing the motor counts.
        """
        return int(degrees * self._counts_per_deg())

    def _counts_to_deg(self, counts):
        """Converts motor counts to rotary stage degrees.

        Args:
            counts (int): Motor counts.

        return: Float representing the rotary stage degrees.
        """
        return round(counts / self._counts_per_deg(), self.ROUNDING_DECIMAL)

    def _counts_per_deg(self):
        """Computes motor counts per degree. Depends on Rotary table gear ratio and
        the programmed motor counts per motor revolution.

        return: Integer representing counts per degree.
        """
        return int(
            self.COUNTS_PER_MOTOR_REV * self.GEAR_RATIO / self.DEGREES_PER_REVOLUTION
        )

    def _clear_faults(self):
        """Clears latch faults in the driver encoder and register."""
        self._command(self.ENCODER, "clear")
        self._command(self.SET_INPUT, self.FAULT_STATUS, val=self.FAULT_REGISTER_CLEAR)

    def _enable_drive(self):
        """Enable drive in programmed position mode for stepper motor."""
        self._command(self.SET_INPUT, self.DRIVER_ENABLE, val=self.DRIVER_ENABLE_ON)

    def _disable_drive(self):
        """Disable motor driver."""
        self._command(self.SET_INPUT, self.DRIVER_ENABLE, val=self.DRIVER_ENABLE_OFF)

    def _command(self, command, param, **kwargs):
        """Sends command to motor driver.

        Args:
            command (str): ASCII command.

        return: The driver output message.
        """
        driver_message = f"{command} {param} "
        if kwargs.get("val", False):
            driver_message += kwargs.get("val")
        driver_message += "\r"
        self.ser.write(driver_message.encode())
        time.sleep(self.COMMAND_SLEEP_TIME)
        return str(self.ser.read(self.BYTE_LENGTH_TO_READ), "utf-8")
