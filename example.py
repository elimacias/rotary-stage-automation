from rotary_stage import RTLA

def main():
    degrees_to_rotate = 20
    rotation_direction = "cw"
    rotation_type = "absolute"

    rtla_args = {
        "device_path": "/dev/ttyUSB0",
        "baudrate": 9615,
        "time_out": 0.1,
        "velocity": 1.35,
        "acceleration": 100,
        "deceleration": 100,
    }

    rtla_instance = RTLA(**rtla_args)
    rtla_instance.setup()
    rtla_instance.set_rotation(degrees_to_rotate, rotation_direction, rotation_type)
    rtla_instance.run()
    rtla_instance.cleanup()


if __name__ == "__main__":
    main()
