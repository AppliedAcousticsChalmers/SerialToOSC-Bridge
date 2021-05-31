import argparse

from _tracker import HeadTracker


def parse_cmd_args():
    # create parser and introduce all possible arguments
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-dt",
        "--DEVICE_TYPE",
        type=str,
        required=True,
        choices=["POLHEMUS_PATRIOT", "POLHEMUS_FASTRACK", "RAZOR_AHRS", "AUTO_ROTATE"],
        help="Type information of hardware providing head tracking data",
    )
    parser.add_argument(
        "-sp",
        "--SERIAL_PORT",
        type=str,
        required=False,
        default="",
        help="System specific path to tracker interface being provided by an appropriate hardware driver",
    )
    parser.add_argument(
        "-oa",
        "--OSC_ADDRESS",
        type=str,
        required=False,
        default="127.0.0.1",
        help="IP address of OSC target",
    )
    parser.add_argument(
        "-op",
        "--OSC_PORT",
        type=int,
        required=True,
        help="IP port of OSC target",
    )
    parser.add_argument(
        "-ot",
        "--OSC_TARGET",
        type=str,
        required=True,
        help='OSC target address, usually in the shape of "/client/parameter"',
    )
    parser.add_argument(
        "-vm",
        "--VERBOSE_MODE",
        type=int,
        required=False,
        default=1,
        choices=[0, 1, 2],
        help="Logging verbosity (0: Waning messages, 1: Status messages, 2: Debug messages)",
    )
    # parse arguments
    _args = parser.parse_args()
    # debug print arguments
    if _args.VERBOSE_MODE > 1:
        print(f"found arguments:")
        for a in _args.__dict__:
            print(f" --> {a}: {_args.__dict__[a]}")
    return _args


if __name__ == "__main__":
    # parse command line arguments
    args = parse_cmd_args()

    try:
        # initialize tracker and with OSC client
        tracker = HeadTracker.create_instance_by_type(
            device_type=args.DEVICE_TYPE,
            serial_port=args.SERIAL_PORT,
            osc_address=args.OSC_ADDRESS,
            osc_port=args.OSC_PORT,
            osc_target=args.OSC_TARGET,
            verbose_mode=args.VERBOSE_MODE,
        )

        # execute continuous tracking (the tracker contains code to handle
        # keyboard interrupts and gracefully halt execution)
        tracker.run()

    except ValueError as e:
        print(e.args[0])
        exit(1)

    if args.VERBOSE_MODE:
        print("application ended.")
    exit(0)
