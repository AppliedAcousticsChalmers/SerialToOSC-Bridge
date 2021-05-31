import argparse


def parse_cmd_args():
    # create parser and introduce all possible arguments
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-sp",
        "--SERIAL_PORT",
        type=str,
        required=True,
        help="System specific path to tracker interface being provided by an appropriate hardware driver",
    )
    parser.add_argument(
        "-dt",
        "--DEVICE_TYPE",
        type=str,
        required=True,
        choices=["POLHEMUS_PATRIOT", "POLHEMUS_FASTRACK", "RAZOR_AHRS", "AUTO_ROTATE"],
        help="Type information of hardware providing head tracking data",
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

    # parse arguments
    args = parser.parse_args()

    for a in args.__dict__:
        value = args.__dict__[a]
        # TODO: this has to be reimplemented
        set_arg(config, a, value)


def transform_into_type(str_or_instance, _type):
    """
    Parameters
    ----------
    str_or_instance : str, Type or None
        string or instance of type that should be transformed
    _type : type
        type that should be transformed into

    Returns
    -------
    class
        type instance

    Raises
    ------
    ValueError
        in case unknown type is given
    """

    def get_type_str():
        return f"{_type.__module__}.{_type.__name__}"

    if str_or_instance is None:
        return None
    elif isinstance(str_or_instance, str):
        if str_or_instance.upper() == "NONE":
            return None
        try:
            # transform string into enum, will fail in case an invalid type string was given
            # noinspection PyUnresolvedReferences
            return _type[str_or_instance]
        except KeyError:
            raise ValueError(
                f'unknown parameter "{str_or_instance}", see `{get_type_str()}` for reference!'
            )
    elif isinstance(str_or_instance, _type):
        return str_or_instance
    else:
        raise ValueError(
            f"unknown parameter type `{type(str_or_instance)}`, see `{get_type_str()}` for "
            f"reference!"
        )


def transform_into_wrapped_angles(azim, elev, tilt, is_deg=True, deg_round_precision=0):
    """
    Parameters
    ----------
    azim : float
        azimuth / yaw angle (will be wrapped to -180..180 degrees)
    elev : float
        elevation / pitch angle (will be wrapped to -90..90 degrees)
    tilt : float
        tilt / roll angle (will be wrapped to -180..180 degrees)
    is_deg : bool, optional
        if provided and delivered values are in degrees, radians otherwise
    deg_round_precision : int, optional
        number of decimals to round to (only in case of angles in degrees)

    Returns
    -------
    list of float
        azimuth, elevation and tilt angles in degrees or radians being wrapped
    """
    if is_deg:
        _AZIM_WRAP = 360
        _ELEV_WRAP = 180
        _TILT_WRAP = 360
    else:
        import math

        _AZIM_WRAP = 2 * math.pi
        _ELEV_WRAP = math.pi
        _TILT_WRAP = 2 * math.pi

    azim = azim % _AZIM_WRAP
    elev = elev % _ELEV_WRAP
    tilt = tilt % _TILT_WRAP

    if azim > _AZIM_WRAP / 2:
        azim -= _AZIM_WRAP
    if elev > _ELEV_WRAP / 2:
        elev -= _ELEV_WRAP
    if tilt > _TILT_WRAP / 2:
        tilt -= _TILT_WRAP

    if is_deg:
        azim = round(azim, ndigits=deg_round_precision)
        elev = round(elev, ndigits=deg_round_precision)
        tilt = round(tilt, ndigits=deg_round_precision)

    return [azim, elev, tilt]


LOG_SEPARATOR = "--------------------------------------------------------------"
"""String to improve visual orientation for a clear logging behaviour."""
