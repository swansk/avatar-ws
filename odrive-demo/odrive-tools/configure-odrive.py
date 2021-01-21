import argparse
import time

import odrive
from odrive.enums import *
from fibre.protocol import ChannelBrokenException

encoder_config_dict = {
    "mode": 257,
    "use_index": False,
    "find_idx_on_lockin_only": False,
    "abs_spi_cs_gpio_pin": 3,
    "zero_count_on_find_idx": True,
    "cpr": 16384,
    "offset": 2224,
    "pre_calibrated": False,
    "offset_float": 0.9906406402587891,
    "enable_phase_interpolation": True,
    "bandwidth": 1000.0,
    "calib_range": 0.019999999552965164,
    "calib_scan_distance": 50.26548385620117,
    "calib_scan_omega": 12.566370964050293,
    "idx_search_unidirectional": False,
    "ignore_illegal_hall_state": False,
    "sincos_gpio_pin_sin": 3,
    "sincos_gpio_pin_cos": 4,
}


motor_config_dict = {
    "pre_calibrated": False,
    "pole_pairs": 21,
    "calibration_current": 10.0,
    "resistance_calib_max_voltage": 2.0,
    "phase_inductance": 5.5034452088875696e-05,
    "phase_resistance": 0.13042733073234558,
    "torque_constant": 0.03999999910593033,
    "direction": -1,
    "motor_type": 0,
    "current_lim": 10.0,
    "current_lim_margin": 8.0,
    "torque_lim": "inf",
    "inverter_temp_limit_lower": 100.0,
    "inverter_temp_limit_upper": 120.0,
    "requested_current_range": 60.0,
    "current_control_bandwidth": 1000.0,
    "acim_slip_velocity": 14.706000328063965,
    "acim_gain_min_flux": 10.0,
    "acim_autoflux_min_Id": 10.0,
    "acim_autoflux_enable": False,
    "acim_autoflux_attack_gain": 10.0,
    "acim_autoflux_decay_gain": 1.0,
}


def configure_odrive_axis_device(odrive_axis_device, config_dict):
    for key in config_dict:
        setattr(odrive_axis_device.config, key, config_dict[key])


def calibrate_odrive_axis(odrive_axis):
    # Motor calibration
    odrive_axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

    while odrive_axis.current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)

    odrive_axis.motor.config.pre_calibrated = True

    # Encoder calibration
    odrive_axis.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION

    while odrive_axis.current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)

    odrive_axis.encoder.config.pre_calibrated = True


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Configure ODrive axis with settings used for AVATAR Capstone project. Please only have one ODrive connected when using this script (unless you specify the serial no)."
    )
    parser.add_argument("--axis", type=str, help="Axis to configure.", default="axis0")
    parser.add_argument(
        "--serial_no",
        type=str,
        help="Serial number of ODrive to configure.",
        default=None,
    )
    parser.add_argument(
        "-cal",
        action="store_true",
        help="Calibrate motor after configuration.",
        default=False,
    )
    parser.add_argument(
        "--cs_pin",
        type=int,
        help="GPIO pin connected to encoder CS",
        default=None
    )
    args = parser.parse_args()

    # Search for connected Odrive.
    my_odrive = odrive.find_any(serial_number=args.serial_no)

    if my_odrive == None:
        print("Could not find ODrive, exiting...")
        return -1

    # Check ODrive version, report and error if incorrect
    fw_version = "{}.{}.{}".format(
        my_odrive.fw_version_major,
        my_odrive.fw_version_minor,
        my_odrive.fw_version_revision,
    )
    print("Detected ODrive with FW Version:", fw_version)
    if fw_version != "0.5.1":
        print(
            "Need version 0.5.1, please run command: 'sudo odrivetool dfu' to update firmware, exiting..."
        )
        return -1

    odrive_axis = getattr(my_odrive, args.axis)

    if args.cs_pin is not None:
        encoder_config_dict["abs_spi_cs_gpio_pin"] = args.cs_pin

    # Configure motor and encoder
    configure_odrive_axis_device(odrive_axis.encoder, encoder_config_dict)
    configure_odrive_axis_device(odrive_axis.motor, motor_config_dict)

    if args.cal:
        print("Calibrating ODrive...")
        calibrate_odrive_axis(odrive_axis)

    # Save configuration
    my_odrive.save_configuration()
    print("Odrive configured!")


if __name__ == "__main__":
    main()