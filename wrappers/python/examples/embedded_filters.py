## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

#####################################################
##                   Embedded Filters              ##
#####################################################


import struct
import sys
import pyrealsense2 as rs


'''
Aim of this example is to show how the python API for embedded filters must be used.
Scenario is:
1. Get connected DDS device
2. Get its depth sensor
3. Get its embedded filters
4. For each embedded filter: show supported options and print current values
5. For each embedded filter: show supported COMPOSITE options and print current values too -
   these are a completely separate identity space from the ordinary (scalar) options above (see
   rs2_composite_option_id in rs_composite_option.h): a single multi-field control exchanged
   atomically, in ONE UVC transaction. get/set_composite_option() hand back/take raw bytes rather
   than a typed value - Python has no templates, so (mirroring the C++/C99 samples under
   examples/composite-option and examples/C/composite-ctl) the caller unpacks/packs those bytes
   against the option's documented wire layout with the `struct` module.
'''

def list_embedded_filter_options(embedded_filter):
    options = embedded_filter.get_supported_options()
    print("Supported options:")
    for opt in options:
        print(repr(opt) + ": " + str(embedded_filter.get_option_value(opt).value))
    print("\n")


# Known wire layouts for composite options this example knows how to interpret - there is no
# generic "any composite option" cast (the SDK ships no per-id dispatch), so a new composite
# option needs its layout added here too, same as every other composite-option sample's typed
# dispatch. See rs_hkr_improved_close_range_control.h / rs_hkr_temporal_filter_dpp.h for the documented layouts.
_COMPOSITE_OPTION_LAYOUTS = {
    rs.composite_option_id.hkr_improved_close_range_control: (
        # dppc_header (version, flags, ctl_id, param_count, param_type) shared by the whole HKR
        # DPP control family, then Improved Close Range's 7 logical fields, then 1 reserved (always 0) slot.
        '<BBHBBiiiiiiii',
        ['version', 'flags', 'ctl_id', 'param_count', 'param_type',
         'enable', 'filter_type', 'downscale_ratio', 'shift_mode', 'shift_pixels',
         'threshold_mode', 'threshold_mm', 'reserved0']),
    rs.composite_option_id.hkr_temporal_filter_dpp: (
        # No shared header on this one - just its own 4 fields, tightly packed.
        '<ifii',
        ['enabled', 'smooth_alpha', 'smooth_delta', 'persistency_index']),
}


def list_embedded_filter_composite_options(embedded_filter):
    composite_ids = embedded_filter.get_supported_composite_options()
    if not composite_ids:
        print("Supports composite options: no (scalar options only)")
        return

    print("Supported composite options:")
    for id in composite_ids:
        print("  {}:".format(repr(id)))
        print("    read-only: {}".format(embedded_filter.is_composite_option_read_only(id)))
        print("    description: \"{}\"".format(embedded_filter.get_composite_option_description(id)))
        try:
            raw = embedded_filter.get_composite_option(id)
        except RuntimeError as e:
            # Registered but not actually functional on this device/FW is a real, expected
            # outcome (supports_composite_option()/get_supported_composite_options() only
            # reflect static registration, never a live capability check) - report and move on
            # rather than treating it as fatal, same as the C++/C99 samples' per-id try/catch.
            print("    SKIPPED (registered but not functional on this device/FW): {}".format(e))
            continue

        layout = _COMPOSITE_OPTION_LAYOUTS.get(id)
        if layout is None:
            print("    raw bytes ({}): {}".format(len(raw), raw.hex()))
            continue

        fmt, field_names = layout
        values = struct.unpack(fmt, raw)
        for name, value in zip(field_names, values):
            print("    {} = {}".format(name, value))
    print("\n")


def main(arguments=None):
    ctx = rs.context()
    try:
        device = ctx.query_devices()[0]
    except IndexError:
        print('Device is not connected')
        sys.exit(1)
    depth_sensor = device.first_depth_sensor()
    embedded_filters = depth_sensor.query_embedded_filters()
    if len(embedded_filters) == 0:
        print('No embedded filters found in this device')
        sys.exit(1)
    for filter in embedded_filters:
        if filter.get_type() == rs.embedded_filter_type.decimation:
            print("Decimation Embedded Filter found")
            list_embedded_filter_options(filter)
        elif filter.get_type() == rs.embedded_filter_type.temporal:
            print("Temporal Embedded Filter found")
            list_embedded_filter_options(filter)
        else:
            print("Embedded Filter found is of type: {}".format(filter.get_type()))
        # Composite options are a separate per-filter registry, orthogonal to the scalar-option
        # type dispatch above - checked for every filter regardless of type.
        list_embedded_filter_composite_options(filter)


if __name__ == '__main__':
    main()
