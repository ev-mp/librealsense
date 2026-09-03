## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

#####################################################
##                   Embedded Filters              ##
#####################################################


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
   atomically, in ONE UVC transaction. Each id known to this example is registered below against
   its typed accessor method (e.g. get_hdrd_control()) - the Python equivalent of
   the C++ wrapper's get_composite_option_as<T>() template call (see
   wrappers/python/pyrs_options.cpp) - so what comes back is a real object bound field-for-field
   against the actual C struct, not raw bytes. Fields are then read off that object by
   introspection, so this example carries no hand-maintained wire layout of its own: no struct
   format string, no byte offsets, no field list to keep in sync by hand.
'''

def list_embedded_filter_options(embedded_filter):
    options = embedded_filter.get_supported_options()
    print("Supported options:")
    for opt in options:
        print(repr(opt) + ": " + str(embedded_filter.get_option_value(opt).value))
    print("\n")


# Typed accessor method registered per known composite-option id - the SDK ships no generic "any
# composite option" cast (there is no per-id dispatch table, in C++ or Python), so a new composite
# option still needs an entry here, same as every other composite-option sample in this repo
# dispatches by known id. Unlike a raw wire-layout table though, nothing here names a byte offset
# or format string: the registered method returns a real typed object (see
# rs.hdrd_control / rs.temporal_filter_dpp_config in pyrs_options.cpp), and
# _typed_fields() below reads its fields back by introspection.
_COMPOSITE_OPTION_ACCESSORS = {
    rs.composite_option_id.hkr_hdrd_control: 'get_hdrd_control',
    rs.composite_option_id.hkr_temporal_filter_dpp: 'get_temporal_filter_dpp_config',
}


def _typed_fields(value):
    '''The real, public fields on a typed composite-option object, discovered from the object
    itself rather than a hand-maintained name list. `header` is skipped - it's wire-transport
    framing (version/ctl_id/param_count/...), not one of the control's own logical fields.'''
    return [name for name in dir(value)
            if not name.startswith('_') and name != 'header' and not callable(getattr(value, name))]


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

        accessor_name = _COMPOSITE_OPTION_ACCESSORS.get(id)
        if accessor_name is None:
            print("    (no typed accessor registered for this id in this example - see "
                  "get_composite_option()/get_composite_option_range() for the untyped, raw-bytes API)")
            continue
        try:
            value = getattr(embedded_filter, accessor_name)(id)
        except RuntimeError as e:
            # Registered but not actually functional on this device/FW is a real, expected
            # outcome (supports_composite_option()/get_supported_composite_options() only
            # reflect static registration, never a live capability check) - report and move on
            # rather than treating it as fatal, same as the C++/C99 samples' per-id try/catch.
            print("    SKIPPED (registered but not functional on this device/FW): {}".format(e))
            continue

        for name in _typed_fields(value):
            print("    {} = {}".format(name, getattr(value, name)))
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
