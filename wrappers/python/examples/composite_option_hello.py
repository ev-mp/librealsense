## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

#####################################################
##            Composite Option - Hello World       ##
#####################################################

'''
The smallest possible example of the composite-option API's TYPED surface: rs2.minz_control is
the actual rs2_minz_control C struct (see rs_hkr_minz_control.h), bound field-for-field directly
against pyrealsense2's C++ implementation - the exact same technique already used for
rs2.intrinsics/rs2.extrinsics/rs2.vector (see wrappers/python/c_files.cpp). There is exactly ONE
place, on the Python side, that names its fields: this binding (wrappers/python/pyrs_options.cpp),
which reads them straight from the real struct via pointer-to-member. If rs2_minz_control's shape
ever changes, that binding fails to compile - nothing here has its own hand-duplicated copy of the
layout to fall out of sync.

filter.get_minz_control(id) is the Python equivalent of the C++ wrapper's own
get_composite_option_as<rs2_minz_control>(id) template call - Python has no templates, so it's a
concretely-named method instead of a generic one, same as every other composite-option sample in
this repo dispatches by known id (see examples/composite-option/rs-composite-option-demo.cpp).

See wrappers/python/examples/embedded_filters.py for the untyped, raw-bytes version of this (get
raw bytes back, unpack them yourself with `struct`) - useful for an id this wrapper hasn't bound a
typed method for yet.
'''

import pyrealsense2 as rs


def main():
    print("Hello, composite option!")
    print("This demo shows how to use the composite_option API and its struct types (e.g. "
          "rs2.minz_control) directly from Python.\n")

    ctx = rs.context()
    dev = ctx.query_devices()[0]
    print("Device: {}".format(dev.get_info(rs.camera_info.name)))

    sensor = dev.first_depth_sensor()
    # Composite options live on one of the sensor's embedded filters, not the sensor itself.
    minz_filter = next(
        f for f in sensor.query_embedded_filters()
        if rs.composite_option_id.hkr_minz_control in f.get_supported_composite_options())

    # Typed get - no raw bytes, no struct.unpack: this IS an rs2_minz_control, with real
    # attributes, straight from the device.
    cfg = minz_filter.get_minz_control(rs.composite_option_id.hkr_minz_control)
    print("\nCurrent value: {}".format(cfg))
    print("  version         = {}".format(cfg.version))
    print("  flags           = {}".format(cfg.flags))
    print("  ctl_id          = {}".format(cfg.ctl_id))
    print("  param_count     = {}".format(cfg.param_count))
    print("  param_type      = {}".format(cfg.param_type))
    print("  enable          = {}".format(cfg.enable))
    print("  downscale_ratio = {}".format(cfg.downscale_ratio))
    print("  disparity_shift = {}".format(cfg.disparity_shift))
    print("  threshold       = {}".format(cfg.threshold))
    print("  threshold_mode  = {}".format(cfg.threshold_mode))

    # The typed range works the same way - min/max/step/default are each a full minz_control.
    rng = minz_filter.get_minz_control_range(rs.composite_option_id.hkr_minz_control)
    print("\nSupported range (version={}):".format(rng.version))
    print("  downscale_ratio : [{}, {}]".format(rng.min.downscale_ratio, rng.max.downscale_ratio))
    print("  disparity_shift : [{}, {}]".format(rng.min.disparity_shift, rng.max.disparity_shift))
    print("  threshold       : [{}, {}]".format(rng.min.threshold, rng.max.threshold))


if __name__ == '__main__':
    main()
