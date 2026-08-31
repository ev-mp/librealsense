// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.


#include <src/ds/features/improved-close-range-filter-feature.h>
#include <src/ds/d500/d500-device.h>
#include <src/ds/d500/hdrd-embedded-filter.h>
#include <src/ds/ds-private.h>
#include <src/uvc-sensor.h>

#include <librealsense2/h/rs_composite_option.h>
#include <librealsense2/h/rs_hkr_improved_close_range_control.h>


namespace librealsense {


/* static */ const feature_id improved_close_range_filter_feature::ID = "Improved Close Range filter feature";

improved_close_range_filter_feature::improved_close_range_filter_feature( d500_depth_sensor & depth_sensor )
{
    // Registers the ONE composite option this filter exposes, mirroring
    // temporal_filter_feature exactly. ds::DS5_HKR_IMPROVED_CLOSE_RANGE_CONTROL/rs2_improved_close_range_control drive the same
    // physical XU control formerly exposed as the scalar "Improved Close Range Depth" option
    // (close_range_xu_option, since removed).
    auto raw_depth_ep = std::dynamic_pointer_cast< uvc_sensor >( depth_sensor.get_raw_sensor() );
    depth_sensor.add_embedded_filter( std::make_shared< hdrd_embedded_filter >(
        raw_depth_ep,
        ds::DS5_HKR_IMPROVED_CLOSE_RANGE_CONTROL,
        static_cast< uint32_t >( sizeof( rs2_improved_close_range_control ) ),
        RS2_COMPOSITE_OPTION_HKR_IMPROVED_CLOSE_RANGE_CONTROL,
        "HKR Improved Close Range Control (prototype) - use rs2_set/get_composite_option, see rs_hkr_improved_close_range_control.h" ) );
}

feature_id improved_close_range_filter_feature::get_id() const
{
    return ID;
}


}  // namespace librealsense
