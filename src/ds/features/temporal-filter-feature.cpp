// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.


#include <src/ds/features/temporal-filter-feature.h>
#include <src/ds/d500/d500-device.h>
#include <src/ds/d500/d500-temporal-embedded-filter.h>
#include <src/uvc-sensor.h>


namespace librealsense {


/* static */ const feature_id temporal_filter_feature::ID = "Temporal filter feature";

temporal_filter_feature::temporal_filter_feature( d500_depth_sensor & depth_sensor )
{
    auto raw_depth_ep = std::dynamic_pointer_cast< uvc_sensor >( depth_sensor.get_raw_sensor() );
    depth_sensor.add_embedded_filter( std::make_shared< d500_temporal_embedded_filter >( raw_depth_ep ) );
}

feature_id temporal_filter_feature::get_id() const
{
    return ID;
}


}  // namespace librealsense
