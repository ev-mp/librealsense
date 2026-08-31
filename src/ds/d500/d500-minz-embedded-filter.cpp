// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

#include "d500-minz-embedded-filter.h"
#include "ds/ds-private.h"
#include <src/ds/composite-xu-option.h>

#include <librealsense2/h/rs_composite_option.h>
#include <librealsense2/h/rs_hkr_minz_control.h>

namespace librealsense {

d500_minz_embedded_filter::d500_minz_embedded_filter( std::weak_ptr< uvc_sensor > raw_depth_ep )
{
    // PROTOTYPE / DEMO: registers the ONE composite option this filter exposes, mirroring
    // d500_temporal_embedded_filter exactly. ds::DS5_HKR_MINZ_CONTROL/rs2_minz_control drive the
    // same physical XU control formerly exposed as the scalar "Improved Close Range Depth"
    // option (close_range_xu_option, since removed - see minz-filter-feature.cpp callers).
    auto opt = std::make_shared< composite_xu_option >(
        raw_depth_ep,
        ds::depth_xu,
        ds::DS5_HKR_MINZ_CONTROL,
        static_cast< uint32_t >( sizeof( rs2_minz_control ) ),
        "HKR MinZ Control (prototype) - use rs2_set/get_composite_option, see rs_hkr_minz_control.h" );
    register_composite_option( RS2_COMPOSITE_OPTION_HKR_MINZ_CONTROL, opt );
}

}  // namespace librealsense
