// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

#include "d500-temporal-embedded-filter.h"
#include "ds/ds-private.h"
#include <src/ds/composite-xu-option.h>

#include <librealsense2/h/rs_composite_option.h>
#include <librealsense2/h/rs_hkr_temporal_filter_dpp.h>

namespace librealsense {

d500_temporal_embedded_filter::d500_temporal_embedded_filter( std::weak_ptr< uvc_sensor > raw_depth_ep )
{
    // PROTOTYPE / DEMO: registers the ONE composite option this filter exposes. Adding another
    // composite option elsewhere in the SDK never needs a new class - just another
    // composite_xu_option instance with its own (xu, ctrl_id, wire_size, rs2_composite_option_id).
    auto opt = std::make_shared< composite_xu_option >(
        raw_depth_ep,
        ds::depth_xu,
        ds::DS5_HKR_TEMPORAL_FILTER_DPP,
        static_cast< uint32_t >( sizeof( rs2_temporal_filter_dpp_config ) ),
        "HKR Temporal Filter DPP (prototype) - use rs2_set/get_composite_option, see rs_hkr_temporal_filter_dpp.h" );
    register_composite_option( RS2_COMPOSITE_OPTION_HKR_TEMPORAL_FILTER_DPP, opt );
}

}  // namespace librealsense
