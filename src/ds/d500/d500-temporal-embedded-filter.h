// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

// PROTOTYPE / DEMO class - not a finalized production control.

#pragma once

#include <src/proc/temporal-embedded-filter.h>
#include <src/uvc-sensor.h>

#include <memory>


namespace librealsense {

// HKR (D555-class) Depth Post-Processing "Temporal Filter" embedded filter. Registers ONE
// composite_xu_option (a generic, reusable class - see src/ds/composite-xu-option.h) under
// RS2_OPTION_HKR_TEMPORAL_FILTER_DPP, in this filter's OWN options container (inherited via
// embedded_filter_base) - NOT directly on d500_depth_sensor. Mirrors
// d500_close_range_embedded_filter's registration pattern exactly (see
// d500-close-range-embedded-filter.h/.cpp).
class d500_temporal_embedded_filter : public temporal_embedded_filter
{
public:
    explicit d500_temporal_embedded_filter( std::weak_ptr< uvc_sensor > raw_depth_ep );

    rs2_embedded_filter_type get_type() const override { return RS2_EMBEDDED_FILTER_TYPE_TEMPORAL; }
};

}  // namespace librealsense
