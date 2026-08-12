// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

// PROTOTYPE / DEMO class - not a finalized production control.

#pragma once

#include <src/proc/close-range-embedded-filter.h>
#include <src/uvc-sensor.h>

#include <memory>


namespace librealsense {

// HKR/D5X5 MinZ control, registered via the generic composite-option mechanism (see
// src/ds/composite-xu-option.h) under RS2_COMPOSITE_OPTION_HKR_MINZ_CONTROL, in this filter's OWN
// options container (inherited via embedded_filter_base) - NOT directly on d500_depth_sensor.
// Mirrors d500_temporal_embedded_filter's registration pattern exactly.
//
// This addresses the SAME physical XU control (unit 3, selector 0x14) that used to be exposed as
// "Improved Close Range Depth" - a scalar, enable-only RS2_OPTION_EMBEDDED_FILTER_ENABLED option
// (close_range_xu_option, since removed). Consolidated onto this composite option instead, which
// exposes all 5 fields (enable, downscale_ratio, disparity_shift, threshold, threshold_mode)
// atomically rather than just enable. PROTOTYPE/DEMO PR: note this means
// RS2_OPTION_EMBEDDED_FILTER_ENABLED is no longer registered for this filter, so any generic
// caller that assumed every embedded filter has it (e.g. common/embedded-filter-model.cpp in the
// viewer) will throw for this one - a known, accepted consequence of the consolidation.
class d500_minz_embedded_filter : public close_range_embedded_filter
{
public:
    explicit d500_minz_embedded_filter( std::weak_ptr< uvc_sensor > raw_depth_ep );

    rs2_embedded_filter_type get_type() const override { return RS2_EMBEDDED_FILTER_TYPE_CLOSE_RANGE; }
};

}  // namespace librealsense
