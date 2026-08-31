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
// options container - NOT directly on d500_depth_sensor.
//
// Replaces the old scalar-only "Improved Close Range Depth" option (same physical XU control)
// with the 7-field composite one. RS2_OPTION_EMBEDDED_FILTER_ENABLED is no longer registered for
// this filter as a result - a generic caller that assumes every filter has it will throw here.
class d500_minz_embedded_filter : public close_range_embedded_filter
{
public:
    explicit d500_minz_embedded_filter( std::weak_ptr< uvc_sensor > raw_depth_ep );

    rs2_embedded_filter_type get_type() const override { return RS2_EMBEDDED_FILTER_TYPE_CLOSE_RANGE; }
};

}  // namespace librealsense
