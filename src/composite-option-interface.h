// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

// PROTOTYPE / DEMO extension - not a finalized production interface.
// Generic dispatch surface for "composite options": a sensor implementing this interface can
// serve ANY number of composite (multi-field, packed, atomically-exchanged) XU controls,
// selected by rs2_composite_option_id - there is no per-feature interface/method. See
// include/librealsense2/h/rs_composite_option.h for the public entry points this backs.

#pragma once

#include "core/extension.h"
#include <librealsense2/h/rs_composite_option.h>

namespace librealsense {

class composite_option_interface
{
public:
    virtual ~composite_option_interface() = default;

    // Generic dispatch by option id. Each call performs EXACTLY ONE UVC control transaction
    // (one set_xu/get_xu). data/data_size are an opaque byte blob whose layout is defined by
    // whichever typed struct corresponds to option_id (e.g. rs2_temporal_filter_dpp_config for
    // RS2_COMPOSITE_OPTION_HKR_TEMPORAL_FILTER_DPP) - the caller is responsible for casting.
    virtual void set_composite_option( rs2_composite_option_id option_id, const void * data, uint32_t data_size ) = 0;
    virtual void get_composite_option( rs2_composite_option_id option_id, void * data, uint32_t * data_size ) const = 0;
};

MAP_EXTENSION( RS2_EXTENSION_COMPOSITE_OPTIONS, librealsense::composite_option_interface );

}  // namespace librealsense
