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

#include <vector>
#include <cstdint>

namespace librealsense {

class composite_option_interface
{
public:
    virtual ~composite_option_interface() = default;

    // Generic dispatch by option id. Each call performs EXACTLY ONE UVC control transaction
    // (one set_xu/get_xu). The bytes are an opaque blob whose layout is documented as a
    // comment on the corresponding rs2_composite_option_id enumerator (see
    // rs_composite_option.h) - the SDK ships no typed struct for it. The caller is
    // responsible for defining its own local struct matching that documented layout and
    // casting to/from it.
    //
    // get_composite_option returns a vector sized exactly to that option's known wire size -
    // the caller has no generic way to know that size in advance, so the SDK owns the
    // allocation (mirrors librealsense::safety_sensor::get_safety_preset).
    virtual void set_composite_option( rs2_composite_option_id option_id, const void * data, uint32_t data_size ) = 0;
    virtual std::vector< uint8_t > get_composite_option( rs2_composite_option_id option_id ) const = 0;
};

MAP_EXTENSION( RS2_EXTENSION_COMPOSITE_OPTIONS, librealsense::composite_option_interface );

}  // namespace librealsense
