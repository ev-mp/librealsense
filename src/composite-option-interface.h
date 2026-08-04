// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

// PROTOTYPE / DEMO extension - not a finalized production interface.
// Generic per-option interface backing a "composite option": an rs2_option that is a
// multi-field, atomically-exchanged control (see include/librealsense2/h/rs_composite_option.h
// for the public entry points this backs). Deliberately independent of librealsense::option -
// the two interfaces share no common ancestor, so a class implementing both (see
// librealsense::composite_xu_option, src/ds/composite-xu-option.h) does so via plain multiple
// inheritance, never a diamond.
//
// Unlike the earlier iteration of this mechanism, dispatch by id no longer happens here: each
// composite option is registered as its own librealsense::option (via the existing
// options_container::register_option, keyed by rs2_option - see d500_temporal_embedded_filter)
// and this interface is queried per *option* (via As<composite_option_interface>(&option)), not
// per *sensor*. There is no per-feature footprint in this file at all.

#pragma once

#include "core/extension.h"
#include <librealsense2/h/rs_types.h>

#include <vector>
#include <cstdint>
#include <cstddef>

namespace librealsense {

class composite_option_interface
{
public:
    virtual ~composite_option_interface() = default;

    // Each call performs EXACTLY ONE UVC control transaction (one set_xu/get_xu). The bytes are
    // an opaque blob whose layout is documented on the corresponding rs2_option (see
    // rs_composite_option.h / rs_hkr_temporal_filter_dpp.h).
    //
    // get_raw returns a vector sized exactly to this option's known wire size - the caller has
    // no generic way to know that size in advance, so the SDK owns the allocation (mirrors
    // librealsense::safety_sensor::get_safety_preset).
    virtual std::vector< uint8_t > get_raw() const = 0;
    virtual void set_raw( const void * data, size_t size ) = 0;

    // Backs rs2_get_composite_option_range. Generic, versioned convention (reusable for ANY
    // composite option, not just this one): a 4-byte little-endian "unsigned int version"
    // (currently always 1) followed by four back-to-back payloads, each the option's normal
    // wire size (as returned by get_raw()) - min, max, step, def, in that order. This is exactly
    // the layout of rs2_temporal_filter_dpp_range for RS2_OPTION_HKR_TEMPORAL_FILTER_DPP (see
    // rs_hkr_temporal_filter_dpp.h), and is meant to generalize the same way to any future
    // composite option's range struct (version field first, then one payload-sized min/max/step/
    // def each).
    virtual std::vector< uint8_t > get_raw_range() const = 0;
};

MAP_EXTENSION( RS2_EXTENSION_COMPOSITE_OPTION, librealsense::composite_option_interface );

}  // namespace librealsense
