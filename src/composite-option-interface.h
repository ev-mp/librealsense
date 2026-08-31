// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

// PROTOTYPE / DEMO extension - not a finalized production interface.
//
// A completely independent, standalone interface backing a "composite option": a multi-field
// control whose fields are exchanged atomically (see include/librealsense2/h/rs_composite_option.h
// for the public entry points this backs). This is NOT a specialization of, and shares NO common
// ancestor with, librealsense::option/option_interface - not even via a common base class. A
// class implementing composite_option_interface (see librealsense::composite_xu_option,
// src/ds/composite-xu-option.h) implements ONLY this interface: there is no set(float)/query()
// anywhere on it, so the entire class of bug that motivated this design (generic code that
// iterates every registered rs2_option and blindly calls get_option()/set_option() on each one,
// unexpectedly throwing when it lands on a composite entry) is structurally impossible, not
// defensively guarded against - a composite option is never reachable through the scalar-option
// enumeration/registry (options_container::get_supported_options()/get_option()) at all.
//
// Composite options live in their own registry, keyed by their own id type
// (rs2_composite_option_id - see rs_composite_option.h), on the SAME options_container that also
// holds ordinary rs2_option-keyed options (see src/core/options-container.h) - two fully separate
// maps on one class, not two classes.

#pragma once

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
    // an opaque blob whose layout is documented on the corresponding rs2_composite_option_id (see
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
    // the layout of rs2_temporal_filter_dpp_range for RS2_COMPOSITE_OPTION_HKR_TEMPORAL_FILTER_DPP
    // (see rs_hkr_temporal_filter_dpp.h), and is meant to generalize the same way to any future
    // composite option's range struct (version field first, then one payload-sized min/max/step/
    // def each).
    virtual std::vector< uint8_t > get_raw_range() const = 0;

    // Real, meaningful metadata for a composite control, just as it is for a scalar option -
    // declared fresh here rather than inherited, since this interface has no relationship to
    // librealsense::option.
    virtual bool is_read_only() const = 0;
    virtual bool is_enabled() const = 0;
    virtual const char * get_description() const = 0;
};

}  // namespace librealsense
