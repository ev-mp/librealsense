// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

// PROTOTYPE / DEMO class - not a finalized production control ID or wire format.
//
// Generic, reusable composite (multi-field, atomically-exchanged) XU control - NOT named after
// any specific feature. Any future multi-param XU control that needs to exchange a packed
// struct atomically with the device can reuse this exact class; the only per-feature footprint
// is the (extension_unit, ctrl_id, wire_size) triple passed to the constructor and the
// rs2_option id it gets registered under via the existing options_container::register_option
// (see d500_temporal_embedded_filter, the first user of this class).
//
// Implements two INDEPENDENT interfaces via plain multiple inheritance - not a diamond, since
// `option` and `composite_option_interface` share no common ancestor:
//   - option: the "scalar" interface every registered option must satisfy. Scalar access
//     (set(float)/query()) is not meaningful for a multi-field composite control, so both are
//     hard-blocked (always throw) - see set()/query() below. is_enabled() deliberately keeps its
//     normal, real meaning (genuine runtime availability) rather than being repurposed as this
//     blocking flag, per design: the two concerns must not be conflated.
//   - composite_option_interface: the actual atomic get/set entry points backing
//     rs2_get_composite_option/rs2_set_composite_option.
//
// WRAPS (composition, not inheritance) a uvc_xu_option<T> instance for the underlying transport
// plumbing (metadata pass-through only - see get_range()); reuses that existing class as-is.
// get_raw()/set_raw() perform the actual wire transaction directly (their own single
// get_xu()/set_xu() call each) rather than through the wrapped uvc_xu_option<T>, since that
// class's own set/query are hardwired to sizeof(T) (a single scalar), not this control's full,
// caller-supplied wire_size.

#pragma once

#include <src/option.h>
#include <src/composite-option-interface.h>
#include <src/uvc-sensor.h>
#include <src/platform/uvc-option.h>
#include <src/platform/uvc-device.h>

#include <functional>
#include <memory>
#include <string>
#include <cstdint>
#include <vector>


namespace librealsense {

class composite_xu_option
    : public option
    , public composite_option_interface
{
public:
    composite_xu_option( std::weak_ptr< uvc_sensor > ep,
                         platform::extension_unit xu,
                         uint8_t ctrl_id,
                         uint32_t wire_size,
                         std::string description );

    // option: scalar access is not supported for a composite (multi-field) control - use
    // get_raw()/set_raw() (composite_option_interface) via rs2_get_composite_option/
    // rs2_set_composite_option instead. Always throws - never split into per-field scalar
    // reads/writes, which would defeat the whole point of atomic exchange.
    void set( float value ) override;
    float query() const override;

    // Metadata-only queries - no value marshaling involved, so there is no reason to block
    // these; pass through to the wrapped uvc_xu_option<T>'s existing behavior where sensible.
    option_range get_range() const override;
    bool is_enabled() const override { return true; }
    bool is_read_only() const override { return false; }
    const char * get_description() const override { return _description.c_str(); }
    void enable_recording( std::function< void( const option & ) > record_action ) override
    {
        _recording_function = record_action;
    }

    // composite_option_interface: EXACTLY one get_xu()/set_xu() call - the whole payload
    // travels atomically. This is the non-negotiable HW/FW invariant this class exists for.
    std::vector< uint8_t > get_raw() const override;
    void set_raw( const void * data, size_t size ) override;

    // One get_xu_range() call (a read-only metadata query, not subject to the get_raw/set_raw
    // atomicity contract above) - packs {version=1, min, max, step, def} per the generic
    // convention documented on composite_option_interface::get_raw_range().
    std::vector< uint8_t > get_raw_range() const override;

private:
    std::weak_ptr< uvc_sensor > _ep;
    platform::extension_unit _xu;
    uint8_t _ctrl_id;
    uint32_t _wire_size;
    std::string _description;
    std::function< void( const option & ) > _recording_function = []( const option & ) {};

    // Composition, not inheritance (see file header) - T is an arbitrary fixed-size placeholder;
    // this class never marshals a value through it, only reuses it for the get_range() pass-through.
    uvc_xu_option< uint8_t > _underlying;
};

}  // namespace librealsense
