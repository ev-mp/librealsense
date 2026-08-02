// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

// PROTOTYPE / DEMO class - not a finalized production control ID or wire format.
//
// Generic, reusable composite (multi-field, atomically-exchanged) XU control - NOT named after
// any specific feature. Any future multi-param XU control that needs to exchange a packed
// struct atomically with the device can reuse this exact class; the only per-feature footprint
// is the (extension_unit, ctrl_id, wire_size) triple passed to the constructor and the
// rs2_composite_option_id it gets registered under via the existing
// options_container::register_composite_option (see d500_temporal_embedded_filter, the first
// user of this class).
//
// Implements ONLY composite_option_interface - a fully independent interface with zero
// relationship to librealsense::option/option_interface (no shared base, not even
// extension_snapshot/recordable). This class therefore has no set(float)/query() at all: there
// is nothing here that could be mistaken for a scalar option by generic code walking
// options_container::get_supported_options(), because composite options are never enumerated
// there in the first place (see options_container::get_supported_composite_options(), a
// completely separate registry).
//
// WRAPS (composition, not inheritance) a uvc_xu_option<T> instance for the underlying transport
// plumbing (reuses that existing class as-is, purely for its device/xu/ctrl_id plumbing
// conventions). get_raw()/set_raw()/get_raw_range() perform the actual wire transaction directly
// (their own single get_xu()/set_xu()/get_xu_range() call each) against the device, rather than
// through the wrapped uvc_xu_option<T>, since that class's own set/query are hardwired to
// sizeof(T) (a single scalar), not this control's full, caller-supplied wire_size.

#pragma once

#include <src/composite-option-interface.h>
#include <src/uvc-sensor.h>
#include <src/platform/uvc-option.h>
#include <src/platform/uvc-device.h>

#include <memory>
#include <string>
#include <cstdint>
#include <vector>


namespace librealsense {

class composite_xu_option : public composite_option_interface
{
public:
    composite_xu_option( std::weak_ptr< uvc_sensor > ep,
                         platform::extension_unit xu,
                         uint8_t ctrl_id,
                         uint32_t wire_size,
                         std::string description );

    // composite_option_interface: EXACTLY one get_xu()/set_xu() call - the whole payload
    // travels atomically. This is the non-negotiable HW/FW invariant this class exists for.
    std::vector< uint8_t > get_raw() const override;
    void set_raw( const void * data, size_t size ) override;

    // One get_xu_range() call (a read-only metadata query, not subject to the get_raw/set_raw
    // atomicity contract above) - packs {version=1, min, max, step, def} per the generic
    // convention documented on composite_option_interface::get_raw_range().
    std::vector< uint8_t > get_raw_range() const override;

    bool is_enabled() const override { return true; }
    bool is_read_only() const override { return false; }
    const char * get_description() const override { return _description.c_str(); }

private:
    std::weak_ptr< uvc_sensor > _ep;
    platform::extension_unit _xu;
    uint8_t _ctrl_id;
    uint32_t _wire_size;
    std::string _description;

    // Composition, not inheritance (see file header) - T is an arbitrary fixed-size placeholder;
    // this class never marshals a value through it, only reuses it for its device/xu/ctrl_id
    // transport plumbing conventions.
    uvc_xu_option< uint8_t > _underlying;
};

}  // namespace librealsense
