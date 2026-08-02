// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

// PROTOTYPE / DEMO class - not a finalized production control ID or wire format.
//
// Generic single-XU-control transport: owns the (extension_unit, control-id, wire-size)
// identity of ONE composite XU control and performs a raw byte copy to/from the device. This
// is the mechanism that backs every rs2_composite_option_id registered by a sensor (see
// composite-option-interface.h) - there is one instance of this class per composite option,
// not one bespoke class per feature.
//
// HARD invariant (per HW/FW spec): all fields of a composite option's payload MUST land on
// the device in ONE indivisible UVC transaction. get_raw()/set_raw() below are each
// implemented as EXACTLY ONE get_xu()/set_xu() call - never split into per-field calls. This
// is the atomicity property this whole "structured API" prototype exists to guarantee.
//
// This class is deliberately transport-agnostic: it operates directly on a
// platform::uvc_device&, with no dependency on uvc_sensor/backend/power-state management.
// That keeps it trivially testable against a fake/mock platform::uvc_device (see
// examples/hkr-temporal-filter-dpp-mock) without any live HKR/D555 hardware. Production
// callers (e.g. d500_depth_sensor) are expected to obtain the platform::uvc_device& via their
// own invoke_powered()/power-management path (see uvc_xu_option in src/platform/uvc-option.h
// for the established pattern) and then delegate to this class for the actual XU transaction.

#pragma once

#include <src/platform/uvc-device.h>

namespace librealsense {

class xu_structured_control
{
public:
    xu_structured_control( platform::extension_unit xu, uint8_t ctrl_id, uint32_t wire_size )
        : _xu( std::move( xu ) )
        , _ctrl_id( ctrl_id )
        , _wire_size( wire_size )
    {
    }

    uint32_t wire_size() const { return _wire_size; }

    // Exactly one get_xu() call. Throws if data_size is smaller than this control's wire size.
    void get_raw( platform::uvc_device & dev, void * data, uint32_t data_size ) const;

    // Exactly one set_xu() call - the whole wire_size() payload sent together. Throws if
    // data_size does not exactly match this control's wire size.
    void set_raw( platform::uvc_device & dev, const void * data, uint32_t data_size ) const;

private:
    platform::extension_unit _xu;
    uint8_t _ctrl_id;
    uint32_t _wire_size;
};

}  // namespace librealsense
