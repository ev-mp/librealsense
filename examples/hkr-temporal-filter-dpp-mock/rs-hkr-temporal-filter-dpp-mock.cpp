// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

// PROTOTYPE / DEMO test scaffolding - NOT a real hardware round trip.
//
// This standalone program proves the E2E behavior of the GENERIC "composite option"
// mechanism (src/ds/structured-xu-control.h, see also include/librealsense2/h/rs_composite_option.h)
// as applied to one concrete composite option - the HKR Temporal Filter DPP control - using a
// minimal in-memory fake standing in for platform::uvc_device::set_xu/get_xu. No physical
// HKR/D555 device is required, present, or touched - this is software-only scaffolding, not a
// hardware test.
//
// What this proves:
//   1. Round-trip correctness: a payload sent via set_raw(void*, size) comes back
//      byte-identical in the SDK-allocated vector returned by get_raw(), once memcpy'd into
//      THIS EXAMPLE'S OWN local struct (hkr_temporal_filter_dpp_layout, below) - exactly the
//      contract of the real rs2_set_composite_option(data, data_size) /
//      rs2_get_composite_option() (which returns an rs2_raw_data_buffer, since the caller has
//      no generic way to know an arbitrary option_id's wire size in advance - mirrors
//      rs2_get_safety_preset). The SDK ships NO typed struct for composite options: this
//      example defines its own struct matching the byte layout documented as a comment on
//      RS2_COMPOSITE_OPTION_HKR_TEMPORAL_FILTER_DPP in
//      include/librealsense2/h/rs_composite_option.h.
//   2. Atomicity: set_raw() performs EXACTLY ONE set_xu() call, and get_raw() performs
//      EXACTLY ONE get_xu() call - i.e. the whole payload always travels as a single UVC
//      transaction, never as separate per-field writes/reads. This is the non-negotiable
//      atomicity requirement from the HKR/FW spec.
//
// This exercises the exact same production class (librealsense::xu_structured_control) that
// d500_depth_sensor's generic set_composite_option/get_composite_option dispatch
// (src/ds/d500/d500-device.cpp) delegates to for RS2_COMPOSITE_OPTION_HKR_TEMPORAL_FILTER_DPP
// - only the transport (platform::uvc_device) is faked.

#include <src/ds/structured-xu-control.h>

#include <cassert>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>

using namespace librealsense;

namespace {

// This example's OWN local struct - NOT an SDK type - matching the wire layout documented as
// a comment on RS2_COMPOSITE_OPTION_HKR_TEMPORAL_FILTER_DPP in rs_composite_option.h. Any
// real application would define its own copy of this exact struct the same way; the SDK does
// not provide one.
#pragma pack( push, 1 )
struct hkr_temporal_filter_dpp_layout
{
    int32_t enabled;           // 0 = Off, 1 = On
    float smooth_alpha;        // range [0,1], default 0.4, step 0.01
    int32_t smooth_delta;      // range [1,100], default 20, step 1
    int32_t persistency_index; // range [0,8], default 3, step 1
};
#pragma pack( pop )

// Minimal fake transport standing in for the real UVC backend. Implements just enough of
// platform::uvc_device to exercise xu_structured_control::get_raw/set_raw. Everything else is
// unused by this test and throws loudly if ever invoked - this class is test scaffolding
// only, never a real hardware backend, and must not be confused with one.
class fake_uvc_device : public platform::uvc_device
{
public:
    // --- call counters: this is the atomicity proof ---
    mutable int get_xu_calls = 0;
    int set_xu_calls = 0;

    bool set_xu( const platform::extension_unit & /*xu*/, uint8_t /*ctrl*/, const uint8_t * data, int len ) override
    {
        ++set_xu_calls;
        _storage.assign( data, data + len );
        return true;
    }

    bool get_xu( const platform::extension_unit & /*xu*/, uint8_t /*ctrl*/, uint8_t * data, int len ) const override
    {
        ++get_xu_calls;
        if( (int)_storage.size() != len )
            throw std::runtime_error( "fake_uvc_device: get_xu length mismatch (set_xu was never called with this length)" );
        std::memcpy( data, _storage.data(), len );
        return true;
    }

    // --- everything else: not exercised by this test ---
    void probe_and_commit( platform::stream_profile, platform::frame_callback, int ) override { fail(); }
    void stream_on( std::function< void( const notification & ) > ) override { fail(); }
    void start_callbacks() override { fail(); }
    void stop_callbacks() override { fail(); }
    void close( platform::stream_profile ) override { fail(); }
    void set_power_state( platform::power_state ) override {}
    platform::power_state get_power_state() const override { return platform::D0; }
    void init_xu( const platform::extension_unit & ) override {}
    platform::control_range get_xu_range( const platform::extension_unit &, uint8_t, int ) const override { fail(); return {}; }
    bool get_pu( rs2_option, int32_t & ) const override { fail(); return false; }
    bool set_pu( rs2_option, int32_t ) override { fail(); return false; }
    platform::control_range get_pu_range( rs2_option ) const override { fail(); return {}; }
    std::vector< platform::stream_profile > get_profiles() const override { fail(); return {}; }
    void lock() const override {}
    void unlock() const override {}
    std::string get_device_location() const override { fail(); return {}; }
    platform::usb_spec get_usb_specification() const override { fail(); return {}; }
    bool is_platform_jetson() const override { return false; }

private:
    std::vector< uint8_t > _storage;

    static void fail() { throw std::runtime_error( "fake_uvc_device: unexpected call - not exercised by this test" ); }
};

}  // namespace


int main()
try
{
    fake_uvc_device dev;

    // Same (extension_unit, ctrl-id) VALUES that the real d500_depth_sensor registers for
    // RS2_COMPOSITE_OPTION_HKR_TEMPORAL_FILTER_DPP in production (ds::depth_xu /
    // ds::DS5_HKR_TEMPORAL_FILTER_DPP, see src/ds/ds-private.h and src/ds/d500/d500-device.cpp)
    // - duplicated here as plain literals rather than pulling in ds-private.h, which drags in
    // unrelated FW-update/HWM dependencies that this standalone, transport-mocked example has
    // no need for. The identity of the xu/ctrl-id is incidental to what this example proves
    // (round-trip correctness + exactly-one-wire-call atomicity): the fake transport below
    // never inspects them.
    platform::extension_unit depth_xu = { 0, 3, 2,
        { 0xC9606CCB, 0x594C, 0x4D25, { 0xaf, 0x47, 0xcc, 0xc4, 0x96, 0x43, 0x59, 0x95 } } };
    const uint8_t DS5_HKR_TEMPORAL_FILTER_DPP = 0x30;

    // Generic control, registered for exactly this one composite option's wire size - mirrors
    // how d500_depth_sensor populates its _structured_controls registry.
    xu_structured_control control( depth_xu, DS5_HKR_TEMPORAL_FILTER_DPP, sizeof( hkr_temporal_filter_dpp_layout ) );

    hkr_temporal_filter_dpp_layout sent{};
    sent.enabled = 1;
    sent.smooth_alpha = 0.4f;
    sent.smooth_delta = 20;
    sent.persistency_index = 3;

    // Caller casts to/from void* - exactly the contract of rs2_set/get_composite_option. SET
    // still takes a caller-owned buffer (the caller/producer already knows sizeof() of what
    // it's sending). GET returns an SDK-owned, correctly-sized vector - the caller has no
    // generic way to know a given option_id's wire size in advance - mirroring
    // rs2_get_safety_preset/rs2::safety_sensor::get_safety_preset.
    control.set_raw( dev, &sent, sizeof( sent ) );

    std::vector< uint8_t > bytes = control.get_raw( dev );
    if( bytes.size() != sizeof( hkr_temporal_filter_dpp_layout ) )
        throw std::runtime_error( "get_raw returned an unexpected payload size" );

    hkr_temporal_filter_dpp_layout received{};
    std::memcpy( &received, bytes.data(), sizeof( received ) );

    // 1) Round-trip correctness.
    bool round_trip_ok = ( received.enabled == sent.enabled )
                       && ( received.smooth_alpha == sent.smooth_alpha )
                       && ( received.smooth_delta == sent.smooth_delta )
                       && ( received.persistency_index == sent.persistency_index );

    // 2) Atomicity - exactly one wire call per logical operation, never split per-field.
    bool atomicity_ok = ( dev.set_xu_calls == 1 ) && ( dev.get_xu_calls == 1 );

    std::cout << "sent:     enabled=" << sent.enabled
              << " smooth_alpha=" << sent.smooth_alpha
              << " smooth_delta=" << sent.smooth_delta
              << " persistency_index=" << sent.persistency_index << std::endl;
    std::cout << "received: enabled=" << received.enabled
              << " smooth_alpha=" << received.smooth_alpha
              << " smooth_delta=" << received.smooth_delta
              << " persistency_index=" << received.persistency_index << std::endl;
    std::cout << "set_xu_calls=" << dev.set_xu_calls << "  get_xu_calls=" << dev.get_xu_calls << std::endl;

    assert( round_trip_ok );
    assert( atomicity_ok );
    if( ! round_trip_ok || ! atomicity_ok )
    {
        std::cerr << "FAIL" << std::endl;
        return 1;
    }

    std::cout << "PASS: round-trip correctness AND atomicity (exactly 1 set_xu + 1 get_xu) both verified." << std::endl;
    return 0;
}
catch( const std::exception & e )
{
    std::cerr << "FAIL: unexpected exception: " << e.what() << std::endl;
    return 1;
}
