// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

// PROTOTYPE / DEMO test scaffolding - NOT a real hardware round trip.
//
// This standalone program proves the E2E behavior of the GENERIC "composite option" mechanism
// (see include/librealsense2/h/rs_composite_option.h and src/composite-option-interface.h) as
// applied to one concrete composite option - RS2_COMPOSITE_OPTION_HKR_TEMPORAL_FILTER_DPP - using
// a minimal in-memory fake composite_option_interface implementation standing in for a real
// device-backed librealsense::composite_xu_option (src/ds/composite-xu-option.h). No physical
// HKR/D555 device is required, present, or touched - this is software-only scaffolding, not a
// hardware test.
//
// Composite options are a fully independent identity/registry space from ordinary rs2_option
// scalar options (see rs_composite_option.h) - this test drives the mechanism entirely through
// the REAL PUBLIC ENTRY POINTS for THAT space:
//   - the C API: rs2_set_composite_option / rs2_get_composite_option
//   - the C++ wrapper: rs2::options::get_composite_option()/set_composite_option() - direct
//     calls, no handle/wrapper type and no is<T>()/as<T>() casting (composite options are never
//     reached by casting a scalar option - they live in their own registry from the start).
// The "device" being driven is a minimal fake librealsense::composite_option_interface
// implementation (fake_composite_option, below), registered on a minimal fake
// librealsense::options_interface (fake_options_container) in ITS OWN composite-option registry
// (kept completely separate from that container's - empty - scalar rs2_option registry), wrapped
// in the same rs2_options C-struct that a real rs2::sensor/rs2::embedded_filter wraps (see
// src/proc/synthetic-stream.h) - so the public API code path exercised here is identical to what
// a real sensor goes through, only the very last step (the simulated "wire") is faked.
//
// What this proves:
//   1. Round-trip correctness: a payload sent via rs2_set_composite_option comes back
//      byte-identical from rs2_get_composite_option, once cast into this example's own
//      rs2_temporal_filter_dpp_config (the SDK DOES ship this one struct publicly - see
//      include/librealsense2/h/rs_hkr_temporal_filter_dpp.h).
//   2. Atomicity: exactly one set_raw()/get_raw() call reaches the fake "wire" per logical
//      rs2_set_composite_option()/rs2_get_composite_option() call - i.e. the whole payload
//      always travels as a single transaction, never as separate per-field writes/reads. This
//      is the non-negotiable atomicity requirement from the HKR/FW spec, and is exactly the
//      contract librealsense::composite_xu_option::get_raw()/set_raw() implement via a single
//      get_xu()/set_xu() call each against the real device.

#include <librealsense2/rs.hpp>

#include <src/composite-option-interface.h>
#include <src/core/options-interface.h>
#include <src/proc/synthetic-stream.h>

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>

using namespace librealsense;

namespace {

// Minimal fake composite option standing in for librealsense::composite_xu_option. Implements
// ONLY composite_option_interface - no relationship whatsoever to librealsense::option -
// matching the real class exactly: there is no set(float)/query() anywhere to (mis)use.
// get_raw()/set_raw() are this test's "wire": each is called exactly once per
// rs2_get_composite_option()/rs2_set_composite_option() call, and the counters below are the
// atomicity proof.
class fake_composite_option : public composite_option_interface
{
public:
    // --- call counters: this is the atomicity proof ---
    mutable int get_calls = 0;
    int set_calls = 0;

    bool is_enabled() const override { return true; }
    bool is_read_only() const override { return false; }
    const char * get_description() const override { return "HKR Temporal Filter DPP (fake, for testing)"; }

    // composite_option_interface: the fake "wire" - exactly one call per logical operation.
    std::vector< uint8_t > get_raw() const override
    {
        ++get_calls;
        return _storage;
    }

    // get_range()-style query, not exercised by this test's round-trip/atomicity assertions -
    // just needs to satisfy the interface. Real behavior lives in
    // librealsense::composite_xu_option::get_raw_range() (one get_xu_range() call).
    std::vector< uint8_t > get_raw_range() const override { return {}; }

    void set_raw( const void * data, size_t size ) override
    {
        ++set_calls;
        auto p = reinterpret_cast< const uint8_t * >( data );
        _storage.assign( p, p + size );
    }

private:
    std::vector< uint8_t > _storage;
};

// Minimal fake options container: implements librealsense::options_interface directly (no
// dependency on options_container, so this executable has no link-time dependency on anything
// beyond the genuine extern "C" API exported by realsense2 - see CMakeLists.txt). Holds no
// scalar rs2_option at all - only exactly one registered composite option:
// RS2_COMPOSITE_OPTION_HKR_TEMPORAL_FILTER_DPP - demonstrating the two registries are completely
// separate (this container legitimately supports zero scalar options).
class fake_options_container : public options_interface
{
public:
    explicit fake_options_container( std::shared_ptr< fake_composite_option > opt )
        : _opt( std::move( opt ) )
    {
    }

    // Scalar rs2_option side: intentionally empty - this container exposes no scalar options.
    option & get_option( rs2_option ) override { throw std::runtime_error( "fake_options_container: no scalar options" ); }
    const option & get_option( rs2_option ) const override { throw std::runtime_error( "fake_options_container: no scalar options" ); }
    bool supports_option( rs2_option ) const override { return false; }
    std::vector< rs2_option > get_supported_options() const override { return {}; }
    std::string const & get_option_name( rs2_option ) const override { return _name; }

    // Composite-option side: the one id this test exercises.
    composite_option_interface & get_composite_option( rs2_composite_option_id id ) override
    {
        return const_cast< composite_option_interface & >(
            const_cast< const fake_options_container * >( this )->get_composite_option( id ) );
    }
    const composite_option_interface & get_composite_option( rs2_composite_option_id id ) const override
    {
        if( id != RS2_COMPOSITE_OPTION_HKR_TEMPORAL_FILTER_DPP )
            throw std::runtime_error( "fake_options_container: unsupported composite option id" );
        return *_opt;
    }
    bool supports_composite_option( rs2_composite_option_id id ) const override
    {
        return id == RS2_COMPOSITE_OPTION_HKR_TEMPORAL_FILTER_DPP;
    }
    std::vector< rs2_composite_option_id > get_supported_composite_options() const override
    {
        return { RS2_COMPOSITE_OPTION_HKR_TEMPORAL_FILTER_DPP };
    }
    std::string const & get_composite_option_name( rs2_composite_option_id ) const override { return _name; }

    rsutils::subscription register_options_changed_callback( options_watcher::callback && ) override
    {
        return rsutils::subscription();
    }

    // recordable<options_interface>: not exercised by this test.
    void create_snapshot( std::shared_ptr< options_interface > & snapshot ) const override { snapshot.reset(); }
    void enable_recording( std::function< void( const options_interface & ) > ) override {}

private:
    std::shared_ptr< fake_composite_option > _opt;
    std::string _name = "HKR Temporal Filter DPP";
};

// Lets this standalone test call the protected rs2::options(rs2_options*) constructor - the
// same one rs2::sensor/rs2::embedded_filter use internally - so
// get_composite_option()/set_composite_option() can be exercised exactly as a real
// sensor/embedded_filter would use them.
class fake_options_handle : public rs2::options
{
public:
    explicit fake_options_handle( rs2_options * o )
        : options( o )
    {
    }
};

}  // namespace


int main()
try
{
    auto fake_opt = std::make_shared< fake_composite_option >();
    fake_options_container container( fake_opt );
    rs2_options wrapper( &container );

    rs2_temporal_filter_dpp_config sent{};
    sent.enabled = 1;
    sent.smooth_alpha = 0.4f;
    sent.smooth_delta = 20;
    sent.persistency_index = 3;

    // --- 1) Exercise the raw C API path: rs2_set_composite_option / rs2_get_composite_option ---
    rs2_error * e = nullptr;
    rs2_set_composite_option( &wrapper, RS2_COMPOSITE_OPTION_HKR_TEMPORAL_FILTER_DPP, &sent, sizeof( sent ), &e );
    rs2::error::handle( e );

    auto buffer = rs2_get_composite_option( &wrapper, RS2_COMPOSITE_OPTION_HKR_TEMPORAL_FILTER_DPP, &e );
    rs2::error::handle( e );
    std::shared_ptr< const rs2_raw_data_buffer > buffer_guard( buffer, rs2_delete_raw_data );

    auto size = rs2_get_raw_data_size( buffer, &e );
    rs2::error::handle( e );
    if( (size_t)size != sizeof( rs2_temporal_filter_dpp_config ) )
        throw std::runtime_error( "rs2_get_composite_option returned an unexpected payload size" );

    rs2_temporal_filter_dpp_config received{};
    auto const * raw = rs2_get_raw_data( buffer, &e );
    rs2::error::handle( e );
    std::memcpy( &received, raw, sizeof( received ) );

    // --- 2) Exercise the direct C++ get_composite_option()/set_composite_option() path on the
    //        SAME underlying options object - no wrapper/handle type, no casting ---
    fake_options_handle handle( &wrapper );

    auto supported = handle.get_supported_composite_options();
    bool supports_temporal_filter_dpp
        = std::find( supported.begin(), supported.end(), RS2_COMPOSITE_OPTION_HKR_TEMPORAL_FILTER_DPP ) != supported.end();
    if( ! supports_temporal_filter_dpp )
        throw std::runtime_error( "get_supported_composite_options() unexpectedly missing RS2_COMPOSITE_OPTION_HKR_TEMPORAL_FILTER_DPP" );

    rs2_temporal_filter_dpp_config sent2 = sent;
    sent2.persistency_index = 7;  // change one field to prove this second round trip is independent
    handle.set_composite_option( RS2_COMPOSITE_OPTION_HKR_TEMPORAL_FILTER_DPP, &sent2, sizeof( sent2 ) );

    auto bytes2 = handle.get_composite_option( RS2_COMPOSITE_OPTION_HKR_TEMPORAL_FILTER_DPP );
    if( bytes2.size() != sizeof( rs2_temporal_filter_dpp_config ) )
        throw std::runtime_error( "get_composite_option() returned an unexpected payload size" );

    rs2_temporal_filter_dpp_config received2{};
    std::memcpy( &received2, bytes2.data(), sizeof( received2 ) );

    // 1) Round-trip correctness - both paths.
    bool round_trip_ok = ( received.enabled == sent.enabled ) && ( received.smooth_alpha == sent.smooth_alpha )
                       && ( received.smooth_delta == sent.smooth_delta )
                       && ( received.persistency_index == sent.persistency_index )
                       && ( received2.enabled == sent2.enabled ) && ( received2.smooth_alpha == sent2.smooth_alpha )
                       && ( received2.smooth_delta == sent2.smooth_delta )
                       && ( received2.persistency_index == sent2.persistency_index );

    // 2) Atomicity - exactly one wire call per logical operation (2 sets + 2 gets total across
    // both paths above), never split per-field.
    bool atomicity_ok = ( fake_opt->set_calls == 2 ) && ( fake_opt->get_calls == 2 );

    std::cout << "[C API]  sent:     enabled=" << sent.enabled << " smooth_alpha=" << sent.smooth_alpha
              << " smooth_delta=" << sent.smooth_delta << " persistency_index=" << sent.persistency_index
              << std::endl;
    std::cout << "[C API]  received: enabled=" << received.enabled << " smooth_alpha=" << received.smooth_alpha
              << " smooth_delta=" << received.smooth_delta << " persistency_index=" << received.persistency_index
              << std::endl;
    std::cout << "[C++ direct] sent:     enabled=" << sent2.enabled << " smooth_alpha=" << sent2.smooth_alpha
              << " smooth_delta=" << sent2.smooth_delta << " persistency_index=" << sent2.persistency_index
              << std::endl;
    std::cout << "[C++ direct] received: enabled=" << received2.enabled
              << " smooth_alpha=" << received2.smooth_alpha << " smooth_delta=" << received2.smooth_delta
              << " persistency_index=" << received2.persistency_index << std::endl;
    std::cout << "set_calls=" << fake_opt->set_calls << "  get_calls=" << fake_opt->get_calls << std::endl;

    assert( round_trip_ok );
    assert( atomicity_ok );
    if( ! round_trip_ok || ! atomicity_ok )
    {
        std::cerr << "FAIL" << std::endl;
        return 1;
    }

    std::cout << "PASS: round-trip correctness AND atomicity (exactly 1 set + 1 get per logical "
                 "operation) both verified, through the real public API (rs2_set/get_composite_option "
                 "and rs2::options::get_composite_option()/set_composite_option())."
              << std::endl;
    return 0;
}
catch( const rs2::error & e )
{
    std::cerr << "FAIL: librealsense error: " << e.what() << std::endl;
    return 1;
}
catch( const std::exception & e )
{
    std::cerr << "FAIL: unexpected exception: " << e.what() << std::endl;
    return 1;
}
