// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

// PROTOTYPE / DEMO test scaffolding - NOT a real hardware round trip.
//
// This standalone program proves the E2E behavior of the GENERIC "composite option" mechanism
// (see include/librealsense2/h/rs_composite_option.h and src/composite-option-interface.h) as
// applied to one concrete composite option - RS2_OPTION_HKR_TEMPORAL_FILTER_DPP - using a
// minimal in-memory fake option implementation standing in for a real device-backed
// librealsense::composite_xu_option (src/ds/composite-xu-option.h). No physical HKR/D555 device
// is required, present, or touched - this is software-only scaffolding, not a hardware test.
//
// Unlike the earlier iteration of this example (which called an internal transport class,
// xu_structured_control::get_raw/set_raw, directly - bypassing the public API entirely), this
// version drives the mechanism through the REAL PUBLIC ENTRY POINTS:
//   - the C API: rs2_set_composite_option / rs2_get_composite_option
//   - the C++ wrapper: rs2::options::option_ref() -> is<rs2::composite_option>() ->
//     as<rs2::composite_option>() -> composite_option::set()/get(), mirroring exactly how
//     rs2::frame is cast to rs2::video_frame etc.
// The "device" being driven is a minimal fake librealsense::option +
// librealsense::composite_option_interface implementation (fake_composite_option, below),
// registered on a minimal fake librealsense::options_interface (fake_options_container),
// wrapped in the same rs2_options C-struct that a real rs2::sensor/rs2::embedded_filter wraps
// (see src/proc/synthetic-stream.h) - so the public API code path exercised here is identical
// to what a real sensor goes through, only the very last step (the simulated "wire") is faked.
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

#include <src/option.h>
#include <src/composite-option-interface.h>
#include <src/core/options-interface.h>
#include <src/proc/synthetic-stream.h>

#include <cassert>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>

using namespace librealsense;

namespace {

// Minimal fake option standing in for librealsense::composite_xu_option. Implements both
// `option` (the "scalar" interface every registered option must satisfy - scalar access is
// hard-blocked, exactly like the real composite_xu_option) and `composite_option_interface`
// (the actual get/set entry points), via plain multiple inheritance - not a diamond, since the
// two share no common ancestor. get_raw()/set_raw() are this test's "wire": each is called
// exactly once per rs2_get_composite_option()/rs2_set_composite_option() call, and the counters
// below are the atomicity proof.
class fake_composite_option : public option, public composite_option_interface
{
public:
    // --- call counters: this is the atomicity proof ---
    mutable int get_calls = 0;
    int set_calls = 0;

    // option: scalar access is not supported for a composite (multi-field) control - matches
    // composite_xu_option::set/query exactly.
    void set( float ) override { throw not_implemented_exception( "composite option - use rs2_set_composite_option" ); }
    float query() const override { throw not_implemented_exception( "composite option - use rs2_get_composite_option" ); }
    option_range get_range() const override { return { 0.f, 0.f, 0.f, 0.f }; }
    bool is_enabled() const override { return true; }
    bool is_read_only() const override { return false; }
    const char * get_description() const override { return "HKR Temporal Filter DPP (fake, for testing)"; }
    void enable_recording( std::function< void( const option & ) > ) override {}

    // recordable<option>: not exercised by this test.
    void create_snapshot( std::shared_ptr< option > & snapshot ) const override { snapshot.reset(); }

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
// beyond the genuine extern "C" API exported by realsense2 - see CMakeLists.txt), holding
// exactly one registered option: RS2_OPTION_HKR_TEMPORAL_FILTER_DPP.
class fake_options_container : public options_interface
{
public:
    explicit fake_options_container( std::shared_ptr< fake_composite_option > opt )
        : _opt( std::move( opt ) )
    {
    }

    option & get_option( rs2_option id ) override
    {
        return const_cast< option & >( const_cast< const fake_options_container * >( this )->get_option( id ) );
    }
    const option & get_option( rs2_option id ) const override
    {
        if( id != RS2_OPTION_HKR_TEMPORAL_FILTER_DPP )
            throw std::runtime_error( "fake_options_container: unsupported option id" );
        return *_opt;
    }
    bool supports_option( rs2_option id ) const override { return id == RS2_OPTION_HKR_TEMPORAL_FILTER_DPP; }
    std::vector< rs2_option > get_supported_options() const override { return { RS2_OPTION_HKR_TEMPORAL_FILTER_DPP }; }
    std::string const & get_option_name( rs2_option ) const override { return _name; }
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
// same one rs2::sensor/rs2::embedded_filter use internally - so option_ref() can be exercised
// exactly as a real sensor/embedded_filter would use it.
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
    rs2_set_composite_option( &wrapper, RS2_OPTION_HKR_TEMPORAL_FILTER_DPP, &sent, sizeof( sent ), &e );
    rs2::error::handle( e );

    auto buffer = rs2_get_composite_option( &wrapper, RS2_OPTION_HKR_TEMPORAL_FILTER_DPP, &e );
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

    // --- 2) Exercise the C++ option_ref()/is<T>()/as<T>() path (mirrors rs2::frame's is<T>()/
    //        as<T>() dispatch) on top of the SAME underlying options object ---
    fake_options_handle handle( &wrapper );
    auto option_handle = handle.option_ref( RS2_OPTION_HKR_TEMPORAL_FILTER_DPP );
    if( ! option_handle.is< rs2::composite_option >() )
        throw std::runtime_error( "option_ref(...).is<rs2::composite_option>() unexpectedly false" );

    auto composite_opt = option_handle.as< rs2::composite_option >();
    rs2_temporal_filter_dpp_config sent2 = sent;
    sent2.persistency_index = 7;  // change one field to prove this second round trip is independent
    composite_opt.set( &sent2, sizeof( sent2 ) );

    auto bytes2 = composite_opt.get();
    if( bytes2.size() != sizeof( rs2_temporal_filter_dpp_config ) )
        throw std::runtime_error( "composite_option::get() returned an unexpected payload size" );

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
    std::cout << "[C++ option_ref] sent:     enabled=" << sent2.enabled << " smooth_alpha=" << sent2.smooth_alpha
              << " smooth_delta=" << sent2.smooth_delta << " persistency_index=" << sent2.persistency_index
              << std::endl;
    std::cout << "[C++ option_ref] received: enabled=" << received2.enabled
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
                 "and option_ref()/is<rs2::composite_option>()/as<rs2::composite_option>())."
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
