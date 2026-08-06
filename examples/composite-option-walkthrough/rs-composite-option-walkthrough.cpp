// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

// PROTOTYPE / DEMO reference sample - "how do I use the composite-option API" walkthrough, NOT
// a hardware test (see rs-hkr-temporal-filter-dpp-mock for the round-trip/atomicity proof this
// sample complements). No physical HKR/D555 device is required, present, or touched - this uses
// the same fake-transport approach as that example (a minimal in-memory
// librealsense::composite_option_interface implementation standing in for
// librealsense::composite_xu_option), wrapped in the same rs2_options C-struct a real
// rs2::sensor/rs2::embedded_filter wraps, so every call below goes through the REAL public API
// exactly as application code would.
//
// Composite options are a completely separate identity/registry space from ordinary rs2_option
// scalar options (see include/librealsense2/h/rs_composite_option.h) - there is no per-id
// dispatch inside the SDK for what a composite option's payload *means*; the SDK only moves
// opaque bytes atomically. It is the APPLICATION's job to know, for whichever
// rs2_composite_option_id it cares about, what struct that id's payload casts to (documented per
// id - e.g. RS2_COMPOSITE_OPTION_HKR_TEMPORAL_FILTER_DPP casts to rs2_temporal_filter_dpp_config /
// rs2_temporal_filter_dpp_range, see rs_hkr_temporal_filter_dpp.h). This sample walks through
// exactly that application-side cast, step by step:
//
//   1) Enumeration    - rs2::options::get_supported_composite_options()
//   2) Set            - rs2::options::set_composite_option(id, &cfg, sizeof(cfg))
//   3) Get            - rs2::options::get_composite_option(id), cast bytes -> rs2_temporal_filter_dpp_config
//   4) Get range      - rs2::options::get_composite_option_range(id), cast bytes -> rs2_temporal_filter_dpp_range
//   5) Query info     - supports_composite_option()/is_composite_option_read_only()/get_composite_option_description()

#include <librealsense2/rs.hpp>

#include <src/composite-option-interface.h>
#include <src/core/options-interface.h>
#include <src/proc/synthetic-stream.h>

#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>

using namespace librealsense;

namespace {

// Minimal fake composite option standing in for librealsense::composite_xu_option (which would
// talk to real HKR/D555 hardware over UVC XU - see src/ds/composite-xu-option.h). Implements
// ONLY composite_option_interface - no relationship to librealsense::option whatsoever, matching
// the real class exactly.
class fake_temporal_filter_dpp_option : public composite_option_interface
{
public:
    bool is_enabled() const override { return true; }
    bool is_read_only() const override { return false; }
    const char * get_description() const override
    {
        return "HKR Temporal Filter DPP (prototype) - a 4-field composite control exchanged "
               "atomically; see rs_hkr_temporal_filter_dpp.h for the wire layout.";
    }

    // The "wire": one call per logical get/set, whole payload atomically - exactly the contract
    // librealsense::composite_xu_option::get_raw()/set_raw() implement via one get_xu()/set_xu()
    // call each against the real device.
    std::vector< uint8_t > get_raw() const override { return _storage; }

    void set_raw( const void * data, size_t size ) override
    {
        auto p = reinterpret_cast< const uint8_t * >( data );
        _storage.assign( p, p + size );
    }

    // Fixed, documented {min,max,step,def} bounds for this prototype control (see
    // rs2_temporal_filter_dpp_range in rs_hkr_temporal_filter_dpp.h) - a real
    // composite_xu_option would instead issue one get_xu_range() call to the device.
    std::vector< uint8_t > get_raw_range() const override
    {
        rs2_temporal_filter_dpp_range range{};
        range.version = 1;
        range.min = { 0, 0.f, 1, 0 };
        range.max = { 1, 1.f, 100, 8 };
        range.step = { 1, 0.01f, 1, 1 };
        range.def = { 0, 0.4f, 20, 3 };

        std::vector< uint8_t > bytes( sizeof( range ) );
        std::memcpy( bytes.data(), &range, sizeof( range ) );
        return bytes;
    }

private:
    std::vector< uint8_t > _storage = []()
    {
        // Seed with the documented defaults so get_composite_option() has something sensible to
        // return even before this walkthrough's own "Set" step runs.
        rs2_temporal_filter_dpp_config def{ 0, 0.4f, 20, 3 };
        std::vector< uint8_t > bytes( sizeof( def ) );
        std::memcpy( bytes.data(), &def, sizeof( def ) );
        return bytes;
    }();
};

// Minimal fake options container - implements librealsense::options_interface directly (no
// scalar rs2_option registered at all), exposing exactly one composite option:
// RS2_COMPOSITE_OPTION_HKR_TEMPORAL_FILTER_DPP. Mirrors rs-hkr-temporal-filter-dpp-mock's
// fake_options_container.
class fake_options_container : public options_interface
{
public:
    explicit fake_options_container( std::shared_ptr< fake_temporal_filter_dpp_option > opt )
        : _opt( std::move( opt ) )
    {
    }

    option & get_option( rs2_option ) override { throw std::runtime_error( "fake_options_container: no scalar options" ); }
    const option & get_option( rs2_option ) const override { throw std::runtime_error( "fake_options_container: no scalar options" ); }
    bool supports_option( rs2_option ) const override { return false; }
    std::vector< rs2_option > get_supported_options() const override { return {}; }
    std::string const & get_option_name( rs2_option ) const override { return _name; }

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

    void create_snapshot( std::shared_ptr< options_interface > & snapshot ) const override { snapshot.reset(); }
    void enable_recording( std::function< void( const options_interface & ) > ) override {}

private:
    std::shared_ptr< fake_temporal_filter_dpp_option > _opt;
    std::string _name = "HKR Temporal Filter DPP";
};

// Lets this standalone sample call the protected rs2::options(rs2_options*) constructor - the
// same one rs2::sensor/rs2::embedded_filter use internally.
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
    auto fake_opt = std::make_shared< fake_temporal_filter_dpp_option >();
    fake_options_container container( fake_opt );
    rs2_options wrapper( &container );
    fake_options_handle sensor( &wrapper );

    std::cout << "=== Composite-option API walkthrough (HKR Temporal Filter DPP prototype) ===\n\n";

    // -----------------------------------------------------------------------------------------
    // 1) Enumeration - composite options are a SEPARATE list from get_supported_options()
    //    (scalar rs2_option ids); this sensor legitimately has zero of those in this sample.
    // -----------------------------------------------------------------------------------------
    std::cout << "[1] Enumeration: get_supported_composite_options()\n";
    auto supported = sensor.get_supported_composite_options();
    for( auto id : supported )
        std::cout << "    - composite option id " << (int)id
                   << ( id == RS2_COMPOSITE_OPTION_HKR_TEMPORAL_FILTER_DPP ? " (RS2_COMPOSITE_OPTION_HKR_TEMPORAL_FILTER_DPP)" : "" )
                   << '\n';
    if( supported.empty() )
        throw std::runtime_error( "expected at least one supported composite option" );
    std::cout << '\n';

    auto const id = RS2_COMPOSITE_OPTION_HKR_TEMPORAL_FILTER_DPP;

    // -----------------------------------------------------------------------------------------
    // 2) Set - build the application's own copy of the documented wire-layout struct and send
    //    it as one atomic transaction. The SDK never inspects these bytes - it is pure payload.
    // -----------------------------------------------------------------------------------------
    std::cout << "[2] Set: set_composite_option(id, &cfg, sizeof(cfg))\n";
    rs2_temporal_filter_dpp_config cfg_to_send{};
    cfg_to_send.enabled = 1;
    cfg_to_send.smooth_alpha = 0.55f;
    cfg_to_send.smooth_delta = 35;
    cfg_to_send.persistency_index = 5;
    sensor.set_composite_option( id, &cfg_to_send, sizeof( cfg_to_send ) );
    std::cout << "    sent: enabled=" << cfg_to_send.enabled << " smooth_alpha=" << cfg_to_send.smooth_alpha
              << " smooth_delta=" << cfg_to_send.smooth_delta
              << " persistency_index=" << cfg_to_send.persistency_index << "\n\n";

    // -----------------------------------------------------------------------------------------
    // 3) Get / query value - get_composite_option() returns opaque bytes; the SDK ships no
    //    per-id dispatch, so the APPLICATION casts them to the struct it knows this id uses.
    // -----------------------------------------------------------------------------------------
    std::cout << "[3] Get: get_composite_option(id), cast to rs2_temporal_filter_dpp_config\n";
    std::vector< uint8_t > raw_value = sensor.get_composite_option( id );
    if( raw_value.size() != sizeof( rs2_temporal_filter_dpp_config ) )
        throw std::runtime_error( "unexpected payload size from get_composite_option" );
    rs2_temporal_filter_dpp_config cfg{};
    std::memcpy( &cfg, raw_value.data(), sizeof( cfg ) );  // <-- the application-side cast
    std::cout << "    received: enabled=" << cfg.enabled << " smooth_alpha=" << cfg.smooth_alpha
              << " smooth_delta=" << cfg.smooth_delta << " persistency_index=" << cfg.persistency_index
              << '\n';
    if( cfg.enabled != cfg_to_send.enabled || cfg.smooth_alpha != cfg_to_send.smooth_alpha
        || cfg.smooth_delta != cfg_to_send.smooth_delta || cfg.persistency_index != cfg_to_send.persistency_index )
        throw std::runtime_error( "round-trip mismatch between set and get" );
    std::cout << "    (matches what was sent in step 2)\n\n";

    // -----------------------------------------------------------------------------------------
    // 4) Get range - same "opaque bytes + application-side cast" story, this time to the
    //    per-id range struct (one instance of the config struct per bound: min/max/step/def).
    // -----------------------------------------------------------------------------------------
    std::cout << "[4] Get range: get_composite_option_range(id), cast to rs2_temporal_filter_dpp_range\n";
    std::vector< uint8_t > raw_range = sensor.get_composite_option_range( id );
    if( raw_range.size() != sizeof( rs2_temporal_filter_dpp_range ) )
        throw std::runtime_error( "unexpected payload size from get_composite_option_range" );
    rs2_temporal_filter_dpp_range range{};
    std::memcpy( &range, raw_range.data(), sizeof( range ) );  // <-- the application-side cast
    std::cout << "    version=" << range.version << '\n';
    std::cout << "    enabled:           min=" << range.min.enabled << " max=" << range.max.enabled
               << " step=" << range.step.enabled << " def=" << range.def.enabled << '\n';
    std::cout << "    smooth_alpha:      min=" << range.min.smooth_alpha << " max=" << range.max.smooth_alpha
               << " step=" << range.step.smooth_alpha << " def=" << range.def.smooth_alpha << '\n';
    std::cout << "    smooth_delta:      min=" << range.min.smooth_delta << " max=" << range.max.smooth_delta
               << " step=" << range.step.smooth_delta << " def=" << range.def.smooth_delta << '\n';
    std::cout << "    persistency_index: min=" << range.min.persistency_index << " max=" << range.max.persistency_index
               << " step=" << range.step.persistency_index << " def=" << range.def.persistency_index << '\n';
    std::cout << '\n';

    // -----------------------------------------------------------------------------------------
    // 5) Query info - metadata about the composite option itself (not its payload value).
    // -----------------------------------------------------------------------------------------
    std::cout << "[5] Query info: supports_composite_option() / is_composite_option_read_only() / get_composite_option_description()\n";
    std::cout << "    supports_composite_option:    " << ( sensor.supports_composite_option( id ) ? "true" : "false" ) << '\n';
    std::cout << "    is_composite_option_read_only: " << ( sensor.is_composite_option_read_only( id ) ? "true" : "false" ) << '\n';
    std::cout << "    description: \"" << sensor.get_composite_option_description( id ) << "\"\n\n";

    std::cout << "PASS: composite-option API walkthrough completed - enumeration, set, get (with "
                 "application-side cast), get_range (with application-side cast), and metadata "
                 "queries all exercised through the real public API."
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
