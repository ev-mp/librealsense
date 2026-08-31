// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

// Reference sample - "how do I use the composite-option API" walkthrough, against REAL
// connected devices - no fake/mock transport. Full sequence:
//
//   1) rs2::context::query_devices()          - enumerate connected devices
//   2) dev.query_sensors() + is<depth_sensor>() - find each device's depth sensor(s)
//   3) sensor.query_embedded_filters()         - composite options live on a sensor's EMBEDDED
//      FILTERs, each its OWN independent options registry - NOT on the depth sensor's own
//      registry directly (see src/ds/d500/composite-embedded-filter.cpp's register_composite_option
//      call, used by both hdrd-embedded-filter.h and temporal-filter-feature.cpp). Calling
//      get_supported_composite_options() straight on the sensor always comes back empty; querying
//      only ONE embedded filter only ever shows that filter's own id(s), never another filter's -
//      every filter must be walked to see every composite option a sensor exposes.
//   4) ef.get_supported_composite_options()    - per filter, whichever composite option(s) it
//      registers (zero is a valid, expected outcome for filters with scalar options only)
//   5) for each id found, the FULL per-id application walkthrough below - dispatched to a typed
//      handler because there is no generic "any composite option" cast (the SDK ships no per-id
//      dispatch; the caller must know each id's documented wire struct, see
//      include/librealsense2/h/rs_composite_option.h):
//        - Get (before)   - get_composite_option(id), cast to the documented struct
//        - Set            - read-modify-write: start from what was just read, change only the
//                           fields this walkthrough means to change, send the WHOLE struct back
//                           as one atomic transaction
//        - Get (after)    - confirm against what was sent
//        - Get range      - get_composite_option_range(id), cast to the documented range struct
//        - Query info     - is_composite_option_read_only() / get_composite_option_description()
//
// Each id's walkthrough is wrapped in its own try/catch: a registered-but-non-functional control
// (a real possibility - supports_composite_option()/get_supported_composite_options() only
// reflect that the SDK's device-class code chose to register this id, never a live "does the
// real firmware actually respond to it" check, see composite_xu_option::is_enabled() always
// returning true) is reported as a per-id FAILED line rather than aborting the whole walkthrough,
// so one broken control doesn't prevent seeing results for every other one.

#include <librealsense2/rs.hpp>
#include <librealsense2/h/rs_hkr_temporal_filter_dpp.h>
#include <librealsense2/h/rs_hkr_improved_close_range_control.h>

#include <algorithm>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
    const char * composite_option_name( rs2_composite_option_id id )
    {
        switch( id )
        {
        case RS2_COMPOSITE_OPTION_HKR_TEMPORAL_FILTER_DPP: return "HKR_TEMPORAL_FILTER_DPP";
        case RS2_COMPOSITE_OPTION_HKR_IMPROVED_CLOSE_RANGE_CONTROL:        return "HKR_IMPROVED_CLOSE_RANGE_CONTROL";
        default:                                           return "UNKNOWN";
        }
    }

    // Prints a composite option's raw payload as a byte array - the untyped form
    // get_composite_option() returns, before any application-side cast. Short buffers (e.g. a
    // single ~38-byte value) print on one line; long ones (e.g. a ~156-byte range payload - four
    // bounds packed together) wrap into a rectangular grid instead of one unwieldy line. The
    // per-line width is chosen per call, not fixed at 32, so the LAST row isn't a ragged
    // leftover: pick the fewest rows that keep every row's width <= 64, then divide the buffer
    // evenly across that many rows - every row (including the last) lands somewhere in [32,64]
    // and, whenever the size divides evenly, all rows come out the same width. E.g. 156 bytes ->
    // 3 rows of 52, not 4 rows of 32 + one ragged row of 28.
    void print_bytes( const char * label, const std::vector< uint8_t > & raw )
    {
        constexpr size_t wrap_threshold = 60;
        constexpr size_t max_bytes_per_line = 64;

        std::cout << label << " bytes (" << raw.size() << "):" << std::hex << std::setfill( '0' );

        if( raw.size() <= wrap_threshold )
        {
            for( size_t i = 0; i < raw.size(); ++i )
                std::cout << ' ' << std::setw( 2 ) << (int)raw[i];
        }
        else
        {
            size_t num_lines = ( raw.size() + max_bytes_per_line - 1 ) / max_bytes_per_line;
            size_t bytes_per_line = ( raw.size() + num_lines - 1 ) / num_lines;
            for( size_t i = 0; i < raw.size(); ++i )
            {
                std::cout << ( i % bytes_per_line == 0 ? "\n        " : " " ) << std::setw( 2 ) << (int)raw[i];
            }
        }

        // std::setfill (unlike std::setw) is sticky - it stays in effect on the stream until
        // explicitly changed again, so it must be restored here or every later std::setw(...)
        // call on std::cout (e.g. field_printer's name-column padding) silently inherits '0'
        // instead of the default space.
        std::cout << std::dec << std::setfill( ' ' ) << '\n';
    }

    // ---- Generic "print any struct's fields" machinery -------------------------------------
    //
    // Same shape as librealsense::md_attribute_parser_base / md_uvc_header_parser<St, Attribute>
    // in src/metadata-parser.h: a pointer-to-member (Attribute S::*) is captured once, at
    // construction, behind a common NON-TEMPLATED base interface - that's what lets a list of
    // these be iterated generically without the iterating code ever knowing S or Attribute.
    // print_struct() below has zero field names hardcoded in it and works for ANY struct type;
    // only the small per-type static table (improved_close_range_fields()/temporal_filter_dpp_fields()) needs to
    // name each field, exactly once, ever.

    // uint8_t/int8_t stream as characters via the default operator<< - not useful for a byte-
    // sized numeric field like dpp_header::version - so route those through an int cast.
    // Every other type (int32_t, float, uint16_t, ...) uses the generic overload as-is.
    inline void stream_value( std::ostream & os, uint8_t v ) { os << (unsigned int)v; }
    inline void stream_value( std::ostream & os, int8_t v ) { os << (int)v; }
    template< class T >
    void stream_value( std::ostream & os, const T & v ) { os << v; }

    class field_printer_base
    {
    public:
        virtual const char * name() const = 0;
        // name_width is the widest field name in this print_struct() call's whole table,
        // computed once up front - every row pads its name to that same width (spaces, not a
        // literal '\t') so "=" lands in the same column on every line regardless of how each
        // individual field name's length happens to fall relative to the terminal's tab stops.
        virtual void print( std::ostream & os, const void * struct_ptr, size_t name_width ) const = 0;
        // Just this field's value, no name/no newline - used by print_range() to lay several
        // struct instances (min/max/def/step) out on one row instead of one block per struct.
        virtual void print_value( std::ostream & os, const void * struct_ptr ) const = 0;
        virtual ~field_printer_base() = default;
    };

    // Stores a std::function accessor rather than a raw pointer-to-member so the same class
    // covers both a direct field (S::*) and one nested a level down through a composed header
    // (S::*header, Header::*field) - see the two make_field_printer() overloads below.
    template< class S, class Attribute >
    class field_printer : public field_printer_base
    {
    public:
        field_printer( const char * name, std::function< const Attribute &( const S & ) > accessor )
            : _name( name ), _accessor( std::move( accessor ) ) {}

        const char * name() const override { return _name; }

        void print( std::ostream & os, const void * struct_ptr, size_t name_width ) const override
        {
            auto & s = *reinterpret_cast< const S * >( struct_ptr );
            os << "        " << std::left << std::setw( (int)name_width ) << _name << std::right << " = ";
            stream_value( os, _accessor( s ) );
            os << '\n';
        }

        void print_value( std::ostream & os, const void * struct_ptr ) const override
        {
            auto & s = *reinterpret_cast< const S * >( struct_ptr );
            stream_value( os, _accessor( s ) );
        }

    private:
        const char * _name;
        std::function< const Attribute &( const S & ) > _accessor;
    };

    template< class S, class Attribute >
    std::shared_ptr< field_printer_base > make_field_printer( const char * name, Attribute S::* field )
    {
        return std::make_shared< field_printer< S, Attribute > >( name,
            [field]( const S & s ) -> const Attribute & { return s.*field; } );
    }

    // Overload for a field nested one level down through a composed sub-struct, e.g.
    // rs2_improved_close_range_control::header (a dpp_header) then dpp_header::version.
    template< class S, class H, class Attribute >
    std::shared_ptr< field_printer_base > make_field_printer( const char * name, H S::* header_field, Attribute H::* field )
    {
        return std::make_shared< field_printer< S, Attribute > >( name,
            [header_field, field]( const S & s ) -> const Attribute & { return ( s.*header_field ).*field; } );
    }

    // The one fully generic entry point - takes whichever field table matches S, no per-struct
    // code here at all.
    template< class S >
    void print_struct( std::ostream & os, const std::vector< std::shared_ptr< field_printer_base > > & fields,
                        const S & value )
    {
        size_t name_width = 0;
        for( auto & f : fields )
            name_width = std::max( name_width, std::string( f->name() ).size() );

        for( auto & f : fields )
            f->print( os, &value, name_width );
    }

    // One header line naming the columns, then one row per field: name = [ min, max, default, step ].
    // Same field table as print_struct() - no per-struct code here either.
    template< class S >
    void print_range( std::ostream & os, const std::vector< std::shared_ptr< field_printer_base > > & fields,
                       const S & min_v, const S & max_v, const S & def_v, const S & step_v )
    {
        size_t name_width = 0;
        for( auto & f : fields )
            name_width = std::max( name_width, std::string( f->name() ).size() );

        os << "        " << std::left << std::setw( (int)name_width ) << "" << std::right
           << " [ min, max, default, step ]\n";
        for( auto & f : fields )
        {
            os << "        " << std::left << std::setw( (int)name_width ) << f->name() << std::right << " = [ ";
            f->print_value( os, &min_v );
            os << ", ";
            f->print_value( os, &max_v );
            os << ", ";
            f->print_value( os, &def_v );
            os << ", ";
            f->print_value( os, &step_v );
            os << " ]\n";
        }
    }

    const std::vector< std::shared_ptr< field_printer_base > > & improved_close_range_fields()
    {
        static const std::vector< std::shared_ptr< field_printer_base > > fields = {
            make_field_printer( "version", &rs2_improved_close_range_control::header, &dpp_header::version ),
            make_field_printer( "flags", &rs2_improved_close_range_control::header, &dpp_header::flags ),
            make_field_printer( "ctl_id", &rs2_improved_close_range_control::header, &dpp_header::ctl_id ),
            make_field_printer( "param_count", &rs2_improved_close_range_control::header, &dpp_header::param_count ),
            make_field_printer( "param_type", &rs2_improved_close_range_control::header, &dpp_header::param_type ),
            make_field_printer( "enable", &rs2_improved_close_range_control::enable ),
            make_field_printer( "downscale_ratio", &rs2_improved_close_range_control::downscale_ratio ),
            make_field_printer( "disparity_shift", &rs2_improved_close_range_control::disparity_shift ),
            make_field_printer( "threshold", &rs2_improved_close_range_control::threshold ),
            make_field_printer( "threshold_mode", &rs2_improved_close_range_control::threshold_mode ),
        };
        return fields;
    }

    const std::vector< std::shared_ptr< field_printer_base > > & temporal_filter_dpp_fields()
    {
        static const std::vector< std::shared_ptr< field_printer_base > > fields = {
            make_field_printer( "enabled", &rs2_temporal_filter_dpp_config::enabled ),
            make_field_printer( "smooth_alpha", &rs2_temporal_filter_dpp_config::smooth_alpha ),
            make_field_printer( "smooth_delta", &rs2_temporal_filter_dpp_config::smooth_delta ),
            make_field_printer( "persistency_index", &rs2_temporal_filter_dpp_config::persistency_index ),
        };
        return fields;
    }
    // ---- end generic struct-printing machinery ---------------------------------------------

    // Full read-modify-write + range + metadata walkthrough for RS2_COMPOSITE_OPTION_HKR_TEMPORAL_FILTER_DPP.
    void exercise_temporal_filter_dpp( rs2::options & opts, rs2_composite_option_id id )
    {
        // Both forms of the read, one after another - the raw untyped bytes get_composite_option()
        // returns, and the typed convenience cast get_composite_option_as<T>() returns. Two
        // separate real GETs (device round trips), shown side by side purely to illustrate both
        // APIs - production code would normally only need the typed one.
        print_bytes( "Get (before)", opts.get_composite_option( id ) );
        auto current = opts.get_composite_option_as< rs2_temporal_filter_dpp_config >( id );
        std::cout << "      Get (before):\n";
        print_struct( std::cout, temporal_filter_dpp_fields(), current );

        rs2_temporal_filter_dpp_config cfg_to_send = current;
        cfg_to_send.enabled = 1;
        cfg_to_send.smooth_alpha = 0.55f;
        cfg_to_send.smooth_delta = 35;
        cfg_to_send.persistency_index = 5;
        opts.set_composite_option_from( id, cfg_to_send );
        std::cout << "      Set (read-modify-write): enabled=1 smooth_alpha=0.55 smooth_delta=35 persistency_index=5\n";

        print_bytes( "Get (after)", opts.get_composite_option( id ) );
        auto cfg = opts.get_composite_option_as< rs2_temporal_filter_dpp_config >( id );
        std::cout << "      Get (after):\n";
        print_struct( std::cout, temporal_filter_dpp_fields(), cfg );
        // Real firmware may quantize/clamp on write - unlike an in-memory mock, an exact
        // mismatch here isn't necessarily a bug, so this is reported, not asserted/thrown on.
        bool matches = cfg.enabled == cfg_to_send.enabled && cfg.smooth_alpha == cfg_to_send.smooth_alpha
            && cfg.smooth_delta == cfg_to_send.smooth_delta && cfg.persistency_index == cfg_to_send.persistency_index;
        std::cout << "      All fields (" << ( matches ? "match what was sent" : "differ - FW may quantize/clamp on write" )
                  << ")\n";

        print_bytes( "Get Range", opts.get_composite_option_range( id ) );
        auto range = opts.get_composite_option_range_as< rs2_temporal_filter_dpp_range >( id );
        std::cout << "      Range: enabled[" << range.min.enabled << ".."
                  << range.max.enabled << "] smooth_alpha[" << range.min.smooth_alpha << ".." << range.max.smooth_alpha
                  << "] smooth_delta[" << range.min.smooth_delta << ".." << range.max.smooth_delta
                  << "] persistency_index[" << range.min.persistency_index << ".." << range.max.persistency_index
                  << "]\n";

        std::cout << "      Read-only: " << ( opts.is_composite_option_read_only( id ) ? "true" : "false" ) << '\n';
        std::cout << "      Description: \"" << opts.get_composite_option_description( id ) << "\"\n";
    }

    // Full read-modify-write + range + metadata walkthrough for RS2_COMPOSITE_OPTION_HKR_IMPROVED_CLOSE_RANGE_CONTROL.
    void exercise_improved_close_range_control( rs2::options & opts, rs2_composite_option_id id )
    {
        // Both forms of the read, one after another - the raw untyped bytes get_composite_option()
        // returns, and the typed convenience cast get_composite_option_as<T>() returns. Two
        // separate real GETs (device round trips), shown side by side purely to illustrate both
        // APIs - production code would normally only need the typed one.
        
        print_bytes( "Get Raw Data:", opts.get_composite_option( id ) );
        auto current = opts.get_composite_option_as< rs2_improved_close_range_control >( id );
        std::cout << "Get Structured Data:\n";
        print_struct( std::cout, improved_close_range_fields(), current );

        // Read-modify-write: wire header (version/flags/ctl_id/param_count/param_type) carried
        // over untouched from what the device just reported, not zero-initialized - only the
        // logical fields below are the ones this walkthrough means to change.
        rs2_improved_close_range_control cfg_to_send = current;
        cfg_to_send.enable = int(!cfg_to_send.enable);        
        opts.set_composite_option_from( id, cfg_to_send );
        std::cout << "Set Structured Data: enable => !enable - writing to device\n";

        print_bytes( "Get Raw Data", opts.get_composite_option( id ) );
        auto cfg = opts.get_composite_option_as< rs2_improved_close_range_control >( id );
        std::cout << "Get modified Structure Data:\n";
        print_struct( std::cout, improved_close_range_fields(), cfg );
        // Only `enable` was deliberately changed above (toggled) - check IT against what was
        // sent, and separately check every OTHER field against `current` (what was actually on
        // the device before this Set) to confirm the read-modify-write really did carry the rest
        // through untouched rather than accidentally stomping them.
        bool modified_field_matches = cfg.enable == cfg_to_send.enable;
        bool rest_intact = cfg.header.version == current.header.version && cfg.header.flags == current.header.flags
            && cfg.header.ctl_id == current.header.ctl_id && cfg.header.param_count == current.header.param_count
            && cfg.header.param_type == current.header.param_type && cfg.downscale_ratio == current.downscale_ratio
            && cfg.disparity_shift == current.disparity_shift && cfg.threshold == current.threshold
            && cfg.threshold_mode == current.threshold_mode;
        std::cout << " Struct.enable field" << ( modified_field_matches ? "matches what was sent" : "differs - FW may quantize/clamp on write" )
                  << "; all other fields " << ( rest_intact ? "intact" : "UNEXPECTEDLY CHANGED vs. originally read data" )
                  << ")\n";

        // range.min/max/step/def are each a FULL rs2_improved_close_range_control - the same struct as
        // `current`/`cfg` above, header fields included - and those bytes ARE exactly what the
        // device's real get_xu_range() call returned. Reusing print_struct()/improved_close_range_fields() here
        // (rather than the old hand-written line, which only showed 4 of the 10 fields) shows
        // every field of every bound, including whether the header sub-fields came back
        // meaningfully populated or all-zero too.
        print_bytes( "Get Range", opts.get_composite_option_range( id ) );
        auto range = opts.get_composite_option_range_as< rs2_improved_close_range_control_range >( id );
        std::cout << "      Range:\n";
        print_range( std::cout, improved_close_range_fields(), range.min, range.max, range.def, range.step );

        std::cout << "      Read-only: " << ( opts.is_composite_option_read_only( id ) ? "true" : "false" ) << '\n';
        std::cout << "      Description: \"" << opts.get_composite_option_description( id ) << "\"\n";
    }

    // Dispatches to the right typed walkthrough - there is no generic "any composite option"
    // mechanism by design (see file header); a new composite option id needs a case added here.
    // Returns true on success, false if this id has no typed walkthrough registered (not a
    // failure, just unhandled) - actual device-transaction failures propagate as exceptions for
    // the caller to catch per-id.
    bool exercise_composite_option( rs2::options & opts, rs2_composite_option_id id )
    {
        switch( id )
        {
        // TODO: re-enable once exercise_temporal_filter_dpp() is verified against real HKR
        // Temporal Filter DPP hardware (not available at the time this walkthrough was written).
        //case RS2_COMPOSITE_OPTION_HKR_TEMPORAL_FILTER_DPP: exercise_temporal_filter_dpp( opts, id ); return true;
        case RS2_COMPOSITE_OPTION_HKR_IMPROVED_CLOSE_RANGE_CONTROL:        exercise_improved_close_range_control( opts, id ); return true;
        default:
            std::cout << "      (no typed walkthrough registered for this composite option id)\n";
            return false;
        }
    }
}

int main()
try
{
    rs2::context ctx;
    auto devices = ctx.query_devices();
    std::cout << "=== Composite-option API walkthrough ===\n";
    std::cout << "Found " << devices.size() << " device(s)\n";

    int attempted = 0, succeeded = 0, skipped = 0;

    for( auto && dev : devices )
    {
        std::string dev_name = dev.supports( RS2_CAMERA_INFO_NAME ) ? dev.get_info( RS2_CAMERA_INFO_NAME ) : "Unknown device";
        std::string dev_sn = dev.supports( RS2_CAMERA_INFO_SERIAL_NUMBER ) ? dev.get_info( RS2_CAMERA_INFO_SERIAL_NUMBER ) : "N/A";
        std::cout << "\nDevice: " << dev_name << " (S/N " << dev_sn << ")\n";

        for( auto && sensor : dev.query_sensors() )
        {
            if( ! sensor.is< rs2::depth_sensor >() )
                continue;

            std::string sensor_name = sensor.supports( RS2_CAMERA_INFO_NAME ) ? sensor.get_info( RS2_CAMERA_INFO_NAME ) : "Depth Sensor";
            std::cout << "  Depth sensor: " << sensor_name << '\n';

            // Composite options live on the sensor's EMBEDDED FILTERs, not the sensor's own
            // registry - every filter must be explored, not just the first one.
            for( auto && ef : sensor.query_embedded_filters() )
            {
                std::cout << "    Embedded filter: " << rs2_embedded_filter_type_to_string(ef.get_type()) << '\n';
                auto composite_ids = ef.get_supported_composite_options();
                if( composite_ids.empty() )
                    continue;

                for( auto id : composite_ids )
                {
                    std::cout << "    - " << composite_option_name( id ) << ":\n";
                    ++attempted;
                    try
                    {
                        if(exercise_composite_option( ef, id ) )
                            ++succeeded;
                    }
                    catch( const rs2::error & e )
                    {
                        // Registered but not actually functional on this device/FW (see file
                        // header - supports_composite_option()/get_supported_composite_options()
                        // only reflect static registration, never a live capability check) - skip
                        // it and keep going rather than treating this as a failure.
                        ++skipped;
                        std::cout << "      SKIPPED (registered but not functional on this device/FW): " << e.what() << '\n';
                    }
                    catch( const std::exception & e )
                    {
                        ++skipped;
                        std::cout << "      SKIPPED: " << e.what() << '\n';
                    }
                }
            }
        }
    }

    std::cout << "\n=== Summary: " << attempted << " composite option(s) found, " << succeeded << " walked "
                 "successfully end-to-end, " << skipped << " skipped (registered but non-functional on this "
                 "device/FW) ===\n";
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
