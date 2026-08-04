// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

// PROTOTYPE / DEMO API - not a finalized production interface. See rs_composite_option.h for
// the underlying generic C API and the atomicity contract (single UVC transaction per call).

#ifndef LIBREALSENSE_RS2_COMPOSITE_OPTION_HPP
#define LIBREALSENSE_RS2_COMPOSITE_OPTION_HPP

#include "rs_options.hpp"
#include "../h/rs_composite_option.h"

#include <memory>
#include <vector>
#include <cstdint>

namespace rs2
{
    // Lightweight, non-owning reference to a single (options-object, rs2_option id) pair -
    // obtained via options::option_ref(). Mirrors rs2::frame's is<T>()/as<T>() dispatch
    // (rs_frame.hpp / rs2_is_frame_extendable_to) exactly, just for options instead of frames:
    // is<T>() / as<T>() construct a T from *this and rely on T's converting constructor to flag
    // validity (see composite_option below), rather than throwing.
    //
    // Does NOT own the underlying rs2_options* - it shares the lifetime of whichever
    // sensor/embedded_filter produced it via options::option_ref(); the caller must keep that
    // object alive for as long as this handle (or anything cast from it) is used.
    class option_handle
    {
    public:
        option_handle() = default;

        option_handle( std::shared_ptr< rs2_options > options, rs2_option id )
            : _options( std::move( options ) )
            , _id( id )
        {
        }

        template< class T >
        bool is() const
        {
            T extension( *this );
            return extension;
        }

        template< class T >
        T as() const
        {
            T extension( *this );
            return extension;
        }

        rs2_option get_option_id() const { return _id; }

        operator bool() const { return _valid && _options != nullptr; }

    protected:
        std::shared_ptr< rs2_options > _options;
        rs2_option _id = RS2_OPTION_COUNT;
        bool _valid = true;
    };

    // Generic "composite option" extension: any option that is a multi-field, atomically
    // exchanged composite control (see rs_composite_option.h) extends to this - one wrapper
    // class shared by every such option, not one per feature. Which option this refers to is
    // carried by the underlying option_handle's id - obtain one via options::option_ref(id).
    class composite_option : public option_handle
    {
    public:
        // Mirrors rs2::video_frame(const rs2::frame&) exactly (see rs_frame.hpp): checks the
        // extension via the C API and marks itself invalid (never throws) on mismatch. Guarded
        // by rs2_supports_option() first (a plain, never-throwing lookup) because - unlike
        // rs2_is_sensor_extendable_to/rs2_is_frame_extendable_to, which only ever risk a
        // dynamic_cast - rs2_is_option_extendable_to looks up the option by id first, which
        // throws (surfaced as an rs2_error) if this id isn't registered on the container at all;
        // that's a legitimate "not this option" case here, not an error to propagate.
        composite_option( const option_handle & h )
            : option_handle( h )
        {
            rs2_error * e = nullptr;
            bool extendable = _options && rs2_supports_option( _options.get(), _id, &e ) > 0;
            error::handle( e );
            if( extendable )
            {
                extendable = rs2_is_option_extendable_to( _options.get(), _id, RS2_EXTENSION_COMPOSITE_OPTION, &e ) > 0;
                error::handle( e );
            }
            if( ! extendable )
                _valid = false;
        }

        // Single atomic UVC transaction (one set_xu round trip) - all fields of *data sent
        // together. No ownership transfer - data remains caller-owned, like options::set_option.
        void set( const void * data, size_t size ) const
        {
            rs2_error * e = nullptr;
            rs2_set_composite_option( _options.get(), _id, data, static_cast< unsigned int >( size ), &e );
            error::handle( e );
        }

        // Single atomic UVC transaction (one get_xu round trip). The SDK allocates the result
        // (the caller has no generic way to know this option's wire size in advance) and this
        // wrapper hides the raw rs2_raw_data_buffer/manual-free entirely (mirrors
        // rs2::safety_sensor::get_safety_preset's exact unwrap pattern), returning a plain
        // std::vector<uint8_t> of the option's current raw payload bytes.
        std::vector< uint8_t > get() const
        {
            rs2_error * e = nullptr;
            auto buffer = rs2_get_composite_option( _options.get(), _id, &e );
            return unwrap( buffer, e );
        }

        // Same unwrap pattern as get(), for the option's supported {min,max,step,def} - see
        // rs_hkr_temporal_filter_dpp.h for RS2_OPTION_HKR_TEMPORAL_FILTER_DPP's range struct.
        std::vector< uint8_t > get_range() const
        {
            rs2_error * e = nullptr;
            auto buffer = rs2_get_composite_option_range( _options.get(), _id, &e );
            return unwrap( buffer, e );
        }

        // Full functional parity with plain rs2::option-style metadata queries - pure
        // metadata/no value-marshaling, so these just delegate through normally.
        bool is_read_only() const
        {
            rs2_error * e = nullptr;
            auto res = rs2_is_option_read_only( _options.get(), _id, &e );
            error::handle( e );
            return res > 0;
        }

        // Reflects real, current runtime availability (delegates to rs2_supports_option) - this
        // is NOT the mechanism used to block scalar rs2_get_option/rs2_set_option access on a
        // composite option (that's a hard-blocked, always-throws operation on the underlying
        // option implementation - see librealsense::composite_xu_option).
        bool is_enabled() const
        {
            rs2_error * e = nullptr;
            auto res = rs2_supports_option( _options.get(), _id, &e );
            error::handle( e );
            return res > 0;
        }

        const char * get_description() const
        {
            rs2_error * e = nullptr;
            auto res = rs2_get_option_description( _options.get(), _id, &e );
            error::handle( e );
            return res;
        }

    private:
        static std::vector< uint8_t > unwrap( const rs2_raw_data_buffer * buffer, rs2_error * e )
        {
            std::shared_ptr< const rs2_raw_data_buffer > list( buffer, rs2_delete_raw_data );
            error::handle( e );

            rs2_error * e2 = nullptr;
            auto size = rs2_get_raw_data_size( list.get(), &e2 );
            error::handle( e2 );

            auto start = rs2_get_raw_data( list.get(), &e2 );
            error::handle( e2 );

            std::vector< uint8_t > result;
            result.insert( result.begin(), start, start + size );
            return result;
        }
    };

    inline option_handle options::option_ref( rs2_option id ) const
    {
        // Non-owning: no-op deleter. The real object (sensor/embedded_filter) that _options
        // aliases owns its own lifetime elsewhere - this handle just shares it, exactly as
        // rs2::options itself only ever holds a raw, non-owning rs2_options* alias (see
        // options::_options / options(rs2_options*) above).
        return option_handle( std::shared_ptr< rs2_options >( _options, []( rs2_options * ) {} ), id );
    }
}
#endif // LIBREALSENSE_RS2_COMPOSITE_OPTION_HPP
