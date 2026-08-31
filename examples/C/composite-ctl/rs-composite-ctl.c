/* License: Apache 2.0. See LICENSE file in root directory. */
/* Copyright(c) 2026 RealSense, Inc. All Rights Reserved. */

/* C99 composite-option walkthrough - the same device sweep and the same full per-id exercise
   (Get, Set read-modify-write, Get again + verify, Get range, metadata) as the C++
   rs-composite-option-demo.cpp / rs-minz-control-walkthrough.cpp samples, through the raw C
   API instead: no exceptions (every rs2_error* is checked by hand), no templates (no
   get_composite_option_as<T>()/get_composite_option_range_as<TRange>() - every cast is a manual
   memcpy into the documented wire struct), no RAII (every list/device/sensor/filter/buffer freed
   explicitly), no generic field-printer machinery (no pointer-to-member in C - each struct's
   fields are just spelled out once, by hand, in print_minz_struct()/print_minz_range()).
   Everything else - what gets read, written, and printed - matches the C++ walkthrough field for
   field, so a C99-only caller has one place that shows the full surface of this API family. */

#include <librealsense2/rs.h>
#include <librealsense2/h/rs_hkr_minz_control.h>
#include "example.h"   /* shared check_error()/print_device_info() used by every Examples/C sample */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char * composite_option_name( rs2_composite_option_id id )
{
    switch( id )
    {
    case RS2_COMPOSITE_OPTION_HKR_TEMPORAL_FILTER_DPP: return "HKR_TEMPORAL_FILTER_DPP";
    case RS2_COMPOSITE_OPTION_HKR_MINZ_CONTROL:        return "HKR_MINZ_CONTROL";
    default:                                           return "UNKNOWN";
    }
}

/* C99 equivalent of the C++ walkthrough's print_bytes(): the untyped raw payload, before any
   application-side cast. Short buffers (a single ~38-byte value) print on one line; long ones
   (the ~152-byte range payload - four bounds packed together) wrap into a rectangular grid
   instead of one unwieldy line, picking the fewest rows that keep every row's width in [32,64] so
   the last row isn't a ragged leftover. */
static void print_bytes( const char * label, const unsigned char * data, int size )
{
    const int wrap_threshold = 60;
    const int max_bytes_per_line = 64;
    int i;

    printf( "%s bytes (%d):", label, size );

    if( size <= wrap_threshold )
    {
        for( i = 0; i < size; ++i )
            printf( " %02x", data[i] );
    }
    else
    {
        int num_lines = ( size + max_bytes_per_line - 1 ) / max_bytes_per_line;
        int bytes_per_line = ( size + num_lines - 1 ) / num_lines;
        for( i = 0; i < size; ++i )
            printf( "%s%02x", ( i % bytes_per_line == 0 ) ? "\n        " : " ", data[i] );
    }
    printf( "\n" );
}

/* C99 equivalent of the C++ walkthrough's generic print_struct(minz_fields(), ...) - no
   pointer-to-member table to drive it in C, so every field is spelled out here once, in wire
   order, matching rs_hkr_minz_control.h. */
static void print_minz_struct( const rs2_minz_control * v )
{
    printf( "        %-16s = %d\n", "version", (int)v->version );
    printf( "        %-16s = %d\n", "flags", (int)v->flags );
    printf( "        %-16s = 0x%x\n", "ctl_id", (unsigned int)v->ctl_id );
    printf( "        %-16s = %d\n", "param_count", (int)v->param_count );
    printf( "        %-16s = %d\n", "param_type", (int)v->param_type );
    printf( "        %-16s = %d\n", "enable", v->enable );
    printf( "        %-16s = %d\n", "downscale_ratio", v->downscale_ratio );
    printf( "        %-16s = %d\n", "disparity_shift", v->disparity_shift );
    printf( "        %-16s = %d\n", "threshold", v->threshold );
    printf( "        %-16s = %d\n", "threshold_mode", v->threshold_mode );
}

/* C99 equivalent of the C++ walkthrough's generic print_range(minz_fields(), ...): one header
   line naming the columns, then one row per field, name = [ min, max, default, step ]. */
static void print_minz_range( const rs2_minz_control_range * r )
{
    printf( "        %-16s   [ min, max, default, step ]\n", "" );
#define MINZ_RANGE_ROW( label, field ) \
    printf( "        %-16s = [ %d, %d, %d, %d ]\n", label, \
            (int)r->min.field, (int)r->max.field, (int)r->def.field, (int)r->step.field )
    MINZ_RANGE_ROW( "version", version );
    MINZ_RANGE_ROW( "flags", flags );
    MINZ_RANGE_ROW( "ctl_id", ctl_id );
    MINZ_RANGE_ROW( "param_count", param_count );
    MINZ_RANGE_ROW( "param_type", param_type );
    MINZ_RANGE_ROW( "enable", enable );
    MINZ_RANGE_ROW( "downscale_ratio", downscale_ratio );
    MINZ_RANGE_ROW( "disparity_shift", disparity_shift );
    MINZ_RANGE_ROW( "threshold", threshold );
    MINZ_RANGE_ROW( "threshold_mode", threshold_mode );
#undef MINZ_RANGE_ROW
}

/* Full read-modify-write + range + metadata walkthrough for RS2_COMPOSITE_OPTION_HKR_MINZ_CONTROL -
   the C99 equivalent of exercise_minz_control() in rs-composite-option-demo.cpp. Same 5
   operations, in the same order:
     1) Get (before)  - raw bytes, then the manual cast to rs2_minz_control
     2) Set            - read-modify-write: only `enable` is toggled, everything else carried
                          through untouched from what was just read
     3) Get (after)    - confirm `enable` against what was sent, and every OTHER field against
                          what was on the device before the Set
     4) Get range      - raw bytes, then the manual cast to rs2_minz_control_range
     5) Query info      - read-only flag, human-readable description
   A registered-but-non-functional control is a real possibility on real hardware (see the file
   header note in rs-composite-option-demo.cpp) - reported as SKIPPED via `goto skipped`
   rather than aborting the whole program, the closest C99 equivalent of that walkthrough's
   per-id try/catch. Returns 1 if every step succeeded, 0 if skipped partway through. */
static int exercise_minz_control( const rs2_options * opts, rs2_composite_option_id id )
{
    rs2_error * e = NULL;
    const rs2_raw_data_buffer * raw;
    const unsigned char * bytes;
    int size;
    rs2_minz_control current, cfg_to_send, cfg;
    rs2_minz_control_range range;
    int modified_field_matches, rest_intact;

    /* 1) Get (before) - both forms of the read, same as the C++ walkthrough: the raw untyped
       bytes get_composite_option() returns, and (here, by hand) the typed cast. */
    raw = rs2_get_composite_option( opts, id, &e );
    if( e )
        goto skipped;
    size = rs2_get_raw_data_size( raw, &e );
    if( e )
    {
        rs2_delete_raw_data( raw );
        goto skipped;
    }
    if( (size_t)size != sizeof( current ) )
    {
        printf( "      unexpected payload size: %d (expected %zu)\n", size, sizeof( current ) );
        rs2_delete_raw_data( raw );
        return 0;
    }
    bytes = rs2_get_raw_data( raw, &e );
    if( e )
    {
        rs2_delete_raw_data( raw );
        goto skipped;
    }
    print_bytes( "      Get Raw Data:", bytes, size );
    memcpy( &current, bytes, sizeof( current ) );
    rs2_delete_raw_data( raw );
    printf( "      Get Structured Data:\n" );
    print_minz_struct( &current );

    /* 2) Set - read-modify-write: wire header and every other field carried over untouched from
       `current`, not zero-initialized - only `enable` is the field this walkthrough means to
       change. */
    cfg_to_send = current;
    cfg_to_send.enable = ! cfg_to_send.enable;
    rs2_set_composite_option( opts, id, &cfg_to_send, sizeof( cfg_to_send ), &e );
    if( e )
        goto skipped;
    printf( "      Set Structured Data: enable => !enable - writing to device\n" );

    /* 3) Get (after) - confirm. */
    raw = rs2_get_composite_option( opts, id, &e );
    if( e )
        goto skipped;
    size = rs2_get_raw_data_size( raw, &e );
    if( e )
    {
        rs2_delete_raw_data( raw );
        goto skipped;
    }
    bytes = rs2_get_raw_data( raw, &e );
    if( e )
    {
        rs2_delete_raw_data( raw );
        goto skipped;
    }
    print_bytes( "      Get Raw Data", bytes, size );
    memcpy( &cfg, bytes, sizeof( cfg ) );
    rs2_delete_raw_data( raw );
    printf( "      Get modified Structure Data:\n" );
    print_minz_struct( &cfg );

    /* Only `enable` was deliberately changed above (toggled) - check IT against what was sent,
       and separately check every OTHER field against `current` (what was actually on the device
       before this Set) to confirm the read-modify-write really did carry the rest through
       untouched rather than accidentally stomping them. Real firmware may quantize/clamp on
       write - unlike an in-memory mock, a mismatch here isn't necessarily a bug, so it is
       reported, not treated as fatal. */
    modified_field_matches = ( cfg.enable == cfg_to_send.enable );
    rest_intact = cfg.version == current.version && cfg.flags == current.flags
        && cfg.ctl_id == current.ctl_id && cfg.param_count == current.param_count
        && cfg.param_type == current.param_type && cfg.downscale_ratio == current.downscale_ratio
        && cfg.disparity_shift == current.disparity_shift && cfg.threshold == current.threshold
        && cfg.threshold_mode == current.threshold_mode;
    printf( "      enable field %s; all other fields %s\n",
            modified_field_matches ? "matches what was sent" : "differs - FW may quantize/clamp on write",
            rest_intact ? "intact" : "UNEXPECTEDLY CHANGED vs. originally read data" );

    /* 4) Get range - raw bytes, then the manual cast to rs2_minz_control_range (4 full copies of
       the struct - min/max/step/def, header fields included - see rs_hkr_minz_control.h). */
    raw = rs2_get_composite_option_range( opts, id, &e );
    if( e )
        goto skipped;
    size = rs2_get_raw_data_size( raw, &e );
    if( e )
    {
        rs2_delete_raw_data( raw );
        goto skipped;
    }
    if( (size_t)size != sizeof( range ) )
    {
        printf( "      unexpected range payload size: %d (expected %zu)\n", size, sizeof( range ) );
        rs2_delete_raw_data( raw );
        return 0;
    }
    bytes = rs2_get_raw_data( raw, &e );
    if( e )
    {
        rs2_delete_raw_data( raw );
        goto skipped;
    }
    print_bytes( "      Get Range", bytes, size );
    memcpy( &range, bytes, sizeof( range ) );
    rs2_delete_raw_data( raw );
    /* range.version versions the {min,max,step,def} WRAPPER shape itself (hardcoded by the SDK
       when it packs the reply) - a separate concept from rs2_minz_control::version inside each
       of the four bounds, which IS read from the device. */
    printf( "      Range (version=%u):\n", range.version );
    print_minz_range( &range );

    /* 5) Query info. */
    printf( "      Read-only: %s\n", rs2_is_composite_option_read_only( opts, id, &e ) ? "true" : "false" );
    if( e )
        goto skipped;
    printf( "      Description: \"%s\"\n", rs2_get_composite_option_description( opts, id, &e ) );
    if( e )
        goto skipped;

    return 1;

skipped:
    printf( "      SKIPPED (registered but not functional on this device/FW): %s\n", rs2_get_error_message( e ) );
    rs2_free_error( e );
    return 0;
}

/* Dispatches to the right typed walkthrough - there is no generic "any composite option"
   mechanism by design (see rs_composite_option.h): a new composite option id needs a case added
   here too, same as the C++ walkthrough's exercise_composite_option(). Returns 1 (succeeded), 0
   (skipped - device/FW rejected something partway through), or -1 (no typed walkthrough
   registered for this id at all - not counted as either, same as the C++ version leaving it out
   of both `succeeded` and `skipped`). */
static int exercise_composite_option( const rs2_options * opts, rs2_composite_option_id id )
{
    switch( id )
    {
    case RS2_COMPOSITE_OPTION_HKR_MINZ_CONTROL:
        return exercise_minz_control( opts, id );
    default:
        printf( "      (no typed walkthrough registered for this composite option id)\n" );
        return -1;
    }
}

int main( void )
{
    rs2_error * e = NULL;
    int attempted = 0, succeeded = 0, skipped = 0;
    int d, s, f, c;

    rs2_context * ctx = rs2_create_context( RS2_API_VERSION, &e );
    check_error( e );

    rs2_device_list * devices = rs2_query_devices( ctx, &e );
    check_error( e );
    int device_count = rs2_get_device_count( devices, &e );
    check_error( e );
    printf( "Found %d device(s)\n", device_count );

    for( d = 0; d < device_count; ++d )
    {
        rs2_device * dev = rs2_create_device( devices, d, &e );
        check_error( e );

        const char * dev_name = rs2_supports_device_info( dev, RS2_CAMERA_INFO_NAME, &e )
            ? rs2_get_device_info( dev, RS2_CAMERA_INFO_NAME, &e ) : "Unknown device";
        check_error( e );
        const char * dev_sn = rs2_supports_device_info( dev, RS2_CAMERA_INFO_SERIAL_NUMBER, &e )
            ? rs2_get_device_info( dev, RS2_CAMERA_INFO_SERIAL_NUMBER, &e ) : "N/A";
        check_error( e );
        printf( "\nDevice: %s (S/N %s)\n", dev_name, dev_sn );

        rs2_sensor_list * sensors = rs2_query_sensors( dev, &e );
        check_error( e );
        int sensor_count = rs2_get_sensors_count( sensors, &e );
        check_error( e );

        for( s = 0; s < sensor_count; ++s )
        {
            rs2_sensor * sensor = rs2_create_sensor( sensors, s, &e );
            check_error( e );

            int is_depth = rs2_is_sensor_extendable_to( sensor, RS2_EXTENSION_DEPTH_SENSOR, &e );
            check_error( e );
            if( ! is_depth )
            {
                rs2_delete_sensor( sensor );
                continue;
            }

            const char * sensor_name = rs2_supports_sensor_info( sensor, RS2_CAMERA_INFO_NAME, &e )
                ? rs2_get_sensor_info( sensor, RS2_CAMERA_INFO_NAME, &e ) : "Depth Sensor";
            check_error( e );
            printf( "  Depth sensor: %s\n", sensor_name );

            /* Composite options live on a sensor's EMBEDDED FILTERs, each its own independent
               options registry - not on the depth sensor's own registry directly - so every
               filter must be walked, not just the first one. */
            rs2_embedded_filter_list * filters = rs2_query_embedded_filters( sensor, &e );
            check_error( e );
            int filter_count = rs2_get_embedded_filters_count( filters, &e );
            check_error( e );

            for( f = 0; f < filter_count; ++f )
            {
                rs2_embedded_filter * filter = rs2_create_embedded_filter( filters, f, &e );
                check_error( e );

                rs2_embedded_filter_type filter_type = rs2_get_embedded_filter_type( filter, &e );
                check_error( e );
                printf( "    Embedded filter: %s\n", rs2_embedded_filter_type_to_string( filter_type ) );

                /* rs2_embedded_filter is-a rs2_options in the underlying C++ implementation (see
                   struct rs2_embedded_filter : public rs2_options in src/rs.cpp) - every
                   composite-option C function takes an rs2_options*, so this cast is valid. */
                const rs2_options * opts = (const rs2_options *)filter;

                rs2_composite_options_list * composite_ids = rs2_get_composite_options_list( opts, &e );
                check_error( e );
                int composite_count = rs2_get_composite_options_list_size( composite_ids, &e );
                check_error( e );

                if( composite_count == 0 )
                    printf( "      Supports composite options: no (scalar rs2_option only)\n" );
                else
                    printf( "      Supports composite options: yes (%d)\n", composite_count );

                for( c = 0; c < composite_count; ++c )
                {
                    rs2_composite_option_id id = rs2_get_composite_option_from_list( composite_ids, c, &e );
                    check_error( e );

                    printf( "      - %s:\n", composite_option_name( id ) );
                    ++attempted;
                    int result = exercise_composite_option( opts, id );
                    if( result == 1 )
                        ++succeeded;
                    else if( result == 0 )
                        ++skipped;
                }

                rs2_delete_composite_options_list( composite_ids );
                rs2_delete_embedded_filter( filter );
            }

            rs2_delete_embedded_filter_list( filters );
            rs2_delete_sensor( sensor );
        }

        rs2_delete_sensor_list( sensors );
        rs2_delete_device( dev );
    }

    printf( "\n=== Summary: %d composite option(s) found, %d walked successfully end-to-end, "
            "%d skipped (registered but non-functional on this device/FW) ===\n",
            attempted, succeeded, skipped );

    rs2_delete_device_list( devices );
    rs2_delete_context( ctx );
    return 0;
}
