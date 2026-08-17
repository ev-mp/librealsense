/* License: Apache 2.0. See LICENSE file in root directory. */
/* Copyright(c) 2026 RealSense, Inc. All Rights Reserved. */

/* C99 version of the composite-option walkthrough - same MinZ get/set, through the raw C API:
   no exceptions (check rs2_error* after every call), no templates (manual struct/byte casts),
   no RAII (every list/device/sensor/filter/buffer freed by hand). Main success scenario only. */

#include <librealsense2/rs.h>
#include <librealsense2/h/rs_hkr_minz_control.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void check_error( rs2_error * e )
{
    if( e )
    {
        printf( "error: %s\n", rs2_get_error_message( e ) );
        rs2_free_error( e );
        exit( 1 );
    }
}

int main( void )
{
    rs2_error * e = NULL;

    rs2_context * ctx = rs2_create_context( RS2_API_VERSION, &e );
    check_error( e );

    rs2_device_list * devices = rs2_query_devices( ctx, &e );
    check_error( e );
    int device_count = rs2_get_device_count( devices, &e );
    check_error( e );
    printf( "Found %d device(s)\n", device_count );
    if( device_count == 0 )
        return 0;

    rs2_device * dev = rs2_create_device( devices, 0, &e );
    check_error( e );

    rs2_sensor_list * sensors = rs2_query_sensors( dev, &e );
    check_error( e );
    int sensor_count = rs2_get_sensors_count( sensors, &e );
    check_error( e );

    for( int i = 0; i < sensor_count; ++i )
    {
        rs2_sensor * sensor = rs2_create_sensor( sensors, i, &e );
        check_error( e );

        if( ! rs2_is_sensor_extendable_to( sensor, RS2_EXTENSION_DEPTH_SENSOR, &e ) )
        {
            check_error( e );
            rs2_delete_sensor( sensor );
            continue;
        }
        check_error( e );

        rs2_embedded_filter_list * filters = rs2_query_embedded_filters( sensor, &e );
        check_error( e );
        int filter_count = rs2_get_embedded_filters_count( filters, &e );
        check_error( e );

        for( int f = 0; f < filter_count; ++f )
        {
            rs2_embedded_filter * filter = rs2_create_embedded_filter( filters, f, &e );
            check_error( e );

            /* rs2_embedded_filter is-a rs2_options in the underlying C++ implementation
               (see struct rs2_embedded_filter : public rs2_options in src/rs.cpp) - every
               composite-option C function takes an rs2_options*, so this cast is valid. */
            const rs2_options * opts = ( const rs2_options * )filter;

            if( rs2_supports_composite_option( opts, RS2_COMPOSITE_OPTION_HKR_MINZ_CONTROL, &e ) )
            {
                check_error( e );

                /* Get - raw bytes, application-side cast (no get_composite_option_as<T>() here). */
                const rs2_raw_data_buffer * raw = rs2_get_composite_option(
                    opts, RS2_COMPOSITE_OPTION_HKR_MINZ_CONTROL, &e );
                check_error( e );
                int size = rs2_get_raw_data_size( raw, &e );
                check_error( e );
                if( (size_t)size != sizeof( rs2_minz_control ) )
                {
                    printf( "unexpected payload size: %d\n", size );
                    exit( 1 );
                }
                const unsigned char * bytes = rs2_get_raw_data( raw, &e );
                check_error( e );

                rs2_minz_control current;
                memcpy( &current, bytes, sizeof( current ) );
                rs2_delete_raw_data( raw );

                printf( "Get: enable=%d downscale_ratio=%d disparity_shift=%d threshold=%d threshold_mode=%d\n",
                        current.enable, current.downscale_ratio, current.disparity_shift,
                        current.threshold, current.threshold_mode );

                /* Set - read-modify-write, toggle enable, whole struct sent atomically. */
                rs2_minz_control cfg = current;
                cfg.enable = ! cfg.enable;
                rs2_set_composite_option( opts, RS2_COMPOSITE_OPTION_HKR_MINZ_CONTROL, &cfg, sizeof( cfg ), &e );
                check_error( e );
                printf( "Set: enable=%d\n", cfg.enable );

                /* Get again to confirm. */
                raw = rs2_get_composite_option( opts, RS2_COMPOSITE_OPTION_HKR_MINZ_CONTROL, &e );
                check_error( e );
                bytes = rs2_get_raw_data( raw, &e );
                check_error( e );
                rs2_minz_control after;
                memcpy( &after, bytes, sizeof( after ) );
                rs2_delete_raw_data( raw );

                printf( "Get (after): enable=%d downscale_ratio=%d disparity_shift=%d threshold=%d threshold_mode=%d\n",
                        after.enable, after.downscale_ratio, after.disparity_shift,
                        after.threshold, after.threshold_mode );
                printf( "%s\n", after.enable == cfg.enable ? "matches what was sent" : "differs" );
            }

            rs2_delete_embedded_filter( filter );
        }

        rs2_delete_embedded_filter_list( filters );
        rs2_delete_sensor( sensor );
    }

    rs2_delete_sensor_list( sensors );
    rs2_delete_device( dev );
    rs2_delete_device_list( devices );
    rs2_delete_context( ctx );
    return 0;
}
