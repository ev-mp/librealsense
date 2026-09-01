// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

// Composite-option device sweep, against whatever RealSense devices are
// actually connected (no fake transport). Sequence:
//   1) enumerate connected devices (rs2_query_devices)
//   2) for each device, walk its sensors and find any depth sensor(s)
//   3) for each depth sensor, walk its embedded filters (rs2_query_embedded_filters) - composite
//      options like Improved Close Range/Temporal Filter DPP are registered on an embedded filter's OWN options
//      registry (see e.g. src/ds/d500/hdrd-embedded-filter.h's register_composite_option
//      call), NOT on the depth sensor's own registry directly, matching how the viewer itself
//      only ever queries composite options through rs2::embedded_filter
//      (common/embedded-filter-model.cpp), never through rs2::sensor. Calling
//      get_supported_composite_options() straight on the sensor will always come back empty.
//   4) for each embedded filter, ask whether it supports composite options at all
//      (rs2_get_composite_options_list) - an embedded filter may expose ordinary rs2_option
//      scalar options only, with no composite options, and that's a valid outcome, not an error
//   5) if it does, print every supported composite option id
//   6) for each one, read its current value straight from the device
//      (rs2_get_composite_option) and print every field - e.g. the Temporal Filter DPP prints
//      enabled/smooth_alpha/smooth_delta/persistency_index, Improved Close Range prints its own 7 logical fields
//      plus the shared dppc_header
//
// Drives everything through the real public C++ wrapper (rs2::options, see rs_options.hpp),
// which is a thin pass-through to the C API this is meant to exercise: rs2_get_composite_option,
// rs2_get_composite_options_list, rs2_supports_composite_option, rs2_get_composite_option_range,
// rs2_is_composite_option_read_only, rs2_get_composite_option_description.

#include <librealsense2/rs.hpp>
#include <librealsense2/h/rs_hkr_temporal_filter_dpp.h>
#include <librealsense2/h/rs_hkr_improved_close_range_control.h>

#include <iostream>
#include <string>

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

    void print_temporal_filter_dpp( const rs2_temporal_filter_dpp_config & v )
    {
        std::cout << "        enabled=" << v.enabled
                   << " smooth_alpha=" << v.smooth_alpha
                   << " smooth_delta=" << v.smooth_delta
                   << " persistency_index=" << v.persistency_index << '\n';
    }

    void print_improved_close_range_control( const rs2_improved_close_range_control & v )
    {
        std::cout << "        version=" << (int)v.header.version
                   << " flags=" << (int)v.header.flags
                   << " ctl_id=0x" << std::hex << v.header.ctl_id << std::dec
                   << " param_count=" << (int)v.header.param_count
                   << " param_type=" << (int)v.header.param_type
                   << " enable=" << v.enable
                   << " filter_type=" << v.filter_type
                   << " downscale_ratio=" << v.downscale_ratio
                   << " shift_mode=" << v.shift_mode
                   << " shift_pixels=" << v.shift_pixels
                   << " threshold_mode=" << v.threshold_mode
                   << " threshold_mm=" << v.threshold_mm << '\n';
    }

    // Prints every field of `id`'s current value read straight from the device, dispatching to
    // the right typed cast - there is no generic "print any composite option" mechanism by
    // design (see rs_composite_option.h: the caller is responsible for knowing each option's
    // documented wire struct), so a new composite option needs a case added here too.
    void print_composite_option_value( const rs2::options & opts, rs2_composite_option_id id )
    {
        try
        {
            switch( id )
            {
            case RS2_COMPOSITE_OPTION_HKR_TEMPORAL_FILTER_DPP:
                print_temporal_filter_dpp( opts.get_composite_option_as< rs2_temporal_filter_dpp_config >( id ) );
                break;
            case RS2_COMPOSITE_OPTION_HKR_IMPROVED_CLOSE_RANGE_CONTROL:
                print_improved_close_range_control( opts.get_composite_option_as< rs2_improved_close_range_control >( id ) );
                break;
            default:
                std::cout << "        (no typed printer registered for this composite option id)\n";
                break;
            }
        }
        catch( const std::exception & e )
        {
            std::cout << "        FAILED to read from device: " << e.what() << '\n';
        }
    }
}

int main()
try
{
    rs2::context ctx;
    auto devices = ctx.query_devices();
    std::cout << "Found " << devices.size() << " device(s)\n";

    for( auto && dev : devices )
    {
        std::string dev_name = dev.supports( RS2_CAMERA_INFO_NAME )
            ? dev.get_info( RS2_CAMERA_INFO_NAME ) : "Unknown device";
        std::string dev_sn = dev.supports( RS2_CAMERA_INFO_SERIAL_NUMBER )
            ? dev.get_info( RS2_CAMERA_INFO_SERIAL_NUMBER ) : "N/A";
        std::cout << "\nDevice: " << dev_name << " (S/N " << dev_sn << ")\n";

        bool found_depth_sensor = false;
        for( auto && sensor : dev.query_sensors() )
        {
            if( ! sensor.is< rs2::depth_sensor >() )
                continue;
            found_depth_sensor = true;

            std::string sensor_name = sensor.supports( RS2_CAMERA_INFO_NAME )
                ? sensor.get_info( RS2_CAMERA_INFO_NAME ) : "Depth Sensor";
            std::cout << "  Depth sensor: " << sensor_name << '\n';

            auto embedded_filters = sensor.query_embedded_filters();
            if( embedded_filters.empty() )
            {
                std::cout << "    No embedded filters on this sensor.\n";
                continue;
            }

            for( auto && ef : embedded_filters )
            {
                std::cout << "    Embedded filter: " << rs2_embedded_filter_type_to_string( ef.get_type() ) << '\n';

                auto composite_ids = ef.get_supported_composite_options();
                if( composite_ids.empty() )
                {
                    std::cout << "      Supports composite options: no (scalar rs2_option only)\n";
                    continue;
                }

                std::cout << "      Supports composite options: yes (" << composite_ids.size() << ")\n";
                for( auto id : composite_ids )
                {
                    std::cout << "      - " << composite_option_name( id ) << ":\n";
                    print_composite_option_value( ef, id );
                }
            }
        }

        if( ! found_depth_sensor )
            std::cout << "  No depth sensor on this device.\n";
    }

    return 0;
}
catch( const rs2::error & e )
{
    std::cerr << "FAIL: librealsense error: " << e.what() << std::endl;
    return 1;
}
catch( const std::exception & e )
{
    std::cerr << "FAIL: " << e.what() << std::endl;
    return 1;
}
