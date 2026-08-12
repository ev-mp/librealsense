// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

// PROTOTYPE / DEMO - HKR MinZ Control, one call at a time, against a real connected device.
// Requires a D555/D5xx with FW >= 7.58.39807.10573 (see minz_filter_feature's registration gate
// in d500-factory.cpp). No fake transport here - this is the real depth sensor's real XU control.

#include <librealsense2/rs.hpp>

#include <algorithm>
#include <cstring>
#include <iostream>
#include <stdexcept>

int main()
try
{
    rs2::context ctx;
    auto devices = ctx.query_devices();
    if( devices.size() == 0 )
        throw std::runtime_error( "no RealSense device connected" );

    rs2::depth_sensor sensor = devices[0].first< rs2::depth_sensor >();
    const auto id = RS2_COMPOSITE_OPTION_HKR_MINZ_CONTROL;

    // 1) Enumeration
    auto supported = sensor.get_supported_composite_options();
    bool has_minz = std::find( supported.begin(), supported.end(), id ) != supported.end();
    std::cout << "[1] MinZ control supported: " << ( has_minz ? "yes" : "no" ) << '\n';
    if( ! has_minz )
        return 0;

    // 2) Get range (min/max/step/def), typed
    auto range = sensor.get_composite_option_range_as< rs2_minz_control_range >( id );
    std::cout << "[2] Range (version=" << range.version << "):\n"
              << "    enable:          min=" << range.min.enable          << " max=" << range.max.enable
              << " step=" << range.step.enable          << " def=" << range.def.enable          << '\n'
              << "    downscale_ratio: min=" << range.min.downscale_ratio << " max=" << range.max.downscale_ratio
              << " step=" << range.step.downscale_ratio << " def=" << range.def.downscale_ratio << '\n'
              << "    disparity_shift: min=" << range.min.disparity_shift << " max=" << range.max.disparity_shift
              << " step=" << range.step.disparity_shift << " def=" << range.def.disparity_shift << '\n'
              << "    threshold:       min=" << range.min.threshold      << " max=" << range.max.threshold
              << " step=" << range.step.threshold      << " def=" << range.def.threshold      << '\n'
              << "    threshold_mode:  min=" << range.min.threshold_mode << " max=" << range.max.threshold_mode
              << " step=" << range.step.threshold_mode << " def=" << range.def.threshold_mode << '\n';

    // 3) Get current value, typed
    auto current = sensor.get_composite_option_as< rs2_minz_control >( id );
    std::cout << "[3] Current: enable=" << current.enable
              << " downscale_ratio=" << current.downscale_ratio
              << " disparity_shift=" << current.disparity_shift
              << " threshold=" << current.threshold
              << " threshold_mode=" << current.threshold_mode << '\n';

    // 4) Set - read-modify-write so the wire header (version/flags/ctl_id/param_count/
    //    param_type) is preserved exactly as the device reported it, not zero-initialized.
    rs2_minz_control cfg = current;
    cfg.enable          = 1;
    cfg.downscale_ratio = 2;
    cfg.threshold        = 500;
    cfg.threshold_mode   = 1;   // Manual
    sensor.set_composite_option_from( id, cfg );
    std::cout << "[4] Set: enable=1 downscale_ratio=2 threshold=500 threshold_mode=1 (Manual)\n";

    // 5) Get again to confirm
    auto after = sensor.get_composite_option_as< rs2_minz_control >( id );
    std::cout << "[5] Readback: enable=" << after.enable
              << " downscale_ratio=" << after.downscale_ratio
              << " threshold=" << after.threshold
              << " threshold_mode=" << after.threshold_mode << '\n';

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
