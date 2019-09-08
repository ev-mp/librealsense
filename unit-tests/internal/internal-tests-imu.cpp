// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

//#include "catch/catch.hpp"
//#include <cmath>
//#include <iostream>
#include <chrono>
#include <iomanip>
//#include <ctime>
//#include <algorithm>
//#include <librealsense2/rs.hpp>
//#include <librealsense2/hpp/rs_sensor.hpp>

//#include "../../common/tiny-profiler.h"
//#include "./../src/environment.h"
#include "./../unit-tests-common.h"

using namespace std;
using namespace librealsense;
using namespace librealsense::platform;

bool is_accel__supported(rs2::device& dev)
{
    bool found_gyro = false;
    bool found_accel = false;
    // The same device should support gyro and accel
    found_gyro = false;
    found_accel = false;
    for (auto sensor : dev.query_sensors())
    {
        for (auto profile : sensor.get_stream_profiles())
        {
            if (profile.stream_type() == RS2_STREAM_GYRO)
                found_gyro = true;

            if (profile.stream_type() == RS2_STREAM_ACCEL)
                found_accel = true;
        }
    }
    return found_gyro && found_accel;
}

TEST_CASE("dso-13262 IMU FPS on start differs from requested", "[live]")
{
    // Require at least one device to be plugged in
    rs2::context ctx;
    std::cout << "IMU consistent data rate test started" << std::endl;
    auto list = ctx.query_devices();
    REQUIRE(list.size());

    for (auto dev : ctx.query_devices())
    {
        if (is_accel__supported(dev))
        {
            // Go over the device's sensors
            for (rs2::sensor& sensor : dev.query_sensors())
            {
                std::vector<rs2::stream_profile> profs;
                std::vector<rs2::stream_profile> accel_profs;
                std::vector<rs2::stream_profile> gyro_profs;
                REQUIRE_NOTHROW(profs = sensor.get_stream_profiles());
                REQUIRE(profs.size() > 0);

                // Get all Accel and Gyro profiles
                std::copy_if(profs.begin(), profs.end(), std::back_inserter(accel_profs),
                    [](const rs2::stream_profile& prof){ return (rs2_stream::RS2_STREAM_ACCEL == prof.stream_type())
                                                             && (rs2_format::RS2_FORMAT_MOTION_XYZ32F == prof.format());});
                std::copy_if(profs.begin(), profs.end(), std::back_inserter(gyro_profs),
                    [](const rs2::stream_profile& prof){ return (rs2_stream::RS2_STREAM_GYRO == prof.stream_type())
                                                             && (rs2_format::RS2_FORMAT_MOTION_XYZ32F == prof.format());});
                // Run the test for IMU sensors that support gyr
                if ( (accel_profs.size() > 0) && (gyro_profs.size() > 0))
                {
                    // Duplicate profile to make the number of internal iterations times 4
                    accel_profs.insert(accel_profs.end(),accel_profs.begin(),accel_profs.end());
                    gyro_profs.insert(gyro_profs.end(),gyro_profs.begin(),gyro_profs.end());

                    for (auto i=0UL; i<1000; i++)
                    {
                        std::cout << "Iteration started : " << i <<std::endl;
                        bool warn = false;
                        std::mutex data_mutex;
                        for (auto accel_p : accel_profs)
                        {
                            for (auto gyro_p : gyro_profs)
                            {
                                // Ev always use the same profile
                                //accel_p = accel_profs[0];
                                //gyro_p = gyro_profs[0];
                                std::cout << "\nStart test for profiles: Accel-" << accel_p.fps() << ", Gyro-" << gyro_p.fps() << " fps" << std::endl;
                                std::map<int,std::pair<double,double> > last_arrived; // Frame index: {hw,sw} TS
                                sensor.open({accel_p, gyro_p});
                                auto accel_interval = 1000.f /accel_p.fps();
                                auto gyro_interval = 1000.f /gyro_p.fps();

                                // Profile HW timestamps
                                if (sensor.supports(RS2_OPTION_GLOBAL_TIME_ENABLED))
                                    sensor.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED,false);

                                auto start_time = chrono::high_resolution_clock::now();
                                sensor.start([accel_p,gyro_p,start_time, accel_interval, gyro_interval,
                                    &last_arrived, &data_mutex, &warn](rs2::frame f)
                                {
                                    std::lock_guard<std::mutex> lock(data_mutex);
                                    auto arrival_time = std::chrono::duration<double, std::milli>(chrono::high_resolution_clock::now() - start_time);
                                    auto st = f.get_profile().stream_type();
                                    auto si = f.get_profile().stream_index();
                                    auto uid = f.get_profile().unique_id();
                                    auto fnum = f.get_frame_number();
                                    auto stream_uid = std::make_pair(st,si);
                                    auto hw_interval = 0.f;
                                    auto sw_interval = 0.f;

                                    float expected_interval = 0.f;

                                    try
                                    {
                                        if (last_arrived.end() != last_arrived.find(uid))
                                        {
                                           auto ts =last_arrived[uid];
                                           hw_interval = f.get_timestamp() - ts.first;
                                           sw_interval = arrival_time.count() - ts.second;
                                           expected_interval = (rs2_stream::RS2_STREAM_ACCEL == st) ? accel_interval : gyro_interval;
                                           CAPTURE(st);
                                           CAPTURE(fnum);
                                           CAPTURE(hw_interval);
                                           CAPTURE(sw_interval);
                                           CAPTURE(expected_interval);
                                           if ((fabs(expected_interval-hw_interval) > expected_interval * 0.3f)
                                                && fnum > 140) // heuristic overcome FW issues
                                                warn = true;
//                                           if ((fabs(expected_interval-sw_interval) > expected_interval * 0.5f)
//                                                && fnum > 10) // heuristic overcome FW issues
//                                                warn = true;
                                        }
                                        last_arrived[uid]=std::make_pair(f.get_timestamp(), arrival_time.count());

                                        std::cout << std::left << std::setw(8) << rs2_stream_to_string(st)
                                        << ":" << std::setw(5) << fnum
                                        << std::setw(8) << std::setprecision(3) << std::fixed
                                        << ",\t HW TS :" << f.get_timestamp() << ", Host TS :" << arrival_time.count()
                                        << std::right
                                        << ",\t HW interval =  " << std::setw(6) << hw_interval
                                        << ", Host interval =  " << std::setw(6) << sw_interval
                                        << ", expected = " << std::setprecision(1) << expected_interval
                                        << (warn? "\t WARNINIG !!!" : "") << std::endl;
                                    }
                                    catch(...) { std::cout << "error in callback\n";}
                                });

                                std::cout << "Collecting data for 1 sec" << std::endl;
                                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                                sensor.stop();
                                sensor.close();
                                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                                if (warn) break;
                            }
                            if (warn) break;
                        }
                        if (warn)
                        {
                            std::cout << "Break occured on iteration: " << i <<std::endl;
                            break;
                        }
                        std::cout << "Iteration " << i << " ended, going to recess"<< std::endl;
                        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
                    }
                }
            }
        }
    }

}
