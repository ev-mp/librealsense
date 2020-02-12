// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Intel Corporation. All Rights Reserved.

////////////////////////////////////////////////
// Regressions tests against the known issues //
////////////////////////////////////////////////

#include <cmath>
#include "unit-tests-common.h"
#include "../include/librealsense2/rs_advanced_mode.hpp"
#include <librealsense2/hpp/rs_types.hpp>
#include <librealsense2/hpp/rs_frame.hpp>
#include <iostream>
#include <chrono>
#include <ctime>
#include <algorithm>
#include <librealsense2/rsutil.h>

using namespace rs2;

TEST_CASE("DSO-14512", "[live]"){
    {
        rs2::log_to_file(RS2_LOG_SEVERITY_DEBUG,"lrs_log.txt");
        rs2::context ctx;
        if (make_context(SECTION_FROM_TEST_NAME, &ctx))
        {
            for (size_t iter = 0; iter < 1000; iter++)
            {
                std::vector<rs2::device> list;
                REQUIRE_NOTHROW(list = ctx.query_devices());
                REQUIRE(list.size());

                auto dev = list[0];
                CAPTURE(dev.get_info(RS2_CAMERA_INFO_NAME));
                disable_sensitive_options_for(dev);

                std::mutex m;
                int fps = is_usb3(dev) ? 30 : 15; // In USB2 Mode the devices will switch to lower FPS rates

                for (auto i = 0; i < 10; i++)
                {
                    std::map<std::string, size_t> frames_per_stream{};

                    auto profiles = configure_all_supported_streams(dev, 640, 480, fps);

                    for (auto s : profiles.first)
                    {
                        REQUIRE_NOTHROW(s.start([&m, &frames_per_stream](rs2::frame f)
                            {
                                std::lock_guard<std::mutex> lock(m);
                                ++frames_per_stream[f.get_profile().stream_name()];
                            }));
                    }

                    std::this_thread::sleep_for(std::chrono::seconds(2));
                    // Stop & flush all active sensors. The separation is intended to semi-confirm the FPS
                    for (auto s : profiles.first)
                        REQUIRE_NOTHROW(s.stop());
                    for (auto s : profiles.first)
                        REQUIRE_NOTHROW(s.close());

                    // Verify frames arrived for all the profiles specified
                    std::stringstream active_profiles, streams_arrived;
                    active_profiles << "Profiles requested : " << profiles.second.size() << std::endl;
                    for (auto& pf : profiles.second)
                        active_profiles << pf << std::endl;
                    streams_arrived << "Streams recorded : " << frames_per_stream.size() << std::endl;
                    for (auto& frec : frames_per_stream)
                        streams_arrived << frec.first << ": frames = " << frec.second << std::endl;

                    CAPTURE(active_profiles.str().c_str());
                    CAPTURE(streams_arrived.str().c_str());
                    REQUIRE(profiles.second.size() == frames_per_stream.size());
                    std::stringstream s;
                    s << "Streaming cycle " << i << " iteration " << iter << " completed,\n"
                        << active_profiles.str() << std::endl << streams_arrived.str() << std::endl;
                    WARN(s.str().c_str());
                }

                if (dev.is<rs400::advanced_mode>())
                {
                    auto advanced = dev.as<rs400::advanced_mode>();
                    if (advanced.is_enabled())
                    {
                        std::cout << "Iteration " << iter << " ended, resetting device..." << std::endl;
                        advanced.hardware_reset();
                        std::this_thread::sleep_for(std::chrono::seconds(3));
                    }
                }
                else
                {
                    FAIL("Device doesn't support AdvancedMode API");
                }
            }
        }
    }
}
