#include <signal.h>
#include <iostream>
#include <fstream>
#include <iostream>
#include <sstream>
#include <../include/librealsense2/rs.hpp>
#include <../include/librealsense2/rs_advanced_mode.hpp>

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

// 3rd party header for writing png files
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

volatile sig_atomic_t stop;
void sigintHook(int signum)
{
    std::cout << "---GOT SIGINT, STOPPING---" << std::endl;
    stop = 1;
}

std::string ExePath() {
    wchar_t  buffer[MAX_PATH];
    GetModuleFileName(NULL, buffer, MAX_PATH);
    std::wstring wbuf(buffer);
    std::string f(wbuf.begin(), wbuf.end());
    return f.substr(0, f.find_last_of("\\/"));
}

bool checkDepth(rs2::frameset set) {
    if (!set.get_depth_frame())
    {
        return false;
    }

    auto frames_depth = set.get_depth_frame().get_data();
    const auto width = 640;
    const auto height = 480;
    const auto small_size = std::min(height / 3, width);

    uint32_t elems_count = 0;
    auto data = static_cast<uint16_t const*>(frames_depth);

    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            const uint32_t index = j + i * width;

            // reject depth values if they are out of interest range
            if (data[index] > 200)
            {
                elems_count += 1;
            }
        }
    }

    const auto depthNonZeroPercent =
        static_cast<float>(elems_count) / ((small_size + 1) * (small_size + 1));

    return depthNonZeroPercent > 0.3f;
}

void saveFrameOnDisk(rs2::video_frame vf, std::string nameFrameset) {
    std::stringstream png_file;
    png_file << ExePath() << "\\rs_" << nameFrameset <<"_frame_" << vf.get_profile().stream_name() << ".png";
    stbi_write_png(png_file.str().c_str(), vf.get_width(), vf.get_height(),
        vf.get_bytes_per_pixel(), vf.get_data(), vf.get_stride_in_bytes());
    std::cout << "Saved " << png_file.str() << std::endl;
}

void saveFramesetOnDisk(rs2::frameset frameset, std::string nameFrameset) {
    try {
        rs2::colorizer color_map;
        auto depth_frame = frameset.get_depth_frame();
        if (auto vf = depth_frame.as<rs2::video_frame>())
        {
            if (vf.is<rs2::depth_frame>())
                vf = color_map.process(depth_frame);

            saveFrameOnDisk(vf, nameFrameset);
        }

        auto color_frame = frameset.get_color_frame();
        if (auto vf = color_frame.as<rs2::video_frame>())
        {
            saveFrameOnDisk(vf, nameFrameset);
        }

        auto ir_frame = frameset.get_infrared_frame(1);
        if (auto vf = ir_frame.as<rs2::video_frame>())
        {
            saveFrameOnDisk(vf, nameFrameset);
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }
}

void setMeteringRegion(rs2::pipeline* pipe)
{
    try
    {
        auto sensors = pipe->get_active_profile().get_device().query_sensors();
        for (auto& sensor : sensors)
        {
            auto prof = sensor.get_stream_profiles();
            auto stream_color = std::find_if(prof.begin(), prof.end(), [](const auto& x) {
                return x.stream_type() == RS2_STREAM_COLOR;
                }) != prof.end();

            if (stream_color && sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE))
            {
                auto roi_sensor = sensor.as<rs2::roi_sensor>();
                if (roi_sensor)
                {
                    sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1.0f);
                    rs2::region_of_interest roi;
                    roi.min_x = 160;
                    roi.min_y = 120;
                    roi.max_x = 480;
                    roi.max_y = 360;

                    roi_sensor.set_region_of_interest(roi);
                }
            }

            if (!stream_color && sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE))
            {
                sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1.0f);
            }
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "-> Failed to set metering region: " << e.what() << std::endl;
    }
}

int main(int argc, char* argv[]) try
{
    int RSfps = 15;
    int CheckInterval = 3;// 20;
    int DispatiryShift = 0;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-fps") == 0) {
            RSfps = atoi(argv[i + 1]);
        }else if (strcmp(argv[i], "-i") == 0) {
            CheckInterval = atoi(argv[i + 1]);
        }else if (strcmp(argv[i], "-d") == 0) {
            DispatiryShift = atoi(argv[i + 1]);
        }
    }

    std::cout << "Lets begin" << std::endl;
    std::cout << "Camera framerate = " << (RSfps == 0 ? "default" : std::to_string(RSfps)) << std::endl;
    std::cout << "Check interval in sec = " << CheckInterval << std::endl;
    std::cout << "Disparity shift = " << DispatiryShift << std::endl;

    signal(SIGINT, sigintHook);
    auto rs_log_path = ExePath() += "\\realsense_log.txt";
    rs2::log_to_file(RS2_LOG_SEVERITY_DEBUG, rs_log_path.c_str());

    const auto RS_UPDATE_INTERVAL = std::chrono::seconds(CheckInterval);
    auto last_check = std::chrono::steady_clock::now() - RS_UPDATE_INTERVAL - std::chrono::seconds(1); // ensure at least 1 run
    rs2::frameset previous_frameset;
    rs2::frameset current_frameset;
    bool hardwareReset = false;

    while(!stop)
    {
        auto timeToSleep = std::chrono::steady_clock::now() - last_check;
        if (timeToSleep < RS_UPDATE_INTERVAL)
        {
            Sleep((int)(timeToSleep.count() / 1000000));
        }

        std::cout << "---GO---" << std::endl;
        last_check = std::chrono::steady_clock::now();
        rs2::pipeline pipe;
        rs2::config cfg;
        rs2::context ctx;

        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, RSfps);
        cfg.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, RSfps);
        cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, RSfps);
        std::cout << "-> Try pipe start" << std::endl;


		auto pipe_start = std::chrono::high_resolution_clock::now();
        pipe.start(cfg);
		auto pipe_start_duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - pipe_start);
		std::cout << "pipe_start_duration (ms) = " << pipe_start_duration.count() << std::endl;

		auto dev = pipe.get_active_profile().get_device();
		std::cout << " RS2_API_VERSION: " << RS2_API_VERSION_STR << std::endl;
		std::cout << " RS2_CAMERA_INFO_NAME: " << dev.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
		std::cout << " RS2_CAMERA_INFO_SERIAL_NUMBER: " << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
		std::cout << " RS2_CAMERA_INFO_FIRMWARE_VERSION: " << dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION) << std::endl;


        auto adev = pipe.get_active_profile().get_device().as<rs400::advanced_mode>();
        auto depth_table = adev.get_depth_table();
        depth_table.disparityShift = DispatiryShift;
        adev.set_depth_table(depth_table);

        //at first, skip some frames to avoid auto-exposure inappropriate level
        int frame_n = 0;
		auto frame_start = std::chrono::high_resolution_clock::now();
        while (frame_n < 10)
        {
            current_frameset = pipe.wait_for_frames();
            frame_n++;
			if (frame_n == 1)
			{
				auto frame_start_duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - frame_start);
				std::cout << "frame_start_duration (ms) = " << frame_start_duration.count() << std::endl;
				auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - pipe_start);
				std::cout << "total_duration (ms) = " << total_duration.count() << std::endl;
			}

				
        }

        std::cout << "-> Set metering region" << std::endl;
        setMeteringRegion(&pipe);

        std::cout << "-> Checking frames for 10 seconds..." << std::endl;
        while (std::chrono::steady_clock::now() - last_check < std::chrono::seconds(10)) {
            current_frameset = pipe.wait_for_frames();
            if (!hardwareReset) {
                previous_frameset = current_frameset;
                if (!checkDepth(current_frameset)) {
                    std::cout << "-> Catch bad depth. Try hardware reset" << std::endl;
                    hardwareReset = true;
                    break;
                }
            }
            else {
                if (checkDepth(current_frameset)) {
                    std::cout << "-> Hardware reset helped" << std::endl;
                }
                else {
                    std::cout << "-> Hardware reset couldn't help" << std::endl;
                }

                saveFramesetOnDisk(current_frameset, "after_HR");
                if (previous_frameset)
                    saveFramesetOnDisk(previous_frameset, "before_HR");

                hardwareReset = false;
                stop = 1;
                break;
            }
        }

        std::cout << "-> Try pipe stop" << ((hardwareReset) ? " WITH HARDWARE RESET" : "") << std::endl;
        pipe.stop();

        if (hardwareReset) {
            auto list = ctx.query_devices();
            for (int i = 0; i < list.size(); i++)
            {
                rs2::device dev = list[i];
                dev.hardware_reset();
            }
        }

        std::cout << "-> Stopped successfully" << std::endl;

        if (!stop) {
            std::cout << "Wait " << CheckInterval << " seconds..." << std::endl;
        }
        else {
            std::cout << "Finish" << std::endl;
        }
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error& e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}