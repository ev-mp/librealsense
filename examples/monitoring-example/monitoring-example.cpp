#include <signal.h>
#include <iostream>
#include <../include/librealsense2/rs.hpp>
#include <../include/librealsense2/rs_advanced_mode.hpp>
#include <windows.h>


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

int main(int argc, char* argv[]) try
{
    signal(SIGINT, sigintHook);
    //Evgeniauto rs_log_path = ExePath() += "\\realsense_log.txt";
    //evgenirs2::log_to_file(RS2_LOG_SEVERITY_DEBUG, rs_log_path.c_str());
    auto rs_log_path = ExePath() += "\\realsense_log.txt";
    rs2::log_to_file(RS2_LOG_SEVERITY_DEBUG, rs_log_path.c_str());
    //rs2::log_to_console(RS2_LOG_SEVERITY_DEBUG);

    const auto RS_UPDATE_INTERVAL = std::chrono::milliseconds(100);
    auto last_check = std::chrono::steady_clock::now() - RS_UPDATE_INTERVAL - std::chrono::seconds(1); // ensure at least 1 run
    auto loop_count = 0;

    while(!stop)
    {
        if (std::chrono::steady_clock::now() - last_check >= RS_UPDATE_INTERVAL)
        {
            std::cout << "---GO---#" << ++loop_count << std::endl;
            last_check = std::chrono::steady_clock::now();
            rs2::pipeline pipe;
            rs2::config cfg;

            cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);
            cfg.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 30);
            cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
            std::cout << "-> Try pipe start" << std::endl;
            pipe.start(cfg);

            std::cout << "Capture 15 frames" << std::endl;
            for (auto i = 0; i < 15; ++i)
                pipe.wait_for_frames();

            std::cout << "-> Collect info" << std::endl;

            auto adev = pipe.get_active_profile().get_device().as<rs400::advanced_mode>();
            auto depth_table = adev.get_depth_table();
            depth_table.disparityShift = 30;
            adev.set_depth_table(depth_table);
            std::cout << "-> Set disparityShift = 30" << std::endl;

            auto depthSensor = pipe.get_active_profile().get_device().first<rs2::depth_sensor>();
            auto depth_scale = depthSensor.get_depth_scale();
            std::cout << " Depth scale: " << depth_scale << std::endl;

            if (depthSensor.supports(RS2_OPTION_EMITTER_ENABLED) &&
                depthSensor.get_option(RS2_OPTION_EMITTER_ENABLED) != 1.f)
            {
                std::cout << "-> Set RS2_OPTION_EMITTER_ENABLED = 1.f" << std::endl;
                depthSensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f);
            }

            if (depthSensor.supports(RS2_OPTION_LASER_POWER) &&
                depthSensor.get_option(RS2_OPTION_LASER_POWER) != 150.f)
            {
                std::cout << "-> Set RS2_OPTION_LASER_POWER = 150.f" << std::endl;
                depthSensor.set_option(RS2_OPTION_LASER_POWER, 150.f);
            }

            auto dev = pipe.get_active_profile().get_device();
            std::cout << " RS2_CAMERA_INFO_NAME: " << dev.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
            std::cout << " RS2_CAMERA_INFO_SERIAL_NUMBER: " << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
            std::cout << " RS2_CAMERA_INFO_FIRMWARE_VERSION: " << dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION) << std::endl;

            auto sensors = dev.query_sensors();
            for (rs2::sensor sensor : sensors)
            {
                if (sensor.supports(RS2_OPTION_ASIC_TEMPERATURE))
                    std::cout << " RS2_OPTION_ASIC_TEMPERATURE: " << sensor.get_option(RS2_OPTION_ASIC_TEMPERATURE) << std::endl;
                if (sensor.supports(RS2_OPTION_PROJECTOR_TEMPERATURE))
                    std::cout << " RS2_OPTION_PROJECTOR_TEMPERATURE: " << sensor.get_option(RS2_OPTION_PROJECTOR_TEMPERATURE) << std::endl;
                if (sensor.supports(RS2_OPTION_MOTION_MODULE_TEMPERATURE))
                    std::cout << " RS2_OPTION_MOTION_MODULE_TEMPERATURE: " << sensor.get_option(RS2_OPTION_MOTION_MODULE_TEMPERATURE) << std::endl;

                if (sensor.supports(RS2_OPTION_EMITTER_ENABLED))
                    std::cout << " RS2_OPTION_EMITTER_ENABLED: " << sensor.get_option(RS2_OPTION_EMITTER_ENABLED) << std::endl;
                else if (sensor.supports(RS2_OPTION_LASER_POWER))
                    std::cout << " RS2_OPTION_LASER_POWER: " << sensor.get_option(RS2_OPTION_LASER_POWER) << std::endl;

                if (sensor.supports(RS2_CAMERA_INFO_NAME))
                    std::cout << " RS2_CAMERA_INFO_NAME: " << sensor.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
            }

            std::cout << "-> Try pipe stop" << std::endl;
            pipe.stop();
            std::cout << "---ALL GOOD---" << std::endl;
            std::cout << "Wait 10 seconds..." << std::endl;
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