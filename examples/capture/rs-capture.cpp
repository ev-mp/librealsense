// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "example.hpp"          // Include short list of convenience functions for rendering
#include <librealsense2/hpp/rs_internal.hpp> // Include RealSense Cross Platform API
#include <fstream>
#include "tclap/CmdLine.h"
#include "tclap/ValueArg.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <../third-party/stb_image_write.h>
#include <string>
using namespace TCLAP;
// Capture Example demonstrates how to
// capture depth and color video streams and render them to the screen
int main(int argc, char * argv[]) try
{
    CmdLine cmd("librealsense rs-data-collect example tool", ' ');
    ValueArg<std::string> file_arg("f", "FullFilePath", "the file", false, "", "");
    ValueArg<int>    width_arg("w", "Width", "Width", false, 0, "");
    ValueArg<int>    height_arg("r", "height", "height", false, 0, "");
    MultiArg<int>    intrinsics_arg("i", "intrinsics", "intrinsics", false, "");

    cmd.add(file_arg);
    cmd.add(width_arg);
    cmd.add(height_arg);
    cmd.add(intrinsics_arg);

    cmd.parse(argc, argv);

    auto file = file_arg.getValue();

    const int W = width_arg.getValue();
    const int H = height_arg.getValue();
    auto vec = intrinsics_arg.getValue();
    if (vec.size() < 4)
    {
        throw std::runtime_error("intrinsics argguments are missing");
    }

    auto fx = vec[0];
    auto fy = vec[1];
    auto px = vec[2];
    auto py = vec[3];

    using namespace rs2;
    /*const int W = 1280;
    const int H = 720;*/
    const int BPP = 2;

    bypass_device dev;
    rs2_intrinsics intrinsics{ W, H, px, py, fx, fy, RS2_DISTORTION_NONE , {0,0,0,0,0} };

    dev.add_sensor("DS5u");
    dev.add_video_stream(0, RS2_STREAM_DEPTH, 0, 0, W, H, BPP, RS2_FORMAT_Z16, intrinsics);


    // Create a simple OpenGL window for rendering:
    window app(1280, 720, "RealSense Capture Example");
    // Declare two textures on the GPU, one for color and one for depth
    texture depth_image, color_image;
    auto s = dev.query_sensors().front();
    s.get_stream_profiles();
    auto profiles = s.get_stream_profiles();
    auto depth = profiles[0];

    std::ifstream f(file, std::ios::binary);
    if (!f.good())
    {
        throw std::runtime_error("Couldn't open the file");
    }
    std::vector<uint8_t> pixels(W * H * BPP, 0);

    f.read((char*)pixels.data(), pixels.size());
    

    frame_queue queue;
    s.start(queue);

    dev.on_video_frame(0, pixels.data(), [](void*) {}, W*BPP, BPP, 0, RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK, 1, depth);


    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;
    auto frame = queue.wait_for_frame();

    decimation_filter dec_filter;
    spatial_filter spat_filter;
    dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 1);

    auto dec_frame = dec_filter.proccess(frame);
    auto spat_frame = spat_filter.proccess(dec_frame);
    rs2::frame color = color_map(spat_frame); // Find and colorize the depth data

    auto out_file_name = "output_" + file;
    std::ofstream ofile(out_file_name, std::ofstream::out | std::ios::binary);
    auto profile = spat_frame.get_profile().as<video_stream_profile>();
    ofile.write((char*)spat_frame.get_data(), profile.width()* profile.height()*2);
    ofile.close();

    auto out_file_name_png = out_file_name + ".png";
    stbi_write_png(out_file_name_png.c_str(), profile.width(), profile.height(), 3, color.get_data(), profile.width() * 3);

    while (app) // Application still alive?
    {
       
                                           // Render depth on to the first half of the screen and color on to the second

        depth_image.render(color, { 0,               0, app.width(), app.height() });
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}



