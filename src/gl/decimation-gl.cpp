// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#include <iostream>
#include <chrono>
#include <strstream>
//#include <glad/glad.h>

//#include "opengl3.h"
#include "option.h"
#include "../include/librealsense2-gl/rs_processing_gl.hpp"
#include "decimation-gl.h"

using namespace rs2;
using namespace librealsense::gl;

static const char* decimation_fragment_shader_text =
"#version 130\n"
"in vec2 textCoords;\n"
"uniform sampler2D textureSampler;\n"
"uniform uint decimation_factor;\n"
"uniform uint width;\n"
"uniform uint height;\n"
"precision mediump float;\n"
"void main(void) {\n"
"    float pixel_width = 1.0 / width;\n"
"    float pixel_height = 1.0 / height;\n"
"    float y = 0.0;\n"
"    float u = 0.0;\n"
"    float v = 0.0;\n"
"    float tex_y = 1.0 - textCoords.y;\n"
"    //if (mod(floor(gl_FragCoord.x), 2.0) == 0.0){\n"
"    //    vec2 tx1 = vec2(textCoords.x, tex_y);\n"
"    //    vec4 px1 = texture2D(textureSampler, tx1);\n"
"    //    vec2 tx2 = vec2(textCoords.x + pixel_width, tex_y);\n"
"    //    vec4 px2 = texture2D(textureSampler, tx2);\n"
"    //    y = px1.x; u = px1.y; v = px2.y;\n"
"    //}\n"
"    //else\n"
"    //{\n"
"    //    vec2 tx1 = vec2(textCoords.x - pixel_width, tex_y);\n"
"    //    vec4 px1 = texture2D(textureSampler, tx1);\n"
"    //    vec2 tx2 = vec2(textCoords.x, tex_y);\n"
"    //    vec4 px2 = texture2D(textureSampler, tx2);\n"
"    //    y = px2.x; u = px1.y; v = px2.y;\n"
"    //}\n"
"    ////y *= 256.0; u *= 256.0; v *= 256.0;\n"
"    //float c = y - (16.0 / 256.0);\n"
"    //float d = u - 0.5;\n"
"    //float e = v - 0.5;\n"
"    //vec3 color = vec3(0.0);\n"
"    ////color.x = clamp(((298.0 / 256.0) * c + (409.0 / 256.0) * e + 0.5), 0.0, 1.0);\n"
"    ////color.y = clamp(((298.0 / 256.0) * c - (100.0 / 256.0) * d - (208.0/256.0) * e + 0.5), 0.0, 1.0);\n"
"    ////color.z = clamp(((298.0 / 256.0) * c + (516.0 / 256.0) * d + 0.5), 0.0, 1.0);\n"
"    //color.x = clamp((y + 1.40200 * (v - 0.5)), 0.0, 1.0);\n"
"    //color.y = clamp((y - 0.34414 * (u - 0.5) - 0.71414 * (v - 0.5)), 0.0, 1.0);\n"
"    //color.z = clamp((y + 1.77200 * (u - 0.5)), 0.0, 1.0);\n"
"    //gl_FragColor = vec4(color.xyz, opacity);\n"
"    //Evgeni - todo\n"
"    gl_FragColor = vec4(1., 1., 0., 0.5);\n"
"}";

class decimation_shader : public texture_2d_shader
{
public:
    decimation_shader()
        : texture_2d_shader(shader_program::load(
            texture_2d_shader::default_vertex_shader(), 
            decimation_fragment_shader_text, "position", "textureCoords"))
    {
        _width_location = _shader->get_uniform_location("width");
        _height_location = _shader->get_uniform_location("height");
    }

    void set_size(int w, int h)
    {
        _shader->load_uniform(_width_location, (float)w);
        _shader->load_uniform(_height_location, (float)h);
    }

private:
    uint32_t _width_location;
    uint32_t _height_location;
};

void decimation_filter_gl::cleanup_gpu_resources()
{
    _viz.reset();
    _fbo.reset();
    _enabled = 0;
}

void decimation_filter_gl::create_gpu_resources()
{
    _viz = std::make_shared<visualizer_2d>(std::make_shared<decimation_shader>());
    _fbo = std::make_shared<fbo>(_padded_width, _padded_height);
    _enabled = glsl_enabled() ? 1 : 0;
}

decimation_filter_gl::decimation_filter_gl() : decimation_filter()
{
    _source.add_extension<gpu_video_frame>(static_cast<rs2_extension>(RS2_EXTENSION_VIDEO_FRAME_GL));

    auto opt = std::make_shared<librealsense::ptr_option<int>>(0, 1, 0, 1, &_enabled, "GLSL enabled");
    register_option(RS2_OPTION_COUNT, opt);

    initialize();
}

decimation_filter_gl::~decimation_filter_gl()
{
    perform_gl_action([&]()
    {
        cleanup_gpu_resources();
    }, []{});
}

rs2::frame decimation_filter_gl::process_frame(const rs2::frame_source& src, const rs2::frame& f)
{
    scoped_timer t(__FUNCTION__);

    // Update input/output profiles
    if (update_output_profile(f))
    {
        perform_gl_action([&]()
        {
            _fbo = std::make_shared<fbo>(_padded_height, _padded_height);
        }, [this] {
            _enabled = false;
        });
    }

    rs2::frame res  = f;
    rs2::stream_profile profile = f.get_profile();
    rs2_format format = profile.format();
    rs2_stream type = profile.stream_type();

    perform_gl_action([&]()
    {
        scoped_timer t(__FUNCTION__);

        frame_type tgt_type{};
        uint16_t gl_internal_type{};
        uint16_t gl_internal_format{};
        switch (type)
        {
            case RS2_STREAM_COLOR:
            {
                tgt_type = RS2_EXTENSION_VIDEO_FRAME_GL;
                bool rgb_cs = val_in_range(format, { RS2_FORMAT_RGB8, RS2_FORMAT_BGR8 });
                gl_internal_format = rgb_cs ? GL_RGB : GL_RG;
                gl_internal_type = rgb_cs ? GL_RGB8 : GL_RG8;
                break;
            }
            case RS2_STREAM_INFRARED:
            case RS2_STREAM_FISHEYE:
                tgt_type = RS2_EXTENSION_VIDEO_FRAME_GL;
                gl_internal_format  = GL_RED;
                gl_internal_type = val_in_range(format, { RS2_FORMAT_RGB8, RS2_FORMAT_BGR8 }) ? GL_BYTE : GL_UNSIGNED_SHORT;
                break;
            case RS2_STREAM_DEPTH:
            {
                // TODO Decimation of disparity format to be evaluated
                auto disp = f.is<rs2::disparity_frame>();
                tgt_type = disp ? RS2_EXTENSION_DISPARITY_FRAME_GL : RS2_EXTENSION_DEPTH_FRAME_GL;
                gl_internal_format = GL_RED;
                gl_internal_type = disp ? GL_FLOAT : GL_UNSIGNED_SHORT;
            }
            break;
            default:
                LOG_WARNING("Unsupported format type " << format << " for GLSL Decimation");
        }

        if (auto tgt = prepare_target_frame(f, src, static_cast<rs2_extension>(tgt_type)))
        {
            auto gf = dynamic_cast<gpu_addon_interface*>((frame_interface*)res.get());

            uint32_t work_texture;

            if (auto input_frame = f.as<rs2::gl::gpu_frame>())
            {
                work_texture = input_frame.get_texture_id(0);
            }
            else
            {
                glGenTextures(1, &work_texture);
                glBindTexture(GL_TEXTURE_2D, work_texture);
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RG8, _padded_width, _padded_height, 0, GL_RG, GL_UNSIGNED_BYTE, f.get_data());
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            }

            uint32_t output_decimated;
            gf->get_gpu_section().output_texture(0, &output_decimated, TEXTYPE_RGB);
            glBindTexture(GL_TEXTURE_2D, output_decimated);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, _padded_width, _padded_height, 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

            gf->get_gpu_section().set_size(_padded_width, _padded_height);

            glBindFramebuffer(GL_FRAMEBUFFER, _fbo->get());
            glDrawBuffer(GL_COLOR_ATTACHMENT0);

            glBindTexture(GL_TEXTURE_2D, output_decimated);
            _fbo->createTextureAttachment(output_decimated);

            _fbo->bind();
            glClearColor(1, 0, 0, 1);
            glClear(GL_COLOR_BUFFER_BIT);

            auto& shader = (decimation_shader&)_viz->get_shader();
            shader.begin();
            shader.set_size(_padded_width, _padded_height);
            shader.end();

            _viz->draw_texture(work_texture);

            _fbo->unbind();

            glBindTexture(GL_TEXTURE_2D, 0);

            if (!f.is<rs2::gl::gpu_frame>())
            {
                glDeleteTextures(1, &work_texture);
            }
        }
    }, 
    [this]{
        _enabled = false;
    }); 

    return res;
}