// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#pragma once

//#include <memory>
//#include <map>
//#include <vector>

#include "proc/decimation-filter.h"
#include "synthetic-stream-gl.h"
//#include <librealsense2/rs.hpp>

namespace rs2
{
    class stream_profile;
    class visualizer_2d;
}

namespace librealsense 
{
    namespace gl
    {
        class decimation_filter_gl : public decimation_filter, public gpu_processing_object
        {
        public:
            decimation_filter_gl();
            ~decimation_filter_gl() override;

            void cleanup_gpu_resources() override;
            void create_gpu_resources() override;

            rs2::frame process_frame(const rs2::frame_source& source, const rs2::frame& f) override;
        private:
            int _enabled = 0;

            std::shared_ptr<rs2::visualizer_2d> _viz;
            std::shared_ptr<rs2::fbo> _fbo;
        };
    }
}
