// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#pragma once

#include <chrono>
#include <limits> // For std::numeric_limits
#include "core/processing.h"
#include "image.h"
#include "option.h"
#include "source.h"

namespace librealsense
{
    class synthetic_source : public synthetic_source_interface
    {
    public:
        synthetic_source(frame_source& actual)
            : _actual_source(actual), _c_wrapper(new rs2_source { this })
        {
        }

        frame_interface* allocate_video_frame(std::shared_ptr<stream_profile_interface> stream,
                                              frame_interface* original,
                                              int new_bpp = 0,
                                              int new_width = 0,
                                              int new_height = 0,
                                              int new_stride = 0,
                                              rs2_extension frame_type = RS2_EXTENSION_VIDEO_FRAME) override;

        frame_interface* allocate_composite_frame(std::vector<frame_holder> frames) override;

        frame_interface* allocate_points(std::shared_ptr<stream_profile_interface> stream, frame_interface* original) override;

        void frame_ready(frame_holder result) override;

        rs2_source* get_c_wrapper() override { return _c_wrapper.get(); }

    private:
        frame_source& _actual_source;
        std::shared_ptr<rs2_source> _c_wrapper;
    };

    // "Borrowed from rendering.h" - to be refactored into a common base. Evgeni
    using clock = std::chrono::high_resolution_clock;

    class benchmarker : public benchmark_interface, public readonly_option
    {
    public:

        virtual float query() const { return static_cast<float>(get_benchmark_ms()); };
        double get_benchmark_ms() const { return _sample_ms; };

        virtual option_range get_range() const { return option_range{ 0.f, std::numeric_limits<float>::max(), 1.f, 0.f }; }

        virtual bool is_enabled() const { return true; }

        virtual const char * get_description() const { return "Processing time"; }

        virtual ~benchmarker() = default;

        // Helper class to keep track of time
        class benchmark_timer
        {
        public:
            benchmark_timer()
            {
                _start = std::chrono::steady_clock::now();
            }

            void reset() { _start = std::chrono::steady_clock::now(); }

            // Get elapsed milliseconds since timer creation
            double elapsed_ms() const
            {
                return std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(elapsed()).count();
            }

            clock::duration elapsed() const
            {
                return clock::now() - _start;
            }

            clock::time_point mark()
            {
                return (_start = clock::now());
            }
        private:
            clock::time_point _start;
        };

    protected:
        friend class processing_block;
        void start(){ _timer.mark(); }
        void stop() { _sample_ms = _timer.elapsed_ms(); }

        benchmark_timer _timer;
        double _sample_ms;
    };

    class processing_block : public processing_block_interface, public options_container
    {
    public:
        processing_block();

        void set_processing_callback(frame_processor_callback_ptr callback) override;
        void set_output_callback(frame_callback_ptr callback) override;
        void invoke(frame_holder frames) override;

        synthetic_source_interface& get_source() override { return _source_wrapper; }

        virtual ~processing_block(){_source.flush();}

    protected:

        frame_source _source;
        std::mutex _mutex;
        frame_processor_callback_ptr _callback;
        synthetic_source _source_wrapper;
        rs2_extension _output_type;

        std::shared_ptr<benchmarker> _benchmark;
    };

    // Auxillary interface class for enabling ROI masking for the processing block
    // Requires to implement get() and set() functionality
    class processing_block_roi_method : public region_of_interest_method, public roi_interface
    {
    public:
        explicit processing_block_roi_method():
            //evgeni_roi_norm({0.f, 0.f, 1.f, 1.f}),
            _roi_norm({ 0.4f, 0.4f, 0.6f, 0.6f }),
            _roi_method(nullptr)
        {}

        region_of_interest_method& get_roi_method() const override
        {
            if (!_roi_method.get())
                throw librealsense::not_implemented_exception("Region-of-interest is not implemented for this device!");
            return *_roi_method;
        }

        void set_roi_method(std::shared_ptr<region_of_interest_method> roi_method) override
        {
            _roi_method = roi_method;
        }

    protected:
        normalized_region_of_interest _roi_norm;
        std::shared_ptr<region_of_interest_method> _roi_method;
    };
}
