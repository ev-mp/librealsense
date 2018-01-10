// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved
//// In-place Domain Transform Edge-preserving filter based on
// http://inf.ufrgs.br/~eslgastal/DomainTransform/Gastal_Oliveira_SIGGRAPH2011_Domain_Transform.pdf

#pragma once

#include <map>
#include <vector>
#include <cmath>

#include "../include/librealsense2/hpp/rs_frame.hpp"
#include "../include/librealsense2/hpp/rs_processing.hpp"

namespace librealsense
{
    class spatial_filter : public processing_block, public processing_block_roi_method
    {
    public:
        spatial_filter();

        void set(const region_of_interest& roi) override;
        region_of_interest get() const override;

    protected:
        void    update_configuration(const rs2::frame& f);

        rs2::frame prepare_target_frame(const rs2::frame& f, const rs2::frame_source& source);

        template <typename T>
        void dxf_smooth(void *frame_data, float alpha, float delta, int iterations)
        {
            static_assert((std::is_arithmetic<T>::value), "Spatial filter assumes numeric types");

            for (int i = 0; i < iterations; i++)
            {
                recursive_filter_horizontal<T>(frame_data, alpha, delta);
                recursive_filter_vertical<T>(frame_data, alpha, delta);
            }
        }

        template <typename T>
        void  recursive_filter_horizontal(void * image_data, float alpha, float deltaZ)
        {
            size_t v{}, u{};

            // Handle conversions for invalid input data
            bool fp = (std::is_floating_point<T>::value);

            // Filtering integer values requires round-up to the nearest discrete value
            const float round = fp ? 0.f : 0.5f;
            // Disparity value of 0.001 corresponds to 0.5 mm at 0.5 meter to 5 mm at 5m
            // For Depth values the smoothing will take effect when the gradient is more than 4 level (~0.4mm)
            const T noise = fp ? static_cast<T>(0.001f) : static_cast<T>(4);
            const T max_radius = static_cast<T>(fp ? 2.f : deltaZ);

            auto image = reinterpret_cast<T*>(image_data);

            //Evgenifor (v = 0; v < _height; v++)
            for (v = _selected_roi.min_y; v < _selected_roi.max_y; v++)
            {
                // left to right
                T *im = image + v * _width + _selected_roi.min_x;
                T val0 = im[0];
                //Evgeni for (u = 1; u < _width; u++)
                for (u = _selected_roi.min_x+1; u < _selected_roi.max_x; u++)
                {
                    T val1 = im[1];

                    if (val0 >= noise && val1 >= noise)
                    {
                        T diff = static_cast<T>(fabs(val1 - val0));

                        if (diff >= noise && diff <= max_radius)
                        {
                            float filtered = val1 * alpha + val0 * (1.0f - alpha);
                            val0 = static_cast<T>(filtered + round);
                            im[1] = val0;
                        }
                        else
                            val0 = val1;
                    }
                    else
                        val0 = val1;

                    im += 1;
                }

                // right to left
                //Evgeni im = image + (v + 1) * _width - 2;  // end of row - two pixels
                im = image + v * _width + _selected_roi .max_x - 2;  // end of ROI row - two pixels
                T val1 = im[1];

                //Evgeni for (u = _width - 1; u > 0; u--)
                for (u = _selected_roi.max_x - 1; u > _selected_roi.min_x; u--)
                {
                    T val0 = im[0];

                    if (val1 >= noise && val0> noise)
                    {
                        T diff = static_cast<T>(fabs(val1 - val0));

                        if (diff >= noise && diff <= max_radius)
                        {
                            float filtered = val0 * alpha + val1 * (1.0f - alpha);
                            val1 = static_cast<T>(filtered + round);
                            im[0] = val1;
                        }
                        else
                            val1 = val0;
                    }
                    else
                        val1 = val0;

                    im -= 1;
                }
            }
        }

        template <typename T>
        void recursive_filter_vertical(void * image_data, float alpha, float deltaZ)
        {
            size_t v{}, u{};

            // Handle conversions for invalid input data
            constexpr bool fp = (std::is_floating_point<T>::value);

            // Filtering integer values requires round-up to the nearest discrete value
            constexpr float round = fp ? 0.f : 0.5f;
            // Disparity value of 0.001 corresponds to 0.5 mm at 0.5 meter to 5 mm at 5m
            // For Depth values the smoothing will take effect when the gradient is more than 4 level (~0.4mm)
            constexpr T noise = fp ? static_cast<T>(0.001f) : static_cast<T>(4);
            const T max_radius = static_cast<T>(fp ? 2.f : deltaZ);

            auto image = reinterpret_cast<T*>(image_data);

            // we'll do one row at a time, top to bottom, then bottom to top

            // top to bottom

            T *im = image;
            T im0{};
            T imw{};
            //Evgeni for (v = 1; v < _height; v++)
            for (v = _selected_roi.min_y + 1 ; v < _selected_roi.max_y; v++)
            {
                //Evgeni - small refactoring
                im = &image[v*_width];

                //Evgeni for (u = 0; u < _width; u++)
                for (u = _selected_roi.min_x ; u < _selected_roi.max_x; u++)
                {
                    //Evgeni
                    /*im0 = im[0];
                    imw = im[_width];*/
                    im0 = im[ u];
                    imw = im[ u + _width];

                    if ((im0 >noise) && (imw > noise))
                    {
                        float delta = static_cast<float>(fabs(im0 - imw));
                        if (delta > noise && delta < max_radius)
                        {
                            float filtered = imw * alpha + im0 * (1.f - alpha);
                            im[_width + u] = static_cast<T>(filtered + round);
                        }
                    }
                    im += 1;
                }
            }

            // bottom to top
            im = image + (_height - 2) * _width;
            //Evgeni for (v = 1; v < _height; v++, im -= (_width * 2))
            for (v = _selected_roi.min_y + 1; v < _selected_roi.max_y; v++, im -= (_width * 2))
            {
                //Evgeni for (u = 0; u < _width; u++)
                for (u = _selected_roi.min_x; u < _selected_roi.max_x; u++)
                {
                    //Evgeni
                    /*im0 = im[0];
                    imw = im[_width];*/
                    im0 = im[u];
                    imw = im[u + _width];

                    if ((im0 >noise) && (imw > noise))
                    {
                        float delta = static_cast<float>(fabs(im0 - imw));
                        if (delta > noise && delta < max_radius)
                        {
                            float filtered = im0 * alpha + imw * (1.f - alpha);
                            //Evgeni im[0] = static_cast<T>(filtered + round);
                            im[u] = static_cast<T>(filtered + round);
                        }
                    }
                    im += 1;
                }
            }
        }

    private:

        float                   _spatial_alpha_param;
        uint8_t                 _spatial_delta_param;
        uint8_t                 _spatial_iterations;
        float                   _spatial_radius;            // The convolution radius is domain-dependent
        size_t                  _width, _height, _stride;
        size_t                  _bpp;
        rs2_extension           _extension_type;            // Strictly Depth/Disparity
        size_t                  _current_frm_size_pixels;
        rs2::stream_profile     _source_stream_profile;
        rs2::stream_profile     _target_stream_profile;
        bool                    _stereoscopic_depth;
        float                   _focal_lenght_mm;
        float                   _stereo_baseline_mm;
        region_of_interest      _selected_roi;
    };
}
