// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.
#pragma once

#include "video.h"

#include <memory>

namespace librealsense
{
    struct region_of_interest
    {
        int min_x;
        int min_y;
        int max_x;
        int max_y;
    };

    // Axis normalized to [0..1] range
    struct normalized_region_of_interest
    {
        float min_x_norm;
        float min_y_norm;
        float max_x_norm;
        float max_y_norm;
    };

    class region_of_interest_method
    {
    public:
        virtual void set(const region_of_interest& roi) = 0;
        virtual region_of_interest get() const = 0;

        virtual ~region_of_interest_method() = default;
    };

    class roi_interface
    {
    public:
        virtual region_of_interest_method& get_roi_method() const = 0;
        virtual void set_roi_method(std::shared_ptr<region_of_interest_method> roi_method) = 0;
    };

    MAP_EXTENSION(RS2_EXTENSION_ROI, librealsense::roi_interface);
}
