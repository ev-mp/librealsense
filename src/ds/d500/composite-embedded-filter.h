// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

#pragma once

#include <src/uvc-sensor.h>
#include <librealsense2/h/rs_composite_option.h>

#include <memory>
#include <cstdint>
#include <string>


namespace librealsense {

// Generic HKR/D5X5 composite-option embedded filter: registers ONE composite_xu_option (see
// src/ds/composite-xu-option.h) under `option_id`, in this filter's OWN options container
// (inherited via `Base` -> embedded_filter_base) - NOT directly on d500_depth_sensor. `Base`
// supplies the RS2_EXTENSION_* identity (e.g. temporal-embedded-filter.h), `Type` the
// rs2_embedded_filter_type it reports; every other difference between composite-option filters
// (ctrl_id/wire_size/option_id/description) is a constructor argument, not a subclass - see
// hdrd-embedded-filter.h and temporal-filter-feature.cpp for the two current instantiations.
template< class Base, rs2_embedded_filter_type Type >
class composite_embedded_filter : public Base
{
public:
    composite_embedded_filter( std::weak_ptr< uvc_sensor > raw_depth_ep,
                                uint8_t ctrl_id,
                                uint32_t wire_size,
                                rs2_composite_option_id option_id,
                                std::string description );

    rs2_embedded_filter_type get_type() const override { return Type; }
};

}  // namespace librealsense
