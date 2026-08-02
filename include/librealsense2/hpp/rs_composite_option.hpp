// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

// PROTOTYPE / DEMO API - not a finalized production interface. See rs_composite_option.h for
// the underlying generic C API and the atomicity contract (single UVC transaction per call).

#ifndef LIBREALSENSE_RS2_COMPOSITE_OPTION_HPP
#define LIBREALSENSE_RS2_COMPOSITE_OPTION_HPP

#include "rs_sensor.hpp"
#include "../h/rs_composite_option.h"

namespace rs2
{
    // Generic "composite option" extension: any sensor that supports AT LEAST ONE composite
    // (multi-field, atomically-exchanged) option extends to this - see is<T>()/as<T>() in
    // rs_types.hpp, backed by rs2_is_sensor_extendable_to(RS2_EXTENSION_COMPOSITE_OPTIONS).
    // There is one wrapper class for every composite option, not one per feature: which option
    // you are getting/setting is selected by the rs2_composite_option_id argument.
    class composite_option_sensor : public sensor
    {
    public:
        composite_option_sensor(sensor s)
            : sensor(s.get())
        {
            rs2_error* e = nullptr;
            if (rs2_is_sensor_extendable_to(_sensor.get(), RS2_EXTENSION_COMPOSITE_OPTIONS, &e) == 0 && !e)
            {
                _sensor.reset();
            }
            error::handle(e);
        }

        operator bool() const { return _sensor.get() != nullptr; }

        // Single atomic UVC transaction (one set_xu round trip) - all fields of *data sent together.
        void set_composite_option(rs2_composite_option_id option_id, const void* data, unsigned int data_size) const
        {
            rs2_error* e = nullptr;
            rs2_set_composite_option(_sensor.get(), option_id, data, data_size, &e);
            error::handle(e);
        }

        // Single atomic UVC transaction (one get_xu round trip). The SDK allocates the result
        // (the caller has no generic way to know option_id's wire size in advance) and this
        // wrapper hides the raw rs2_raw_data_buffer/manual-free entirely, returning a plain
        // std::vector<uint8_t> of the option's raw payload bytes - cast/memcpy them into the
        // typed struct that corresponds to option_id (e.g. rs2_temporal_filter_dpp_config).
        std::vector<uint8_t> get_composite_option(rs2_composite_option_id option_id) const
        {
            rs2_error* e = nullptr;
            auto buffer = rs2_get_composite_option(_sensor.get(), option_id, &e);

            std::shared_ptr<const rs2_raw_data_buffer> list(buffer, rs2_delete_raw_data);
            error::handle(e);

            auto size = rs2_get_raw_data_size(list.get(), &e);
            error::handle(e);

            auto start = rs2_get_raw_data(list.get(), &e);
            error::handle(e);

            std::vector<uint8_t> result;
            result.insert(result.begin(), start, start + size);

            return result;
        }
    };
}
#endif // LIBREALSENSE_RS2_COMPOSITE_OPTION_HPP
