// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#pragma once

namespace librealsense
{
    class ds5_test_device final : public device
    {
    public:

        ds5_test_device(std::shared_ptr<context> ctx, const platform::backend_device_group& group);

        uint16_t get_sku_id(void) const;

    protected:
        std::shared_ptr<hw_monitor> _hw_monitor;
        firmware_version _fw_version;

        std::shared_ptr<uvc_sensor> create_test_depth_device(std::shared_ptr<context> ctx,
            const std::vector<platform::uvc_device_info>& all_device_infos);

        uvc_sensor& get_test_depth_sensor()
        {
            return dynamic_cast<uvc_sensor&>(get_sensor(_test_depth_device_idx));
        }

    private:

        uint8_t _test_depth_device_idx;

        std::unique_ptr<polling_error_handler> _polling_error_handler;
    };
}
