// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include "device.h"

#include "ds5-device.h"
#include "ds5-private.h"
#include "ds5-test-device.h"

namespace librealsense
{
    ds5_test_device::ds5_test_device(std::shared_ptr<context> ctx,
        const platform::backend_device_group& group)
        : device(ctx, group),
        _test_depth_device_idx(add_sensor(create_test_depth_device(ctx, group.uvc_devices)))
    {
        using namespace ds;

        auto&& backend = ctx->get_backend();

        auto& test_sensor = get_test_depth_sensor();

        if(group.usb_devices.size()>0)
        {
            _hw_monitor = std::make_shared<hw_monitor>(
                std::make_shared<locked_transfer>(
                    backend.create_usb_device(group.usb_devices.front()), test_sensor));
        }
        else
        {
            _hw_monitor = std::make_shared<hw_monitor>(
                            std::make_shared<locked_transfer>(
                                std::make_shared<command_transfer_over_xu>( get_test_depth_sensor(), depth_xu, DS5_HWMONITOR), test_sensor));
        }


        std::string device_name = (rs400_sku_names.end() != rs400_sku_names.find(group.uvc_devices.front().pid)) ? rs400_sku_names.at(group.uvc_devices.front().pid) : "RS4xx";
        _fw_version = firmware_version(_hw_monitor->get_firmware_version_string(GVD, camera_fw_version_offset));
        auto serial = _hw_monitor->get_module_serial_string(GVD, module_serial_offset);

        auto pid = group.uvc_devices.front().pid;
        auto pid_hex_str = hexify(pid>>8) + hexify(static_cast<uint8_t>(pid));

        std::string is_camera_locked{""};
        if (_fw_version >= firmware_version("5.6.3.0"))
        {
            auto is_locked = _hw_monitor->is_camera_locked(GVD, is_camera_locked_offset);
            is_camera_locked = (is_locked)?"YES":"NO";

#ifdef HWM_OVER_XU
            //if hw_monitor was created by usb replace it xu
            if (group.usb_devices.size() > 0)
            {
                _hw_monitor = std::make_shared<hw_monitor>(
                    std::make_shared<locked_transfer>(
                        std::make_shared<command_transfer_over_xu>(
                            test_sensor, librealsense::ds::depth_xu, librealsense::ds::DS5_HWMONITOR), test_sensor));
            }
#endif
        }

        if (_fw_version >= firmware_version("5.5.8.0"))
        {
            auto error_control = std::unique_ptr<uvc_xu_option<uint8_t>>(new uvc_xu_option<uint8_t>(test_sensor, depth_xu, DS5_ERROR_REPORTING, "Error reporting"));

            _polling_error_handler = std::unique_ptr<polling_error_handler>(
                new polling_error_handler(1000,
                    std::move(error_control),
                    test_sensor.get_notifications_proccessor(),
                    std::unique_ptr<notification_decoder>(new ds5_notification_decoder())));

            _polling_error_handler->start();

            test_sensor.register_option(RS2_OPTION_ERROR_POLLING_ENABLED, std::make_shared<polling_errors_disable>(_polling_error_handler.get()));
        }

        register_info(RS2_CAMERA_INFO_NAME,              device_name);
        register_info(RS2_CAMERA_INFO_SERIAL_NUMBER,     serial);
        register_info(RS2_CAMERA_INFO_FIRMWARE_VERSION,  _fw_version);
        register_info(RS2_CAMERA_INFO_LOCATION,          group.uvc_devices.front().device_path);
        register_info(RS2_CAMERA_INFO_DEBUG_OP_CODE,     std::to_string(static_cast<int>(fw_cmd::GLD)));
        register_info(RS2_CAMERA_INFO_ADVANCED_MODE,     "N/A");
        register_info(RS2_CAMERA_INFO_PRODUCT_ID,        pid_hex_str);
    }

    std::shared_ptr<uvc_sensor> ds5_test_device::create_test_depth_device(std::shared_ptr<context> ctx,
        const std::vector<platform::uvc_device_info>& all_device_infos)
    {
        using namespace ds;

        auto&& backend = ctx->get_backend();

        std::vector<std::shared_ptr<platform::uvc_device>> depth_devices;
        for (auto&& info : filter_by_mi(all_device_infos, 0)) // Filter just mi=0, DEPTH
            depth_devices.push_back(backend.create_uvc_device(info));

        auto test_sensor = std::make_shared<uvc_sensor>("Test Sensor", std::make_shared<platform::multi_pins_uvc_device>(depth_devices), nullptr, this);
        test_sensor->register_xu(depth_xu); // make sure the XU is initialized everytime we power the camera

        return test_sensor;
    }

    uint16_t ds5_test_device::get_sku_id(void) const
    {
        auto prod_id = _hw_monitor->get_product_type(ds::GVD, ds::product_type_offset);

        if (ds::rs400_fw_type_2_pid.end() == ds::rs400_fw_type_2_pid.find(prod_id))
            throw invalid_value_exception(to_string() << "Test device: unsupported product type " << prod_id);

        return ds::rs400_fw_type_2_pid.at(prod_id);
    }
}
