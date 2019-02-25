// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.
#pragma once


#include "streaming.h"

namespace librealsense
{
    class motion_stream_profile_interface : public virtual stream_profile_interface
    {
    public:
        virtual rs2_motion_device_intrinsic get_intrinsics() const = 0;
        virtual void set_intrinsics(std::function<rs2_motion_device_intrinsic()> calc) = 0;
    };

    MAP_EXTENSION(RS2_EXTENSION_MOTION_PROFILE, librealsense::motion_stream_profile_interface);

    class pose_stream_profile_interface : public virtual stream_profile_interface
    {
        //Empty for now
    };

    MAP_EXTENSION(RS2_EXTENSION_POSE_PROFILE, librealsense::pose_stream_profile_interface);

    class pose_sensor_interface : public recordable<pose_sensor_interface>
    {
    public:
        virtual bool export_relocalization_map(std::vector<uint8_t>& lmap_buf) const  = 0;
        virtual bool import_relocalization_map(const std::vector<uint8_t>& lmap_buf) const = 0;
        virtual ~pose_sensor_interface() = default;
    };
    MAP_EXTENSION(RS2_EXTENSION_POSE_SENSOR, librealsense::pose_sensor_interface);

    class pose_sensor_snapshot : /*public pose_sensor_interface,*/ public extension_snapshot
    {
    public:
        void export_relocalization_map(const std::string& lmap_fname) const
        {
            LOG_WARNING(__FUNCTION__ << " not implemented, TODO");
        }
        void import_relocalization_map(const std::string& lmap_fname) const
        {
            LOG_WARNING(__FUNCTION__ << " not implemented, TODO");
        }

        void update(std::shared_ptr<extension_snapshot> ext) override
        {
            pose_sensor_snapshot::update(ext);
            // TODO get localization map
        }

        void create_snapshot(std::shared_ptr<pose_sensor_snapshot>& snapshot) const
        {
            snapshot = std::make_shared<pose_sensor_snapshot>(*this);
        }

        void enable_recording(std::function<void(const pose_sensor_snapshot&)> recording_function)
        {
            //empty
        }

    private:
        std::vector<uint8_t> m_localization_map;
    };

    class tm2_extensions
    {
    public:
        //Empty for now
        virtual void enable_loopback(const std::string& input) = 0;
        virtual void disable_loopback() = 0;
        virtual bool is_enabled() const = 0;
        virtual void connect_controller(const std::array<uint8_t, 6>& mac_address) = 0;
        virtual void disconnect_controller(int id) = 0;
        virtual ~tm2_extensions() = default;
    };
    MAP_EXTENSION(RS2_EXTENSION_TM2, librealsense::tm2_extensions);

//    class pose_sensor : public recordable<pose_sensor>
//    {
//    public:
//        virtual void export_relocalization_map(const std::string& lmap_fname) const = 0;
//        virtual void import_relocalization_map(const std::string& lmap_fname) const = 0;
//    };

//    MAP_EXTENSION(RS2_EXTENSION_POSE_SENSOR, librealsense::pose_sensor);


}
