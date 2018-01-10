// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#ifndef LIBREALSENSE_RS2_EXTENSION_HPP
#define LIBREALSENSE_RS2_EXTENSION_HPP

//#include "rs_types.hpp"
#include "rs_frame.hpp"
#include "rs_sensor.hpp"
#include "rs_processing.hpp"
#include "../h/rs_extension.h"

/*
//class roi_processing_block : processing_block//, public roi_interface
//{
//public:
//    roi_processing_block(processing_block block) : processing_block(block)//, roi_interface()
//    {
//        rs2_error* e = nullptr;
//        if (rs2_is_processing_block_extendable_to(this->get(), RS2_EXTENSION_ROI, &e) == 0 && !e)
//        {
//            _block = nullptr;
//        }
//        error::handle(e);
//    }

//    //void set_region_of_interest(const region_of_interest& roi)
//    //{
//    //    rs2_error* e = nullptr;
//    //    rs2_set_region_of_interest(_sensor.get(), roi.min_x, roi.min_y, roi.max_x, roi.max_y, &e);
//    //    error::handle(e);
//    //}

//    //region_of_interest get_region_of_interest() const
//    //{
//    //    region_of_interest roi {};
//    //    rs2_error* e = nullptr;
//    //    rs2_get_region_of_interest(_sensor.get(), &roi.min_x, &roi.min_y, &roi.max_x, &roi.max_y, &e);
//    //    error::handle(e);
//    //    return roi;
//    //}

//    operator bool() const { return _block.get() != nullptr; }
//};
*/

namespace rs2
{
    class roi_sensor;
    class processing_block;

    class roi_interface
    {
    public:
        explicit roi_interface(rs2::sensor s):
            _roi(nullptr)
        { 
            rs2_error* e = nullptr;
            if (rs2_is_sensor_extendable_to(s.get().get(), RS2_EXTENSION_ROI, &e) == 0 && !e)
            {
                _roi = nullptr;
            }
            else
            {
                // everything is OK
                _roi = std::shared_ptr<rs2_roi>(
                    rs2_create_sensor_roi( s.get().get(), &e),
                    rs2_delete_roi);
            }
            error::handle(e);
        }

        // Evgeni
        //explicit roi_interface(processing_block block):
        //    _roi(nullptr)
        //{ 
        //    rs2_error* e = nullptr;
        //    if (0 != rs2_is_processing_block_extendable_to(block.get(), RS2_EXTENSION_ROI, &e) && !e)
        //    {
        //        // everything is OK
        //        _roi = std::shared_ptr<rs2_roi>(
        //            rs2_create_processing_block_roi(block.get(), &e),
        //            rs2_delete_roi);
        //    }
        //    else
        //        block = nullptr;
        //    error::handle(e);
        //}

        void set_region_of_interest(const region_of_interest& roi)
        {
            rs2_error* e = nullptr;
            rs2_set_region_of_interest(_roi.get(), roi.min_x, roi.min_y, roi.max_x, roi.max_y, &e);
            error::handle(e);
        }

        virtual region_of_interest get_region_of_interest() const
        {
            region_of_interest roi{};
            rs2_error* e = nullptr;
            rs2_get_region_of_interest(_roi.get(), &roi.min_x, &roi.min_y, &roi.max_x, &roi.max_y, &e);
            error::handle(e);
            return roi;
        }

    private:
        std::shared_ptr<rs2_roi>  _roi;
    };

    // Evgeni WIP
    //class roi_interface;

    class roi_sensor : public sensor, public roi_interface
    {
    public:
        roi_sensor(sensor s) : sensor(s.get()), roi_interface(s)
        {
            /*rs2_error* e = nullptr;
            if (rs2_is_sensor_extendable_to(_sensor.get(), RS2_EXTENSION_ROI, &e) == 0 && !e)
            {
                _sensor = nullptr;
            }
            error::handle(e);*/
        }

        //void set_region_of_interest(const region_of_interest& roi)
        //{
        //    rs2_error* e = nullptr;
        //    rs2_set_region_of_interest(_sensor.get(), roi.min_x, roi.min_y, roi.max_x, roi.max_y, &e);
        //    error::handle(e);
        //}

        //region_of_interest get_region_of_interest() const
        //{
        //    region_of_interest roi {};
        //    rs2_error* e = nullptr;
        //    rs2_get_region_of_interest(_sensor.get(), &roi.min_x, &roi.min_y, &roi.max_x, &roi.max_y, &e);
        //    error::handle(e);
        //    return roi;
        //}

        operator bool() const { return _sensor.get() != nullptr; }
    };

    //class roi_processing_block : processing_block//, public roi_interface
    //{
    //public:
    //    roi_processing_block(processing_block block) : processing_block(block)//, roi_interface()
    //    {
    //        rs2_error* e = nullptr;
    //        if (rs2_is_processing_block_extendable_to(this->get(), RS2_EXTENSION_ROI, &e) == 0 && !e)
    //        {
    //            _block = nullptr;
    //        }
    //        error::handle(e);
    //    }

    //    //void set_region_of_interest(const region_of_interest& roi)
    //    //{
    //    //    rs2_error* e = nullptr;
    //    //    rs2_set_region_of_interest(_sensor.get(), roi.min_x, roi.min_y, roi.max_x, roi.max_y, &e);
    //    //    error::handle(e);
    //    //}

    //    //region_of_interest get_region_of_interest() const
    //    //{
    //    //    region_of_interest roi {};
    //    //    rs2_error* e = nullptr;
    //    //    rs2_get_region_of_interest(_sensor.get(), &roi.min_x, &roi.min_y, &roi.max_x, &roi.max_y, &e);
    //    //    error::handle(e);
    //    //    return roi;
    //    //}

    //    operator bool() const { return _block.get() != nullptr; }
    //};
}
#endif // LIBREALSENSE_RS2_EXTENSION_HPP
