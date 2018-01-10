/* License: Apache 2.0. See LICENSE file in root directory.
   Copyright(c) 2017 Intel Corporation. All Rights Reserved. */

/** \file rs2_internal.h
* \brief
* Exposes RealSense internal functionality for C compilers
*/

#ifndef LIBREALSENSE_RS2_EXTENSION_H
#define LIBREALSENSE_RS2_EXTENSION_H

#ifdef __cplusplus
extern "C" {
#endif
#include "../rs.h"

// Evgeni TODO - Update documentation for all ctor/dtors
/**
* create sensor by index
* \param[in] index   the zero based index of sensor to retrieve
* \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
* \return            the requested sensor, should be released by rs2_delete_sensor
*/
rs2_roi* rs2_create_sensor_roi(const rs2_sensor* sensor, rs2_error** error);

/**
* create sensor by index
* \param[in] index   the zero based index of sensor to retrieve
* \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
* \return            the requested sensor, should be released by rs2_delete_sensor
*/
rs2_roi* rs2_create_processing_block_roi(const rs2_processing_block* pb, rs2_error** error);


/**
* delete relasense sensor
* \param[in] sensor realsense sensor to delete
*/
void rs2_delete_roi(rs2_roi* roi);


#ifdef __cplusplus
}
#endif
#endif
