// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

/** \file rs_dpp_header.h
* \brief
* Wire header shared by every control in the HKR Depth Post-Processing (DPP) composite-option
* family. Each DPP control's wire payload is this header followed by a fixed 8-slot int32
* parameter block; param_count says how many of those 8 are active for that particular control,
* the rest MUST be zero on SET. See rs_hkr_improved_close_range_control.h for a control that uses it today.
*/

#ifndef LIBREALSENSE_RS2_DPP_HEADER_H
#define LIBREALSENSE_RS2_DPP_HEADER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#pragma pack(push, 1)

typedef struct dpp_header
{
    uint8_t  version;       /**< Wire struct version. Currently 0x01 */
    uint8_t  flags;         /**< Bitwise control-status mask (active/read-only) */
    uint16_t ctl_id;        /**< dpp_ctrl_list entry identifying this control */
    uint8_t  param_count;   /**< Active param slots out of the fixed 8 that follow */
    uint8_t  param_type;    /**< Per-param int/float bitmap. 0x00 = all-integer */
} dpp_header;

#pragma pack(pop)

#ifdef __cplusplus
}
#endif
#endif  // LIBREALSENSE_RS2_DPP_HEADER_H
