// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

/** \file rs_dpp_header.h
* \brief
* Wire header shared by every control in the HKR Depth Post-Processing (DPP) composite-option
* family. Each DPP control's wire payload is this header followed by a fixed 8-slot int32
* parameter block; param_count says how many of those 8 are active for that particular control,
* the rest MUST be zero on SET. See rs_hkr_hdrd_control.h for a control that uses it today.
*/

#ifndef LIBREALSENSE_RS2_DPP_HEADER_H
#define LIBREALSENSE_RS2_DPP_HEADER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** The only dpp_header wire version this SDK build knows how to interpret. Any composite-option
* struct read back with a different (including 0, the "never populated" sentinel) `header.version`
* is rejected - see rs_options.hpp's get_composite_option_as()/get_composite_option_range_as() -
* rather than having its fields silently misread against a layout this build doesn't actually
* speak. */
#define DPP_HEADER_CURRENT_VERSION 1

#pragma pack(push, 1)

typedef struct dpp_header
{
    uint8_t  version;       /**< Wire struct version - see DPP_HEADER_CURRENT_VERSION */
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
