/* License: Apache 2.0. See LICENSE file in root directory.
   Copyright(c) 2026 RealSense, Inc. All Rights Reserved. */

/** \file rs_hkr_improved_close_range_control.h
* \brief
* Cast-target struct for RS2_COMPOSITE_OPTION_HKR_IMPROVED_CLOSE_RANGE_CONTROL, exchanged via
* rs2_set_composite_option/rs2_get_composite_option/rs2_get_composite_option_range (see
* rs_composite_option.h). Wire layout: dpp_header (see rs_dpp_header.h) + 8 fixed int32 param
* slots, 5 used here (3 reserved, MUST be zero on SET) - 38 bytes total, little-endian,
* #pragma pack(1). ctl_id = 0x0008 (dpp_improved_close_range_filter), confirmed against a real device.
*/

#ifndef LIBREALSENSE_RS2_HKR_IMPROVED_CLOSE_RANGE_CONTROL_H
#define LIBREALSENSE_RS2_HKR_IMPROVED_CLOSE_RANGE_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rs_dpp_header.h"
#include <stdint.h>

#pragma pack(push, 1)

/** The entire 38-byte HKR Improved Close Range payload, header included. */
typedef struct rs2_improved_close_range_control
{
    dpp_header header;

    int32_t  enable;                /**< 0 = Off, 1 = On. Default 0 */
    int32_t  downscale_ratio;       /**< Secondary path resolution scale: 1, 2, or 4. Default 2.
                                     * Pre-stream only */
    int32_t  disparity_shift;       /**< SCP disparity shift for the Improved Close Range depth path. Range
                                     * [0,512]. Default 0 */
    int32_t  threshold;             /**< mm. SET: desired manual threshold; GET: effective
                                     * threshold. Range [0,65535] */
    int32_t  threshold_mode;        /**< 0 = Auto (firmware-computed), 1 = Manual (host-supplied).
                                     * Default 0 */

    int32_t  reserved[3];           /**< MUST be zero on SET */
} rs2_improved_close_range_control;

#pragma pack(pop)

/* Fails to compile if padding/field changes push this off the documented 38-byte wire size. */
typedef char rs2_improved_close_range_control_wire_size_check[ ( sizeof( rs2_improved_close_range_control ) == 38 ) ? 1 : -1 ];

/** {min, max, step, def} bounds for rs2_improved_close_range_control, as returned by
* rs2_get_composite_option_range(RS2_COMPOSITE_OPTION_HKR_IMPROVED_CLOSE_RANGE_CONTROL). Read-only - each
* bound already carries its own header.version, so this wrapper has no version field of its own. */
typedef struct rs2_improved_close_range_control_range
{
    rs2_improved_close_range_control min;
    rs2_improved_close_range_control max;
    rs2_improved_close_range_control step;
    rs2_improved_close_range_control def;
} rs2_improved_close_range_control_range;

#ifdef __cplusplus
}
#endif
#endif  // LIBREALSENSE_RS2_HKR_IMPROVED_CLOSE_RANGE_CONTROL_H
