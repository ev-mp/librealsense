/* License: Apache 2.0. See LICENSE file in root directory.
   Copyright(c) 2026 RealSense, Inc. All Rights Reserved. */

/** \file rs_hkr_improved_close_range_control.h
* \brief
* Cast-target struct for RS2_COMPOSITE_OPTION_HKR_IMPROVED_CLOSE_RANGE_CONTROL, exchanged via
* rs2_set_composite_option/rs2_get_composite_option/rs2_get_composite_option_range (see
* rs_composite_option.h). Post-design-review 7-field layout (Confluence pageId=74058359). Wire
* layout: dpp_header (see rs_dpp_header.h) + 8 fixed int32 param slots, 7 used here (1 reserved,
* MUST be zero on SET) - 38 bytes total, little-endian, #pragma pack(1). ctl_id = 0x0008
* (dpp_improved_close_range_filter), confirmed against a real device.
*
* Open item: this control activates at 720p/960p only; how that's surfaced to the host is still
* undecided, so no resolution-gating is encoded here.
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
    int32_t  filter_type;           /**< 0 = Downscale, 1 = Lookup shift - selects which of the
                                     * next two field-pairs is meaningful */
    int32_t  downscale_ratio;       /**< Meaningful when filter_type == Downscale: 1 = x2, 2 = x4.
                                     * Default 1. 0 is reserved */
    int32_t  shift_mode;            /**< Meaningful when filter_type == Lookup shift: 0 = 126px,
                                     * 1 = 64px, 2 = manual (see shift_pixels). Default 0 */
    int32_t  shift_pixels;          /**< Used when shift_mode == Manual. Range [0,256]. Default 126 */
    int32_t  threshold_mode;        /**< 0 = Zero range, 1 = MinZ range (FW-computed, not exposed
                                     * to the host), 2 = Manual (see threshold_mm). Default 0 */
    int32_t  threshold_mm;          /**< Used when threshold_mode == Manual. Range [0,65535].
                                     * Default 0 */

    int32_t  reserved[1];           /**< MUST be zero on SET */
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
