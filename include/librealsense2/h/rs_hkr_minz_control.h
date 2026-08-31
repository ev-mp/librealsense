/* License: Apache 2.0. See LICENSE file in root directory.
   Copyright(c) 2026 RealSense, Inc. All Rights Reserved. */

/** \file rs_hkr_minz_control.h
* \brief
* PROTOTYPE / DEMO API - not a finalized production interface.
*
* Cast-target structs for RS2_COMPOSITE_OPTION_HKR_MINZ_CONTROL, a composite XU control accessed
* via rs2_set_composite_option/rs2_get_composite_option/rs2_get_composite_option_range (see
* rs_composite_option.h). Post-design-review 7-field layout (Confluence pageId=74058359),
* superseding the 5-field layout on branch proto/structured-api-temporal-filter.
*
* Wire: 6-byte dppc_header (version, flags, ctl_id) + param_count/param_type + 8 int32 param
* slots, 7 used here (1 reserved, MUST be zero on SET) - 38 bytes total, little-endian,
* #pragma pack(1). Header values unchanged from the pre-review layout (version=0x01, flags=0x01,
* ctl_id=0x0008, XU unit 3 selector 0x14) - only the logical param layout changed.
*
* Open item: MinZ activates at 720p/960p only; how that's surfaced to the host is still
* undecided, so no resolution-gating is encoded here.
*/

#ifndef LIBREALSENSE_RS2_HKR_MINZ_CONTROL_H
#define LIBREALSENSE_RS2_HKR_MINZ_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#pragma pack(push, 1)

/** The entire 38-byte HKR MinZ payload, header fields included. */
typedef struct rs2_minz_control
{
    uint8_t  version;               /**< dppc_header wire version. Currently 0x01 */
    uint8_t  flags;                 /**< dppc_header status bitmask (active/read-only) */
    uint16_t ctl_id;                /**< dpp_minz_filter = 0x0008 */
    uint8_t  param_count;           /**< 7 */
    uint8_t  param_type;            /**< 0x00 (all-integer) */

    int32_t  enable;                /**< 0 = Off, 1 = On. Default 0 */
    int32_t  filter_type;           /**< 0 = Downscale, 1 = Lookup shift - selects which of the next
                                     * two field-pairs is meaningful */
    int32_t  downscale_ratio;       /**< Meaningful when filter_type == Downscale: 1 = x2, 2 = x4.
                                     * Default 1. 0 is reserved */
    int32_t  shift_mode;            /**< Meaningful when filter_type == Lookup shift: 0 = 126px,
                                     * 1 = 64px, 2 = manual (see shift_pixels). Default 0 */
    int32_t  shift_pixels;          /**< Used when shift_mode == Manual. Range [0,256]. Default 126 */
    int32_t  threshold_mode;        /**< 0 = Zero range, 1 = MinZ range (FW-computed, not exposed to
                                     * the host), 2 = Manual (see threshold_mm). Default 0 */
    int32_t  threshold_mm;          /**< Used when threshold_mode == Manual. Range [0,65535]. Default 0 */

    int32_t  reserved[1];           /**< MUST be zero on SET */
} rs2_minz_control;

#pragma pack(pop)

/* Fails to compile if padding/field changes push this off the documented 38-byte wire size. */
typedef char rs2_minz_control_wire_size_check[ ( sizeof( rs2_minz_control ) == 38 ) ? 1 : -1 ];

/** {min, max, step, def} bounds for rs2_minz_control, as returned by
* rs2_get_composite_option_range(RS2_COMPOSITE_OPTION_HKR_MINZ_CONTROL). `version` here versions
* this wrapper shape itself, distinct from rs2_minz_control::version inside each bound. */
typedef struct rs2_minz_control_range
{
    unsigned int version;
    rs2_minz_control min;
    rs2_minz_control max;
    rs2_minz_control step;
    rs2_minz_control def;
} rs2_minz_control_range;

/* Future FW revisions can grow into the reserved slot and bump param_count without changing
 * sizeof(rs2_minz_control) - callers keep working unmodified. A genuine breaking change (size
 * change) would need a version-tagged union dispatched on the wire's version byte; not needed
 * today. */

#ifdef __cplusplus
}
#endif
#endif  // LIBREALSENSE_RS2_HKR_MINZ_CONTROL_H
