/* License: Apache 2.0. See LICENSE file in root directory.
   Copyright(c) 2026 RealSense, Inc. All Rights Reserved. */

/** \file rs_hkr_temporal_filter_dpp.h
* \brief
* Public documentation/cast-target structs for RS2_COMPOSITE_OPTION_HKR_TEMPORAL_FILTER_DPP (see
* rs_option.h), a composite (multi-field, atomically-exchanged) XU control accessed through the
* generic rs2_set_composite_option/rs2_get_composite_option/rs2_get_composite_option_range entry
* points (see rs_composite_option.h) rather than through bespoke named functions.
*
* These structs exist purely so callers do not need to hand-roll and keep in sync their own copy
* of the documented wire layout; the SDK does not expose any function named after them - cast the
* raw bytes returned by rs2_get_composite_option/rs2_get_composite_option_range to/from these
* types, and pass a pointer to rs2_temporal_filter_dpp_config + sizeof(...) to
* rs2_set_composite_option.
*/

#ifndef LIBREALSENSE_RS2_HKR_TEMPORAL_FILTER_DPP_H
#define LIBREALSENSE_RS2_HKR_TEMPORAL_FILTER_DPP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** HKR/D555 Depth Post-Processing "Temporal Filter" configuration - the payload exchanged by
* RS2_COMPOSITE_OPTION_HKR_TEMPORAL_FILTER_DPP. Tightly packed, no padding required: every field is
* naturally 4-byte-aligned. 16 bytes total, no dpp_header (unlike rs2_improved_close_range_control) - current
* firmware doesn't speak the DPP family's fixed-8-param envelope for this control yet; expected to
* converge onto dpp_header (see rs_dpp_header.h) once that's negotiated with the FW team. */
typedef struct rs2_temporal_filter_dpp_config
{
    int32_t enabled;             /**< 0 = Off, 1 = On. Default 0 */
    float   smooth_alpha;        /**< Range [0,1], step 0.01. Default 0.4 */
    int32_t smooth_delta;        /**< Range [1,100], step 1. Default 20 */
    int32_t persistency_index;   /**< Range [0,8], step 1. Default 3 */
} rs2_temporal_filter_dpp_config;

/** Supported {min, max, step, def} bounds for rs2_temporal_filter_dpp_config, as returned by
* rs2_get_composite_option_range(RS2_COMPOSITE_OPTION_HKR_TEMPORAL_FILTER_DPP). Read-only - no
* version field of its own. */
typedef struct rs2_temporal_filter_dpp_range
{
    rs2_temporal_filter_dpp_config min;
    rs2_temporal_filter_dpp_config max;
    rs2_temporal_filter_dpp_config step;
    rs2_temporal_filter_dpp_config def;
} rs2_temporal_filter_dpp_range;

#ifdef __cplusplus
}
#endif
#endif  // LIBREALSENSE_RS2_HKR_TEMPORAL_FILTER_DPP_H
