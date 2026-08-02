/* License: Apache 2.0. See LICENSE file in root directory.
   Copyright(c) 2026 RealSense, Inc. All Rights Reserved. */

/** \file rs_hkr_temporal_filter_dpp.h
* \brief
* PROTOTYPE / DEMO API - not a finalized production interface.
*
* Typed payload struct for the HKR (D555-class) Depth Post-Processing "Temporal Filter"
* control - the on-device parameter set for RS2_COMPOSITE_OPTION_HKR_TEMPORAL_FILTER_DPP (see
* rs_composite_option.h). This header defines ONLY the typed struct; getting/setting it goes
* through the generic rs2_get_composite_option/rs2_set_composite_option entry points, e.g.:
*
*   rs2_temporal_filter_dpp_config cfg = { 1, 0.4f, 20, 3 };
*   rs2_set_composite_option(sensor, RS2_COMPOSITE_OPTION_HKR_TEMPORAL_FILTER_DPP, &cfg, sizeof(cfg), &e);
*
* This control packs 4 logical parameters (enabled, smooth_alpha, smooth_delta,
* persistency_index) into ONE UVC Extension Unit control. Per the HW/FW spec, all 4 fields
* MUST land on the device in a single indivisible UVC transaction: sending them one field at a
* time risks the on-device filter running with a partial/mixed parameter set. The generic
* rs2_get/set_composite_option calls each perform EXACTLY ONE get_xu/set_xu round trip.
*/


#ifndef LIBREALSENSE_RS2_HKR_TEMPORAL_FILTER_DPP_H
#define LIBREALSENSE_RS2_HKR_TEMPORAL_FILTER_DPP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rs_types.h"

/**
* Structured, all-fields-at-once configuration for the HKR Temporal Filter Depth
* Post-Processing (DPP) control. Mirrors the on-device parameter set 1:1. This is the payload
* struct for RS2_COMPOSITE_OPTION_HKR_TEMPORAL_FILTER_DPP (see rs_composite_option.h) - get/set
* it via the generic rs2_get_composite_option/rs2_set_composite_option calls.
*/
typedef struct rs2_temporal_filter_dpp_config
{
    int   enabled;             /**< 0 = Off, 1 = On */
    float smooth_alpha;        /**< Exponential moving average alpha, range [0,1], default 0.4, step 0.01 */
    int   smooth_delta;        /**< Step-size boundary threshold that preserves edges, range [1,100], default 20, step 1 */
    int   persistency_index;   /**< Hole-filling / persistency policy, range [0,8], default 3, step 1 */
} rs2_temporal_filter_dpp_config;

#ifdef __cplusplus
}
#endif
#endif  // LIBREALSENSE_RS2_HKR_TEMPORAL_FILTER_DPP_H
