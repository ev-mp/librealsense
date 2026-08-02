/* License: Apache 2.0. See LICENSE file in root directory.
   Copyright(c) 2026 RealSense, Inc. All Rights Reserved. */

/** \file rs_composite_option.h
* \brief
* PROTOTYPE / DEMO API - not a finalized production interface.
*
* Generic "composite option" entry point: ONE pair of functions (rs2_set_composite_option /
* rs2_get_composite_option) shared by every composite (multi-field, atomically-exchanged) XU
* control, keyed by an rs2_composite_option_id. This mirrors Orbbec OrbbecSDK's
* setStructuredData(OBPropertyID, const void*, uint32_t) / getStructuredData(...) pattern:
* there is no bespoke named function per feature - adding a new composite option means adding
* one enumerator here (plus its typed payload struct, e.g. rs2_temporal_filter_dpp_config in
* rs_hkr_temporal_filter_dpp.h) and registering it internally, NOT adding new public functions.
*
* Each call performs EXACTLY ONE UVC control transaction (one set_xu/get_xu round trip): all
* fields of the option's payload struct travel together, atomically - never as separate
* per-field writes/reads.
*/

#ifndef LIBREALSENSE_RS2_COMPOSITE_OPTION_H
#define LIBREALSENSE_RS2_COMPOSITE_OPTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rs_types.h"

/**
* Identifies which composite option (and therefore which typed payload struct) a
* rs2_set_composite_option/rs2_get_composite_option call refers to. This enum is the ONLY
* per-feature footprint of the generic mechanism - the entry point functions themselves never
* change.
*/
typedef enum rs2_composite_option_id
{
    RS2_COMPOSITE_OPTION_HKR_TEMPORAL_FILTER_DPP, /**< Prototype: HKR/D555 Depth Post-Processing Temporal Filter. Payload: rs2_temporal_filter_dpp_config (see rs_hkr_temporal_filter_dpp.h) */
    RS2_COMPOSITE_OPTION_COUNT
} rs2_composite_option_id;
const char* rs2_composite_option_id_to_string(rs2_composite_option_id id);

/**
* rs2_set_composite_option - generic composite-option setter.
* Writes data_size bytes from data to the device in ONE atomic UVC control transaction (one
* set_xu call). The caller is responsible for knowing which typed struct corresponds to
* option_id and passing its address + sizeof(...) as data/data_size.
* \param[in]  sensor       Sensor that exposes the requested composite option
* \param[in]  option_id    Which composite option to write
* \param[in]  data         Pointer to the caller's typed payload struct
* \param[in]  data_size    sizeof(...) of the caller's typed payload struct
* \param[out] error        If non-null, receives any error that occurs during this call, otherwise, errors are ignored
*/
void rs2_set_composite_option(const rs2_sensor* sensor, rs2_composite_option_id option_id, const void* data, unsigned int data_size, rs2_error** error);

/**
* rs2_get_composite_option - generic composite-option getter.
* Reads the current value from the device in ONE atomic UVC control transaction (one get_xu
* call). The caller has no generic way to know the wire size of an arbitrary option_id in
* advance (only the SDK-side control for that specific id knows it), so the SDK heap-allocates
* and returns the result as an rs2_raw_data_buffer - read its bytes with rs2_get_raw_data_size/
* rs2_get_raw_data and free it with rs2_delete_raw_data when done (mirrors rs2_get_safety_preset).
* \param[in]   sensor       Sensor that exposes the requested composite option
* \param[in]   option_id    Which composite option to read
* \param[out]  error        If non-null, receives any error that occurs during this call, otherwise, errors are ignored
* \return                   SDK-allocated buffer holding the option's raw payload bytes; the
*                            caller casts them to the typed struct that corresponds to option_id
*                            (e.g. rs2_temporal_filter_dpp_config). Free with rs2_delete_raw_data.
*/
const rs2_raw_data_buffer* rs2_get_composite_option(const rs2_sensor* sensor, rs2_composite_option_id option_id, rs2_error** error);

#ifdef __cplusplus
}
#endif
#endif  // LIBREALSENSE_RS2_COMPOSITE_OPTION_H
