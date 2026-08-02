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
* there is no bespoke named function per feature, and - importantly - the SDK does NOT ship a
* public typed struct per feature either. Adding a new composite option means adding one
* enumerator here (with its wire layout DOCUMENTED as a comment, not as a type) and
* registering it internally, NOT adding new public functions or public structs. Callers define
* their own local struct matching the documented layout for whichever option_id they use.
*
* Each call performs EXACTLY ONE UVC control transaction (one set_xu/get_xu round trip): all
* fields of the option's payload travel together, atomically - never as separate per-field
* writes/reads.
*/

#ifndef LIBREALSENSE_RS2_COMPOSITE_OPTION_H
#define LIBREALSENSE_RS2_COMPOSITE_OPTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rs_types.h"

/**
* Identifies which composite option a rs2_set_composite_option/rs2_get_composite_option call
* refers to. This enum (and the documented byte layout on each enumerator below) is the ONLY
* per-feature footprint of the generic mechanism - the entry point functions themselves never
* change, and the SDK ships no typed struct for any of these. The caller/app is responsible
* for defining its own local struct matching the documented layout and casting the raw bytes
* to/from it.
*/
typedef enum rs2_composite_option_id
{
    /** Prototype: HKR/D555 Depth Post-Processing Temporal Filter.
     * Documented wire layout (tightly packed, no padding - all fields are naturally
     * 4-byte-aligned so no explicit packing pragma is required in the caller's struct),
     * 16 bytes total, in this exact field order:
     *   int32_t enabled;             0 = Off, 1 = On
     *   float   smooth_alpha;        range [0,1], default 0.4, step 0.01
     *   int32_t smooth_delta;        range [1,100], default 20, step 1
     *   int32_t persistency_index;   range [0,8], default 3, step 1
     * The SDK does not define a type for this - define your own local struct matching this
     * layout and pass &your_struct / sizeof(your_struct) to rs2_set/get_composite_option. */
    RS2_COMPOSITE_OPTION_HKR_TEMPORAL_FILTER_DPP,
    RS2_COMPOSITE_OPTION_COUNT
} rs2_composite_option_id;
const char* rs2_composite_option_id_to_string(rs2_composite_option_id id);

/**
* rs2_set_composite_option - generic composite-option setter.
* Writes data_size bytes from data to the device in ONE atomic UVC control transaction (one
* set_xu call). The caller is responsible for knowing the documented wire layout for option_id
* (see rs2_composite_option_id above) and passing a pointer to its own matching struct +
* sizeof(...) as data/data_size. The SDK does not ship a typed struct for this.
* \param[in]  sensor       Sensor that exposes the requested composite option
* \param[in]  option_id    Which composite option to write
* \param[in]  data         Pointer to the caller's own struct matching the documented layout
* \param[in]  data_size    sizeof(...) of the caller's struct
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
*                            caller casts them into its own local struct matching the
*                            documented layout for option_id (see rs2_composite_option_id
*                            above). Free with rs2_delete_raw_data.
*/
const rs2_raw_data_buffer* rs2_get_composite_option(const rs2_sensor* sensor, rs2_composite_option_id option_id, rs2_error** error);

#ifdef __cplusplus
}
#endif
#endif  // LIBREALSENSE_RS2_COMPOSITE_OPTION_H
