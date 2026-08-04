/* License: Apache 2.0. See LICENSE file in root directory.
   Copyright(c) 2026 RealSense, Inc. All Rights Reserved. */

/** \file rs_composite_option.h
* \brief
* PROTOTYPE / DEMO API - not a finalized production interface.
*
* Generic "composite option" entry points: a small family of functions shared by every
* composite (multi-field, atomically-exchanged) XU control, keyed by the REAL rs2_option enum
* (see e.g. RS2_OPTION_HKR_TEMPORAL_FILTER_DPP in rs_option.h) - there is no separate id type
* and no bespoke named function per feature. This mirrors Orbbec OrbbecSDK's
* setStructuredData(OBPropertyID, const void*, uint32_t) / getStructuredData(...) pattern, but
* reuses rs2_option instead of introducing a parallel enum, so any rs2_options* handle - a
* sensor OR an embedded_filter, both of which wrap rs2_options* (see rs2::options) - that
* supports a given composite option can be driven through the same shape of calls used for
* ordinary scalar options (rs2_get_option/rs2_set_option/rs2_get_option_range).
*
* The SDK does NOT ship a public typed struct per feature's wire layout as a rule; ONE
* exception exists for the prototype HKR Temporal Filter DPP control, whose layout is public
* as a documentation/cast-target convenience (see rs_hkr_temporal_filter_dpp.h) - but the entry
* points below never change shape as new composite options are added: adding one means adding
* an enumerator to rs2_option and registering it internally, not adding new public functions.
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
#include "rs_option.h"

/**
* rs2_set_composite_option - generic composite-option setter.
* Writes size bytes from data to the device in ONE atomic UVC control transaction (one set_xu
* call). The caller is responsible for knowing the documented wire layout for option (e.g. see
* rs2_temporal_filter_dpp_config in rs_hkr_temporal_filter_dpp.h for
* RS2_OPTION_HKR_TEMPORAL_FILTER_DPP) and passing a pointer to a matching struct + sizeof(...)
* as data/size. No ownership transfer - data remains caller-owned, exactly like rs2_set_option.
* \param[in]  options  Options container (sensor or embedded_filter) that exposes this composite option
* \param[in]  option   Which (composite) option to write
* \param[in]  data     Pointer to the caller's struct matching the option's documented wire layout
* \param[in]  size     sizeof(...) of the caller's struct
* \param[out] error    If non-null, receives any error that occurs during this call, otherwise, errors are ignored
*/
void rs2_set_composite_option(const rs2_options* options, rs2_option option, const void* data, unsigned int size, rs2_error** error);

/**
* rs2_get_composite_option - generic composite-option getter.
* Reads the current value from the device in ONE atomic UVC control transaction (one get_xu
* call). The caller has no generic way to know the wire size of an arbitrary option in advance
* (only the SDK-side control for that specific option knows it), so the SDK heap-allocates and
* returns the result as an rs2_raw_data_buffer - read its bytes with
* rs2_get_raw_data_size/rs2_get_raw_data and free it with rs2_delete_raw_data when done (mirrors
* rs2_get_safety_preset / rs2_send_and_receive_raw_data).
* \param[in]   options  Options container (sensor or embedded_filter) that exposes this composite option
* \param[in]   option   Which (composite) option to read
* \param[out]  error    If non-null, receives any error that occurs during this call, otherwise, errors are ignored
* \return                SDK-allocated buffer holding the option's current raw payload bytes; cast
*                         into a struct matching the option's documented wire layout. Free with
*                         rs2_delete_raw_data.
*/
const rs2_raw_data_buffer* rs2_get_composite_option(const rs2_options* options, rs2_option option, rs2_error** error);

/**
* rs2_get_composite_option_range - generic composite-option range getter.
* Reads the option's supported {min, max, step, def} - one instance of the option's struct per
* bound (see e.g. rs2_temporal_filter_dpp_range in rs_hkr_temporal_filter_dpp.h) - packed
* together into a single SDK-allocated rs2_raw_data_buffer, mirroring rs2_get_composite_option's
* ownership model. This is the composite-option analogue of rs2_get_option_range.
* \param[in]   options  Options container (sensor or embedded_filter) that exposes this composite option
* \param[in]   option   Which (composite) option's range to read
* \param[out]  error    If non-null, receives any error that occurs during this call, otherwise, errors are ignored
* \return                SDK-allocated buffer holding the option's range payload bytes; cast into
*                         the range struct documented for option. Free with rs2_delete_raw_data.
*/
const rs2_raw_data_buffer* rs2_get_composite_option_range(const rs2_options* options, rs2_option option, rs2_error** error);

#ifdef __cplusplus
}
#endif
#endif  // LIBREALSENSE_RS2_COMPOSITE_OPTION_H
