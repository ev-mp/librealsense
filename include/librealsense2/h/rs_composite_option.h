/* License: Apache 2.0. See LICENSE file in root directory.
   Copyright(c) 2026 RealSense, Inc. All Rights Reserved. */

/** \file rs_composite_option.h
* \brief
* Generic entry points shared by every composite (multi-field, atomically-exchanged) XU control.
* Composite options are a separate identity space from rs2_option - keyed by
* rs2_composite_option_id, its own enumeration (rs2_get_composite_options_list), never mixed with
* scalar options. Any rs2_options* handle (sensor or embedded_filter) that supports a given
* composite option can be driven through the calls below. Each call is exactly one UVC
* transaction: all fields of the option's payload travel together, atomically.
*/

#ifndef LIBREALSENSE_RS2_COMPOSITE_OPTION_H
#define LIBREALSENSE_RS2_COMPOSITE_OPTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rs_types.h"

/** Identifies a composite (multi-field, atomically-exchanged) control. Completely independent
* of rs2_option (see rs_option.h) - a different enum, with its own namespace and its own
* registration/enumeration path (rs2_get_composite_options_list), never mixed with scalar
* options. */
typedef enum rs2_composite_option_id
{
    /** HKR (D555-class) Depth Post-Processing "Temporal Filter" - see rs_hkr_temporal_filter_dpp.h. */
    RS2_COMPOSITE_OPTION_HKR_TEMPORAL_FILTER_DPP,
    /** HKR/D5X5 Improved Close Range control - see rs_hkr_improved_close_range_control.h. */
    RS2_COMPOSITE_OPTION_HKR_IMPROVED_CLOSE_RANGE_CONTROL,
    RS2_COMPOSITE_OPTION_COUNT /**< Number of enumeration values. Not a valid input: intended to be used in for-loops. */
} rs2_composite_option_id;

/**
* Returns the composite option id's name, or "UNKNOWN" otherwise - the composite-option analogue
* of rs2_option_to_string. Unlike rs2_option, composite option ids have no from_string() reverse
* lookup - there is no equivalent registry-based custom-name mechanism for them (see
* rs2_option_from_string() in rs_option.h).
* \param[in] id    the composite option identifier
*/
const char* rs2_composite_option_id_to_string(rs2_composite_option_id id);

/**
* rs2_set_composite_option - generic composite-option setter.
* Writes size bytes from data to the device in ONE atomic UVC control transaction (one set_xu
* call). The caller is responsible for knowing the documented wire layout for option (e.g. see
* rs2_temporal_filter_dpp_config in rs_hkr_temporal_filter_dpp.h for
* RS2_COMPOSITE_OPTION_HKR_TEMPORAL_FILTER_DPP) and passing a pointer to a matching struct +
* sizeof(...) as data/size. No ownership transfer - data remains caller-owned, exactly like
* rs2_set_option.
* \param[in]  options  Options container (sensor or embedded_filter) that exposes this composite option
* \param[in]  option   Which composite option to write
* \param[in]  data     Pointer to the caller's struct matching the option's documented wire layout
* \param[in]  size     sizeof(...) of the caller's struct
* \param[out] error    If non-null, receives any error that occurs during this call, otherwise, errors are ignored
*/
void rs2_set_composite_option(const rs2_options* options, rs2_composite_option_id option, const void* data, unsigned int size, rs2_error** error);

/**
* rs2_get_composite_option - generic composite-option getter.
* Reads the current value from the device in ONE atomic UVC control transaction (one get_xu
* call). The caller has no generic way to know the wire size of an arbitrary composite option in
* advance (only the SDK-side control for that specific option knows it), so the SDK
* heap-allocates and returns the result as an rs2_raw_data_buffer - read its bytes with
* rs2_get_raw_data_size/rs2_get_raw_data and free it with rs2_delete_raw_data when done (mirrors
* rs2_get_safety_preset / rs2_send_and_receive_raw_data).
* \param[in]   options  Options container (sensor or embedded_filter) that exposes this composite option
* \param[in]   option   Which composite option to read
* \param[out]  error    If non-null, receives any error that occurs during this call, otherwise, errors are ignored
* \return                SDK-allocated buffer holding the option's current raw payload bytes; cast
*                         into a struct matching the option's documented wire layout. Free with
*                         rs2_delete_raw_data.
*/
const rs2_raw_data_buffer* rs2_get_composite_option(const rs2_options* options, rs2_composite_option_id option, rs2_error** error);

/**
* rs2_get_composite_option_range - generic composite-option range getter.
* Reads the option's supported {min, max, step, def} - one instance of the option's struct per
* bound (see e.g. rs2_temporal_filter_dpp_range in rs_hkr_temporal_filter_dpp.h) - packed
* together into a single SDK-allocated rs2_raw_data_buffer, mirroring rs2_get_composite_option's
* ownership model. This is the composite-option analogue of rs2_get_option_range.
* \param[in]   options  Options container (sensor or embedded_filter) that exposes this composite option
* \param[in]   option   Which composite option's range to read
* \param[out]  error    If non-null, receives any error that occurs during this call, otherwise, errors are ignored
* \return                SDK-allocated buffer holding the option's range payload bytes; cast into
*                         the range struct documented for option. Free with rs2_delete_raw_data.
*/
const rs2_raw_data_buffer* rs2_get_composite_option_range(const rs2_options* options, rs2_composite_option_id option, rs2_error** error);

/**
* rs2_get_composite_options_list - the composite-option analogue of rs2_get_options_list: the
* full set of composite ids this options container (sensor or embedded_filter) supports. Never
* contains a scalar rs2_option - composite and scalar options are enumerated completely
* separately, so generic code walking one list never has to worry about hitting an entry from
* the other.
* \param[in]  options  Options container (sensor or embedded_filter)
* \param[out] error    If non-null, receives any error that occurs during this call, otherwise, errors are ignored
* \return              List of supported composite option ids. Free with rs2_delete_composite_options_list.
*/
rs2_composite_options_list* rs2_get_composite_options_list(const rs2_options* options, rs2_error** error);

/**
* Return the number of composite option ids in a list returned by rs2_get_composite_options_list.
* \param[in]  list   the list of composite option ids
* \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
* \return the number of composite option ids in the list
*/
int rs2_get_composite_options_list_size(const rs2_composite_options_list* list, rs2_error** error);

/**
* Retrieve the i-th composite option id in a list returned by rs2_get_composite_options_list.
* \param[in]  list   the list of composite option ids
* \param[in]  i      the index of the composite option id to retrieve
* \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
* \return the composite option id at index i
*/
rs2_composite_option_id rs2_get_composite_option_from_list(const rs2_composite_options_list* list, int i, rs2_error** error);

/**
* Deletes a composite options list returned by rs2_get_composite_options_list.
* \param[in] list  the list to delete
*/
void rs2_delete_composite_options_list(rs2_composite_options_list* list);

/**
* check if a particular composite option is supported (and currently enabled) by this options
* container - the composite-option analogue of rs2_supports_option.
* \param[in]  options  Options container (sensor or embedded_filter)
* \param[in]  option   composite option id to check
* \param[out] error    If non-null, receives any error that occurs during this call, otherwise, errors are ignored
* \return true if the composite option is supported
*/
int rs2_supports_composite_option(const rs2_options* options, rs2_composite_option_id option, rs2_error** error);

/**
* check if a composite option is read-only - the composite-option analogue of rs2_is_option_read_only.
* \param[in]  options  Options container (sensor or embedded_filter) that exposes this composite option
* \param[in]  option   composite option id to check
* \param[out] error    If non-null, receives any error that occurs during this call, otherwise, errors are ignored
* \return true if the composite option is read-only
*/
int rs2_is_composite_option_read_only(const rs2_options* options, rs2_composite_option_id option, rs2_error** error);

/**
* get a composite option's human-readable description - the composite-option analogue of
* rs2_get_option_description.
* \param[in]  options  Options container (sensor or embedded_filter) that exposes this composite option
* \param[in]  option   composite option id to describe
* \param[out] error    If non-null, receives any error that occurs during this call, otherwise, errors are ignored
* \return human-readable composite option description
*/
const char* rs2_get_composite_option_description(const rs2_options* options, rs2_composite_option_id option, rs2_error** error);

#ifdef __cplusplus
}
#endif
#endif  // LIBREALSENSE_RS2_COMPOSITE_OPTION_H
