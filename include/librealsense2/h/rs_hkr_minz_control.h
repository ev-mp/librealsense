/* License: Apache 2.0. See LICENSE file in root directory.
   Copyright(c) 2026 RealSense, Inc. All Rights Reserved. */

/** \file rs_hkr_minz_control.h
* \brief
* PROTOTYPE / DEMO API - not a finalized production interface.
*
* Public documentation/cast-target structs for RS2_COMPOSITE_OPTION_HKR_MINZ_CONTROL (see
* rs_option.h), a composite (multi-field, atomically-exchanged) XU control accessed through the
* generic rs2_set_composite_option/rs2_get_composite_option/rs2_get_composite_option_range entry
* points (see rs_composite_option.h) rather than through bespoke named functions.
*
* Layout matches section 4.2 ("Wire Layout") of "HKR D5X5 MinZ Control - Host interface API
* Proposal" (Confluence pageId=74061948) byte-for-byte: a 6-byte dppc_header (version, flags,
* ctl_id) shared by the whole HKR DPP control family, followed by param_count/param_type and 8
* fixed int32 param slots (of which MinZ uses 5; the rest are reserved and MUST be zero on SET).
* Wire is little-endian; #pragma pack(1) is required because the header's odd 6-byte size would
* otherwise leave the compiler inserting 2 bytes of padding before params[0].
*
* Header field values (version=0x01, flags=0x01, ctl_id=0x0008) were taken from the former
* close_range_xu_option implementation (since removed - see below) and independently confirmed
* against a real-hardware FW validation tool - NOT from the Confluence page's own wire-layout
* table, which states ctl_id=0x0004 and is stale on that point.
*
* This control replaces what used to be exposed as the scalar, enable-only "Improved Close Range
* Depth" option (RS2_OPTION_EMBEDDED_FILTER_ENABLED via close_range_xu_option) - both addressed
* the SAME physical XU control (unit 3, selector 0x14). PROTOTYPE/DEMO PR: that scalar option has
* been removed and consolidated onto this composite option, which exposes full read/write access
* to all 5 fields instead of just enable.
*
* These structs exist purely so callers do not need to hand-roll and keep in sync their own copy
* of the documented wire layout; the SDK does not expose any function named after them - cast the
* raw bytes returned by rs2_get_composite_option/rs2_get_composite_option_range to/from these
* types, and pass a pointer to rs2_minz_control + sizeof(...) to rs2_set_composite_option.
*/

#ifndef LIBREALSENSE_RS2_HKR_MINZ_CONTROL_H
#define LIBREALSENSE_RS2_HKR_MINZ_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#pragma pack(push, 1)

/** HKR/D5X5 MinZ control payload - the ENTIRE 38-byte blob exchanged by
* RS2_COMPOSITE_OPTION_HKR_MINZ_CONTROL over the shared HKR DPP XU control family, including the
* wire header (version/flags/ctl_id) every control in that family carries - not just MinZ's own
* 5 logical fields. */
typedef struct rs2_minz_control
{
    /* --- dppc_header: shared by the whole HKR DPP control family, not MinZ-specific --- */
    uint8_t  version;             /**< Wire struct version, per dppc_header. Currently 0x01 */
    uint8_t  flags;                /**< Bitwise control-status mask (active/read-only), per dppc_header */
    uint16_t ctl_id;               /**< dpp_ctrl_list entry identifying this control. dpp_minz_filter = 0x0008 -
                                    * confirmed against the FW validation tool and the existing
                                    * close_range_xu_option implementation; an earlier doc pass on this
                                    * file incorrectly said 0x0004. */

    /* --- dppc_ctl parameter block header --- */
    uint8_t  param_count;          /**< Populated param slots below. 5 for MinZ */
    uint8_t  param_type;           /**< Per-param int/float bitmap. 0x00 (all-integer) for MinZ */

    /* --- MinZ's 5 logical fields, in wire order (dppc_ctl.params[0..4]) --- */
    int32_t  enable;                /**< 0 = Off, 1 = On. Default 0 */
    int32_t  downscale_ratio;       /**< MinZ secondary path resolution scale: 1, 2, or 4. Default 2. Pre-stream only */
    int32_t  disparity_shift;       /**< SCP disparity shift for the MinZ depth path. Range [0,512]. Default 0 */
    int32_t  threshold;             /**< mm. SET: desired manual threshold; GET: effective threshold. Range [0,65535] */
    int32_t  threshold_mode;        /**< 0 = Auto (firmware-computed), 1 = Manual (host-supplied). Default 0 */

    /* --- dppc_ctl.params[5..7]: unused by MinZ --- */
    int32_t  reserved[3];           /**< MUST be zero on SET */
} rs2_minz_control;

#pragma pack(pop)

/* Byte-exact wire-size guard: fails to compile if padding/field changes push this off 38 bytes. */
typedef char rs2_minz_control_wire_size_check[ ( sizeof( rs2_minz_control ) == 38 ) ? 1 : -1 ];

/** Supported {min, max, step, def} bounds for rs2_minz_control, as returned by
* rs2_get_composite_option_range(RS2_COMPOSITE_OPTION_HKR_MINZ_CONTROL). Each bound is a full,
* independent copy of rs2_minz_control - including its own header fields, which are read-only
* and constant across min/max/step/def (e.g. version/ctl_id/param_count/param_type all report the
* same value in every one of the four copies; a generic accessor may use that as a cheap
* integrity check against the compile-time expected values).
*
* Note this range-wrapper's own `version` is a distinct concept from rs2_minz_control::version
* above (the wire header byte FW actually sends) - this one versions the {min,max,step,def}
* wrapper shape itself, matching the convention used by rs2_temporal_filter_dpp_range. The two
* are necessarily different types (unsigned int vs uint8_t) and are expected to evolve
* independently. */
typedef struct rs2_minz_control_range
{
    unsigned int version;                /**< Schema version of this range struct; starts at 1 */
    rs2_minz_control min;
    rs2_minz_control max;
    rs2_minz_control step;
    rs2_minz_control def;
} rs2_minz_control_range;

/* ---------------------------------------------------------------------------------------------
 * A NOTE ON FUTURE VERSIONING (documentation only - nothing below this point is compiled, and no
 * rs2_minz_control_v2 exists today; this is a provision, not a feature).
 * ---------------------------------------------------------------------------------------------
 * rs2_minz_control::version exists so a future firmware revision can change this control's shape
 * without breaking older hosts. The wire envelope already reserves params[5..7] for exactly that
 * - the expected growth path is a new version populating a previously-reserved slot and bumping
 * param_count. sizeof(rs2_minz_control) does not change in that case, and nothing below is
 * needed: get_composite_option_as<rs2_minz_control>() keeps working unmodified, and it is
 * param_count (not the C++ type) that tells the caller how many fields are meaningful.
 *
 * If a future version ever outgrows the envelope's reserved room - a genuine breaking wire
 * change - sizeof() DOES change, and no C++ trick lets `auto` deduce "whichever struct matches
 * whatever version comes back at runtime": `auto` only ever deduces the one type a function is
 * declared to return. The pattern to reach for at that point is: peek the version field (always
 * fixed at byte 0 - this struct's one permanent invariant) BEFORE committing to a struct type,
 * then dispatch into a small tagged union that IS the one fixed type the caller's `auto` binds
 * to:
 *
 *     typedef struct rs2_minz_control_v2 { ... } rs2_minz_control_v2;   // whatever v2 becomes
 *
 *     struct rs2_minz_control_any
 *     {
 *         uint8_t version;                // which member of `as` is populated
 *         union { rs2_minz_control v1; rs2_minz_control_v2 v2; } as;
 *     };
 *
 *     rs2_minz_control_any get_minz_control_versioned( const rs2::options & sensor, rs2_composite_option_id id )
 *     {
 *         std::vector<uint8_t> raw = sensor.get_composite_option( id );   // no cast yet
 *         rs2_minz_control_any result{};
 *         result.version = raw[0];                                       // fixed offset, by contract
 *         switch( result.version )
 *         {
 *         case 1:  memcpy( &result.as.v1, raw.data(), sizeof( rs2_minz_control ) );    break;
 *         case 2:  memcpy( &result.as.v2, raw.data(), sizeof( rs2_minz_control_v2 ) ); break;
 *         default: throw std::runtime_error( "unrecognized MinZ control wire version" );
 *         }
 *         return result;
 *     }
 *
 *     auto cfg = get_minz_control_versioned( sensor, id );   // auto -> rs2_minz_control_any, fixed
 *     if( cfg.version == 1 ) { use cfg.as.v1 ... }
 *
 * This tagged-union shape works unmodified in C99 too - no templates involved, just a struct
 * with a tag and a union. It is a SEPARATE versioning axis from rs2_minz_control_range::version
 * above, which versions the {min,max,step,def} WRAPPER shape, not this struct's own payload
 * shape - the two can change independently.
 * --------------------------------------------------------------------------------------------- */

#ifdef __cplusplus
}
#endif
#endif  // LIBREALSENSE_RS2_HKR_MINZ_CONTROL_H
