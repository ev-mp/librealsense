# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

import struct
import pytest
import pyrealsense2 as rs
import logging
log = logging.getLogger(__name__)

pytestmark = [
    pytest.mark.device_each("D555"),
    pytest.mark.device_each("D585*"),
]

# HKR MinZ Control's documented wire layout: a dppc_header (version, flags, ctl_id, param_count,
# param_type) shared by the whole HKR DPP control family, then MinZ's 7 logical fields (post
# design-review layout - see rs_hkr_minz_control.h), then 1 reserved (always 0) slot. Same layout
# used by wrappers/python/examples/embedded_filters.py - there is no generic "any composite
# option" cast, so a new composite option needs its own layout (and its own test) added, same as
# everywhere else composite options are exercised in this repo.
_MINZ_FMT = '<BBHBBiiiiiiii'
_MINZ_FIELDS = ['version', 'flags', 'ctl_id', 'param_count', 'param_type',
                'enable', 'filter_type', 'downscale_ratio', 'shift_mode', 'shift_pixels',
                'threshold_mode', 'threshold_mm', 'reserved0']
_MINZ_ITEM_SIZE = struct.calcsize(_MINZ_FMT)


def _unpack_minz(raw):
    return dict(zip(_MINZ_FIELDS, struct.unpack(_MINZ_FMT, raw)))


def _pack_minz(fields):
    return struct.pack(_MINZ_FMT, *(fields[name] for name in _MINZ_FIELDS))


def _find_minz_filter(sensor):
    """Return the embedded filter exposing HKR MinZ Control on this sensor, or None - the
    composite option lives on one of the sensor's embedded filters, its own independent options
    registry, not on the sensor itself (see rs2::embedded_filter)."""
    for embedded_filter in sensor.query_embedded_filters():
        if rs.composite_option_id.hkr_minz_control in embedded_filter.get_supported_composite_options():
            return embedded_filter
    return None


def test_minz_control_basic_parameter_changes(test_device):
    """Check whether HKR MinZ Control is supported; if so, bounce its writable fields to new
    in-range values (verifying each readback) and restore the original value at the end."""
    dev, ctx = test_device
    depth_sensor = dev.first_depth_sensor()
    embedded_filter = _find_minz_filter(depth_sensor)
    if embedded_filter is None:
        pytest.skip("HKR MinZ Control composite option not supported on this device")

    option_id = rs.composite_option_id.hkr_minz_control
    try:
        original_raw = embedded_filter.get_composite_option(option_id)
    except RuntimeError as e:
        # Registered but not actually functional on this device/FW is a real, expected outcome -
        # get_supported_composite_options() only reflects static registration, never a live
        # capability check.
        pytest.skip(f"HKR MinZ Control registered but not functional on this device/FW: {e}")

    original = _unpack_minz(original_raw)

    range_raw = embedded_filter.get_composite_option_range(option_id)
    # rs2_minz_control_range: a 4-byte wrapper version, then 4 full copies of the struct - min,
    # max, step, def, in that order, each _MINZ_ITEM_SIZE bytes.
    min_fields = _unpack_minz(range_raw[4:4 + _MINZ_ITEM_SIZE])
    max_fields = _unpack_minz(range_raw[4 + _MINZ_ITEM_SIZE:4 + 2 * _MINZ_ITEM_SIZE])

    changed_any = False
    try:
        # downscale_ratio and shift_pixels: real, independent min/max bounds with no side-effects
        # on `enable`/`filter_type`/`shift_mode`/`threshold_mode` - safe to bounce within range
        # one at a time. (Their in-range values may be inert on the wire depending on the
        # current `filter_type`/`shift_mode` selection - that's fine, this test only checks that
        # a written value round-trips through GET, not that it visibly affects merged depth.)
        for field in ('downscale_ratio', 'shift_pixels'):
            lo, hi = min_fields[field], max_fields[field]
            if lo >= hi:
                log.info(f"{field}: no room to change, range is [{lo}, {hi}]")
                continue
            new_value = lo if original[field] != lo else hi

            cfg = dict(original)
            cfg[field] = new_value
            embedded_filter.set_composite_option(option_id, _pack_minz(cfg))
            changed_any = True

            readback = _unpack_minz(embedded_filter.get_composite_option(option_id))
            assert readback[field] == new_value, f"{field}: expected {new_value}, got {readback[field]}"

            # Restore this field immediately, before moving on to the next one, so each change is
            # independently verified and reverted rather than compounding on top of the last.
            cfg[field] = original[field]
            embedded_filter.set_composite_option(option_id, _pack_minz(cfg))

        if not changed_any:
            pytest.skip("No writable MinZ field had room to change on this device/FW")
    finally:
        # Always restore the very first raw payload read from the device, regardless of what
        # happened above - the strongest guarantee that this test leaves no lasting side effects.
        embedded_filter.set_composite_option(option_id, original_raw)
        readback_raw = embedded_filter.get_composite_option(option_id)
        assert readback_raw == original_raw, "failed to restore original HKR MinZ Control value"
