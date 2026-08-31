# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

import pytest
import pyrealsense2 as rs
import logging
log = logging.getLogger(__name__)

pytestmark = [
    pytest.mark.device_each("D555"),
    pytest.mark.device_each("D585"),
    pytest.mark.device_exclude("D585S"),  # not registered on the safety-certified D585S (see d500-factory.cpp)
]


def _find_improved_close_range_filter(sensor):
    """Return the embedded filter exposing HKR Improved Close Range Control on this sensor, or None - the
    composite option lives on one of the sensor's embedded filters, its own independent options
    registry, not on the sensor itself (see rs2::embedded_filter)."""
    for embedded_filter in sensor.query_embedded_filters():
        if rs.composite_option_id.hkr_improved_close_range_control in embedded_filter.get_supported_composite_options():
            return embedded_filter
    return None


def test_improved_close_range_control_basic_parameter_changes(test_device):
    """Check whether HKR Improved Close Range Control is supported; if so, bounce its writable fields to new
    in-range values (verifying each readback) and restore the original value at the end."""
    dev, ctx = test_device
    depth_sensor = dev.first_depth_sensor()
    embedded_filter = _find_improved_close_range_filter(depth_sensor)
    if embedded_filter is None:
        pytest.skip("HKR Improved Close Range Control composite option not supported on this device")

    option_id = rs.composite_option_id.hkr_improved_close_range_control
    try:
        original_raw = embedded_filter.get_composite_option(option_id)
    except RuntimeError as e:
        # Registered but not actually functional on this device/FW is a real, expected outcome -
        # get_supported_composite_options() only reflects static registration, never a live
        # capability check.
        pytest.skip(f"HKR Improved Close Range Control registered but not functional on this device/FW: {e}")

    # Typed get/set (see wrappers/python/pyrs_options.cpp) - the SDK's own bound struct, no
    # hand-rolled struct.pack/unpack format string to keep in sync with the wire layout. Only the
    # final restore-and-verify below uses the raw bytes, for the strongest possible guarantee
    # (byte-exact, header and reserved fields included).
    original = embedded_filter.get_improved_close_range_control(option_id)
    range = embedded_filter.get_improved_close_range_control_range(option_id)

    changed_any = False
    try:
        # downscale_ratio and disparity_shift: real, independent min/max bounds with no
        # side-effects on `enable`/`threshold_mode` - safe to bounce within range one at a time.
        for field in ('downscale_ratio', 'disparity_shift'):
            lo, hi = getattr(range.min, field), getattr(range.max, field)
            if lo >= hi:
                log.info(f"{field}: no room to change, range is [{lo}, {hi}]")
                continue
            current = getattr(original, field)
            new_value = lo if current != lo else hi

            # Each set starts from a freshly-read struct - the header/other fields must be
            # carried over exactly as the device just reported them, not zero-initialized.
            cfg = embedded_filter.get_improved_close_range_control(option_id)
            setattr(cfg, field, new_value)
            embedded_filter.set_improved_close_range_control(option_id, cfg)
            changed_any = True

            readback = embedded_filter.get_improved_close_range_control(option_id)
            assert getattr(readback, field) == new_value, f"{field}: expected {new_value}, got {getattr(readback, field)}"

            # Restore this field immediately, before moving on to the next one, so each change is
            # independently verified and reverted rather than compounding on top of the last.
            cfg = embedded_filter.get_improved_close_range_control(option_id)
            setattr(cfg, field, current)
            embedded_filter.set_improved_close_range_control(option_id, cfg)

        if not changed_any:
            pytest.skip("No writable Improved Close Range field had room to change on this device/FW")
    finally:
        # Always restore the very first raw payload read from the device, regardless of what
        # happened above - the strongest guarantee that this test leaves no lasting side effects.
        embedded_filter.set_composite_option(option_id, original_raw)
        readback_raw = embedded_filter.get_composite_option(option_id)
        assert readback_raw == original_raw, "failed to restore original HKR Improved Close Range Control value"
