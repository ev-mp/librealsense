# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

"""Verifies the *effect* of the Decimation Filter DPP composite option on the actual depth
stream, as distinct from pytest-decimation-filter-dpp.py (which only bounces the control's
fields and checks readback - it never opens a stream). Per the HKR Depth Post Processing SW
Arch doc (Confluence pageId=10789655, Use-Case #2) and 640x360 is also a natively-offered depth
profile (produced by the sensor without any DPP involvement), so frame size alone can't tell
the two apart - the RS2_FRAME_METADATA_EMBEDDED_FILTERS bitmask is the only way to confirm the
640x360 frames were actually produced by the DPP decimation block rather than the native path:
  1. With Decimation Filter DPP enabled (enable=1, magnitude=2) *before* streaming starts and the
     640x360 depth profile selected, the resulting frames are still expected to be 640x360.
  2. Those same frames are expected to report RS2_FRAME_METADATA_EMBEDDED_FILTERS with the
     decimation bit (1u << 0) set - proving the DPP block, not the native path, produced them.
  3. With Decimation Filter DPP disabled, the 640x360 profile still streams at 640x360, but the
     decimation bit must NOT be set (the frames come from the native path instead).

Known FW limitation (confirmed 2026-09-04 on D585 Proto Dual RGB, decimation-filter-dpp branch):
RS2_FRAME_METADATA_EMBEDDED_FILTERS is wired up in d500-device.cpp and *is* reliable - reading it
with Temporal Filter DPP or HDRD/Improved Close Range Control enabled instead (see
pytest-embedded-filters-metadata.py) correctly and additively sets their respective bits, and
reads back 0x00 with all three DPP filters disabled. Decimation's documented bit (1u<<0) is the
one exception: it never sets, regardless of the Decimation Filter DPP enable state, at both
1280x720 and 640x360. So this isn't a metadata-plumbing bug - it's specific to the decimation DPP
block itself not being applied by FW on this prototype build. test_..._with_metadata_set is left
as a real (non-xfail) failing assertion until FW implements it - see that test's own docstring for
why xfail isn't used in this repo.
"""

import pytest
import pyrealsense2 as rs
import logging
log = logging.getLogger(__name__)

pytestmark = [
    pytest.mark.device_each("D555"),
    pytest.mark.device_each("D585"),
    pytest.mark.device_exclude("D585S"),  # not registered on the safety-certified D585S (see d500-factory.cpp)
]

STREAM_WIDTH, STREAM_HEIGHT = 640, 360
DECIMATION_APPLIED_BIT = 1 << 0  # RS2_FRAME_METADATA_EMBEDDED_FILTERS bit layout, per Confluence doc


def _find_decimation_filter_dpp_filter(sensor):
    """Return the embedded filter exposing Decimation Filter DPP on this sensor, or None - the
    composite option lives on one of the sensor's embedded filters, its own independent options
    registry, not on the sensor itself (see rs2::embedded_filter)."""
    for embedded_filter in sensor.query_embedded_filters():
        if rs.composite_option_id.decimation_filter_dpp in embedded_filter.get_supported_composite_options():
            return embedded_filter
    return None


def _has_profile(depth_sensor, width, height):
    return any(p.stream_type() == rs.stream.depth
               and p.as_video_stream_profile().width() == width
               and p.as_video_stream_profile().height() == height
               for p in depth_sensor.get_stream_profiles())


def _set_enabled(embedded_filter, option_id, enabled, magnitude=None):
    """Decimation Filter DPP is read/write only while Depth/IR is idle - call this only before
    a stream is opened, never while one is active."""
    cfg = embedded_filter.get_decimation_filter_dpp_config(option_id)
    cfg.enabled = 1 if enabled else 0
    if magnitude is not None:
        cfg.magnitude = magnitude
    embedded_filter.set_decimation_filter_dpp_config(option_id, cfg)


def _stream_one_depth_frame(dev, ctx, width, height, fps=30):
    """Opens the given depth profile, streams a few frames to let the pipeline settle past any
    startup transient, and returns the last one."""
    cfg = rs.config()
    # On hubless multi-device rigs the context sees every connected device; without
    # enable_device(sn) the pipeline picks the first match.
    cfg.enable_device(dev.get_info(rs.camera_info.serial_number))
    cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
    pipe = rs.pipeline(ctx)
    pipe.start(cfg)
    try:
        frame = None
        for _ in range(10):
            frameset = pipe.wait_for_frames(5000)
            frame = frameset.get_depth_frame()
        return frame
    finally:
        pipe.stop()


def _decimation_filter_or_skip(depth_sensor):
    embedded_filter = _find_decimation_filter_dpp_filter(depth_sensor)
    if embedded_filter is None:
        pytest.skip("Decimation Filter DPP composite option not supported on this device")

    option_id = rs.composite_option_id.decimation_filter_dpp
    try:
        embedded_filter.get_composite_option(option_id)
    except RuntimeError as e:
        # Registered but not actually functional on this device/FW is a real, expected outcome -
        # get_supported_composite_options() only reflects static registration, never a live
        # capability check.
        pytest.skip(f"Decimation Filter DPP registered but not functional on this device/FW: {e}")

    if not _has_profile(depth_sensor, STREAM_WIDTH, STREAM_HEIGHT):
        pytest.skip(f"No {STREAM_WIDTH}x{STREAM_HEIGHT} depth profile on this device")

    return embedded_filter, option_id


def test_decimation_filter_dpp_produces_640x360_with_metadata_set(test_device):
    """Enable Decimation Filter DPP (enable=1, magnitude=2) before streaming starts, then stream
    the 640x360 depth profile: the frames must actually be 640x360 and must carry
    RS2_FRAME_METADATA_EMBEDDED_FILTERS with the decimation bit set.

    Expected to fail today on real HW - see the module docstring's "Known FW limitation" note.
    Left as a real (non-xfail) assertion, both because this repo's conftest.py crashes on xfail
    reports (ExceptionChainRepr isn't subscriptable in pytest_runtest_makereport) and because a
    plain failure is the most visible signal the moment FW starts populating this attribute."""
    dev, ctx = test_device
    depth_sensor = dev.first_depth_sensor()
    embedded_filter, option_id = _decimation_filter_or_skip(depth_sensor)

    try:
        _set_enabled(embedded_filter, option_id, enabled=True, magnitude=2)
        frame = _stream_one_depth_frame(dev, ctx, STREAM_WIDTH, STREAM_HEIGHT)
        assert frame is not None, "No depth frame received with decimation enabled"
        assert (frame.get_width(), frame.get_height()) == (STREAM_WIDTH, STREAM_HEIGHT), (
            f"Expected {STREAM_WIDTH}x{STREAM_HEIGHT}, got {frame.get_width()}x{frame.get_height()}")

        assert frame.supports_frame_metadata(rs.frame_metadata_value.embedded_filters), (
            "RS2_FRAME_METADATA_EMBEDDED_FILTERS not reported as supported on this frame "
            "with Decimation Filter DPP enabled")
        value = frame.get_frame_metadata(rs.frame_metadata_value.embedded_filters)
        assert value & DECIMATION_APPLIED_BIT, (
            f"RS2_FRAME_METADATA_EMBEDDED_FILTERS = {value:#x}, decimation bit (1<<0) not set")
    finally:
        _set_enabled(embedded_filter, option_id, enabled=False)


def test_decimation_filter_dpp_disabled_does_not_set_metadata_bit(test_device):
    """Baseline/negative check: with Decimation Filter DPP disabled, the same 640x360 profile
    still streams at 640x360 (produced by the native path), but the decimation bit of
    RS2_FRAME_METADATA_EMBEDDED_FILTERS must not be set."""
    dev, ctx = test_device
    depth_sensor = dev.first_depth_sensor()
    embedded_filter, option_id = _decimation_filter_or_skip(depth_sensor)

    try:
        _set_enabled(embedded_filter, option_id, enabled=False)
        frame = _stream_one_depth_frame(dev, ctx, STREAM_WIDTH, STREAM_HEIGHT)
        assert frame is not None, "No depth frame received with decimation disabled"
        assert (frame.get_width(), frame.get_height()) == (STREAM_WIDTH, STREAM_HEIGHT), (
            f"Expected {STREAM_WIDTH}x{STREAM_HEIGHT}, got {frame.get_width()}x{frame.get_height()}")

        if frame.supports_frame_metadata(rs.frame_metadata_value.embedded_filters):
            value = frame.get_frame_metadata(rs.frame_metadata_value.embedded_filters)
            assert not (value & DECIMATION_APPLIED_BIT), (
                f"RS2_FRAME_METADATA_EMBEDDED_FILTERS = {value:#x} has the decimation bit set "
                f"even though Decimation Filter DPP is disabled")
    finally:
        _set_enabled(embedded_filter, option_id, enabled=False)
