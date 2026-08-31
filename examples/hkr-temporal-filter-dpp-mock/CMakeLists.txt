# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2026 RealSense, Inc. All Rights Reserved.
cmake_minimum_required(VERSION 3.10)

project(RealsenseExamplesHkrTemporalFilterDppMock)

# PROTOTYPE / DEMO test scaffolding: proves the atomicity + round-trip behavior of the generic
# "composite option" mechanism (include/librealsense2/h/rs_composite_option.h,
# src/composite-option-interface.h) as applied to RS2_COMPOSITE_OPTION_HKR_TEMPORAL_FILTER_DPP,
# against a fake in-memory librealsense::composite_option_interface implementation standing in for
# real HKR/D555 hardware (librealsense::composite_xu_option, src/ds/composite-xu-option.h), which
# is not present on this dev machine. Drives the mechanism through the REAL PUBLIC ENTRY POINTS
# (rs2_set_composite_option/rs2_get_composite_option and the direct
# rs2::options::get_composite_option()/set_composite_option() C++ methods - no casting/handle
# type involved) rather than bypassing into an internal transport class directly, so it links
# against realsense2 like any other example.
add_executable(rs-hkr-temporal-filter-dpp-mock
    rs-hkr-temporal-filter-dpp-mock.cpp
)
set_property(TARGET rs-hkr-temporal-filter-dpp-mock PROPERTY CXX_STANDARD 14)
target_link_libraries(rs-hkr-temporal-filter-dpp-mock PRIVATE ${LRS_TARGET})
set_target_properties (rs-hkr-temporal-filter-dpp-mock PROPERTIES FOLDER Examples)
install(TARGETS rs-hkr-temporal-filter-dpp-mock RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
