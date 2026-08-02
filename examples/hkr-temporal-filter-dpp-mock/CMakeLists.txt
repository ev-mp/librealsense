# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2026 RealSense, Inc. All Rights Reserved.
cmake_minimum_required(VERSION 3.10)

project(RealsenseExamplesHkrTemporalFilterDppMock)

# PROTOTYPE / DEMO test scaffolding: proves the atomicity + round-trip behavior of the generic
# "composite option" mechanism (src/ds/structured-xu-control.*) as applied to the HKR Temporal
# Filter DPP control, against a fake in-memory platform::uvc_device standing in for real
# HKR/D555 hardware, which is not present on this dev machine. This executable deliberately
# does NOT link against the realsense2 library/DLL: the control class under test is
# transport-agnostic (it only needs a platform::uvc_device&), so it can be exercised directly
# without a live backend/uvc_sensor stack. It compiles the real production source file
# directly so it is testing actual SDK code, not a reimplementation.
add_executable(rs-hkr-temporal-filter-dpp-mock
    rs-hkr-temporal-filter-dpp-mock.cpp
    ${CMAKE_SOURCE_DIR}/src/ds/structured-xu-control.cpp
)
set_property(TARGET rs-hkr-temporal-filter-dpp-mock PROPERTY CXX_STANDARD 14)
target_include_directories(rs-hkr-temporal-filter-dpp-mock PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(rs-hkr-temporal-filter-dpp-mock PRIVATE rsutils)
set_target_properties (rs-hkr-temporal-filter-dpp-mock PROPERTIES FOLDER Examples)
install(TARGETS rs-hkr-temporal-filter-dpp-mock RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
