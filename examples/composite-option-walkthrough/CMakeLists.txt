# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2026 RealSense, Inc. All Rights Reserved.
cmake_minimum_required(VERSION 3.10)

project(RealsenseExamplesCompositeOptionWalkthrough)

# PROTOTYPE / DEMO reference sample: a "how do I use the composite-option API" walkthrough,
# exercising the FULL surface (enumeration, set, get, get_range, metadata queries) against a fake
# in-memory librealsense::composite_option_interface implementation standing in for real HKR/D555
# hardware (librealsense::composite_xu_option, src/ds/composite-xu-option.h), which is not present
# on this dev machine. See rs-hkr-temporal-filter-dpp-mock for the companion round-trip/atomicity
# proof; this sample is meant to read as documentation for other engineers. Drives everything
# through the REAL PUBLIC API (rs2::options::get_supported_composite_options()/
# set_composite_option()/get_composite_option()/get_composite_option_range()/
# supports_composite_option()/is_composite_option_read_only()/get_composite_option_description()),
# so it links against realsense2 like any other example.
add_executable(rs-composite-option-walkthrough
    rs-composite-option-walkthrough.cpp
)
set_property(TARGET rs-composite-option-walkthrough PROPERTY CXX_STANDARD 14)
target_link_libraries(rs-composite-option-walkthrough PRIVATE ${LRS_TARGET})
set_target_properties (rs-composite-option-walkthrough PROPERTIES FOLDER Examples)
install(TARGETS rs-composite-option-walkthrough RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# PROTOTYPE / DEMO: HKR MinZ Control, one call at a time, against a REAL connected device (no
# fake transport) - see rs-minz-control-walkthrough.cpp.
add_executable(rs-minz-control-walkthrough
    rs-minz-control-walkthrough.cpp
)
set_property(TARGET rs-minz-control-walkthrough PROPERTY CXX_STANDARD 14)
target_link_libraries(rs-minz-control-walkthrough PRIVATE ${LRS_TARGET})
set_target_properties (rs-minz-control-walkthrough PROPERTIES FOLDER Examples)
install(TARGETS rs-minz-control-walkthrough RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
