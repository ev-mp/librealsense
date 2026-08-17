# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2026 RealSense, Inc. All Rights Reserved.
cmake_minimum_required(VERSION 3.10)

project(RealsenseExamplesCompositeOptionWalkthrough)

# PROTOTYPE / DEMO reference sample: a "how do I use the composite-option API" walkthrough,
# exercising the FULL surface (enumeration, get, read-modify-write set, get_range, metadata
# queries) against every composite option found on every embedded filter of every depth sensor of
# every connected device - no fake/mock transport. Each composite option id found gets its own
# try/catch, so a control that's registered but non-functional on this device/FW (e.g.
# RS2_COMPOSITE_OPTION_HKR_TEMPORAL_FILTER_DPP's ctrl_id not being validated against real
# firmware, see src/ds/d500/d500-factory.cpp's rs555_device comment) is reported per-id rather
# than aborting the whole walkthrough - see rs-composite-option-walkthrough.cpp for the full
# sequence and per-id dispatch. Drives everything through the REAL PUBLIC API
# (rs2::options::get_supported_composite_options()/set_composite_option()/get_composite_option()/
# get_composite_option_range()/is_composite_option_read_only()/get_composite_option_description()),
# so it links against realsense2 like any other example.
add_executable(rs-composite-option-walkthrough
    rs-composite-option-walkthrough.cpp
)
set_property(TARGET rs-composite-option-walkthrough PROPERTY CXX_STANDARD 14)
target_link_libraries(rs-composite-option-walkthrough PRIVATE ${LRS_TARGET})
set_target_properties (rs-composite-option-walkthrough PROPERTIES FOLDER Examples)
install(TARGETS rs-composite-option-walkthrough RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# PROTOTYPE / DEMO: sweeps every connected device's depth sensor(s), enumerates whichever
# composite options each one supports, and prints the current value of each - against REAL
# connected devices (no fake transport). See rs-minz-control-walkthrough.cpp.
add_executable(rs-minz-control-walkthrough
    rs-minz-control-walkthrough.cpp
)
set_property(TARGET rs-minz-control-walkthrough PROPERTY CXX_STANDARD 14)
target_link_libraries(rs-minz-control-walkthrough PRIVATE ${LRS_TARGET})
set_target_properties (rs-minz-control-walkthrough PROPERTIES FOLDER Examples)
install(TARGETS rs-minz-control-walkthrough RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# PROTOTYPE / DEMO: same MinZ get/set walkthrough, through the raw C API instead of the C++
# wrapper - no exceptions, no templates, manual struct casts and buffer frees. See
# rs-composite-option-c99.c.
add_executable(rs-composite-option-c99
    rs-composite-option-c99.c
)
set_property(TARGET rs-composite-option-c99 PROPERTY C_STANDARD 99)
target_link_libraries(rs-composite-option-c99 PRIVATE ${LRS_TARGET})
set_target_properties (rs-composite-option-c99 PROPERTIES FOLDER Examples)
install(TARGETS rs-composite-option-c99 RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
