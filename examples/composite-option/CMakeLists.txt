# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2026 RealSense, Inc. All Rights Reserved.
cmake_minimum_required(VERSION 3.10)

project(RealsenseExamplesCompositeOption)

# PROTOTYPE / DEMO reference sample: a "how do I use the composite-option API" walkthrough,
# exercising the FULL surface (enumeration, get, read-modify-write set, get_range, metadata
# queries) against every composite option found on every embedded filter of every depth sensor of
# every connected device - no fake/mock transport. Each composite option id found gets its own
# try/catch, so a control that's registered but non-functional on this device/FW (e.g.
# RS2_COMPOSITE_OPTION_HKR_TEMPORAL_FILTER_DPP's ctrl_id not being validated against real
# firmware, see src/ds/d500/d500-factory.cpp's rs555_device comment) is reported per-id rather
# than aborting the whole walkthrough - see rs-composite-option-demo.cpp for the full sequence and
# per-id dispatch. Drives everything through the REAL PUBLIC API
# (rs2::options::get_supported_composite_options()/set_composite_option()/get_composite_option()/
# get_composite_option_range()/is_composite_option_read_only()/get_composite_option_description()),
# so it links against realsense2 like any other example.
add_executable(rs-composite-option-demo
    rs-composite-option-demo.cpp
)
set_property(TARGET rs-composite-option-demo PROPERTY CXX_STANDARD 14)
target_link_libraries(rs-composite-option-demo PRIVATE ${LRS_TARGET})
set_target_properties (rs-composite-option-demo PROPERTIES FOLDER Examples)
install(TARGETS rs-composite-option-demo RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

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

# The C99 raw-API counterpart of rs-composite-option-demo.cpp lives under examples/C/composite-ctl
# (see rs-composite-ctl.c there) - Examples/C, not Examples, matching where every other pure-C
# sample (rs-infrared, rs-depth, ...) lives.
