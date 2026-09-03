# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2026 RealSense, Inc. All Rights Reserved.
cmake_minimum_required(VERSION 3.10)

project(RealsenseExamplesHkrTemporalFilterDppMock)

# Test scaffolding proving the atomicity + round-trip behavior of the generic composite-option
# mechanism against a fake in-memory composite_option_interface standing in for real HKR/D555
# hardware, driven through the real public entry points - see rs-hkr-temporal-filter-dpp-mock.cpp.
add_executable(rs-hkr-temporal-filter-dpp-mock
    rs-hkr-temporal-filter-dpp-mock.cpp
)
set_property(TARGET rs-hkr-temporal-filter-dpp-mock PROPERTY CXX_STANDARD 14)
target_link_libraries(rs-hkr-temporal-filter-dpp-mock PRIVATE ${LRS_TARGET})
set_target_properties (rs-hkr-temporal-filter-dpp-mock PROPERTIES FOLDER Examples)
install(TARGETS rs-hkr-temporal-filter-dpp-mock RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
