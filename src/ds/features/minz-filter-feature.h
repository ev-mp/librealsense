// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

// PROTOTYPE / DEMO feature - not a finalized production control.

#pragma once

#include <src/feature-interface.h>


namespace librealsense {

class d500_depth_sensor;

// Mirrors temporal_filter_feature exactly (see temporal-filter-feature.h) - wires the HKR MinZ
// Control composite-option embedded filter (d500_minz_embedded_filter) onto a d500_depth_sensor.
class minz_filter_feature : public feature_interface
{
public:
    static const feature_id ID;

    explicit minz_filter_feature( d500_depth_sensor & depth_sensor );

    feature_id get_id() const override;
};

}  // namespace librealsense
