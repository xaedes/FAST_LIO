#pragma once

#include <cstdint>
#include <Eigen/Core>
#include <pcl/pcl_macros.h>

namespace fast_lio {

    /**
     * This contains a point from a Livox lidar.
     */
    struct PointLivox {
        uint32_t offset_time;      // offset time relative to the base time
        float x;                   // X axis, unit:m
        float y;                   // Y axis, unit:m
        float z;                   // Z axis, unit:m
        uint8_t reflectivity;      // reflectivity, 0~255
        uint8_t tag;               // livox tag
        uint8_t line;              // laser number in lidar
    };

} // namespace fast_lio
