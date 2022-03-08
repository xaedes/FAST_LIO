#pragma once

#include <cstdint>
#include <Eigen/Core>
#include <pcl/pcl_macros.h>

namespace fast_lio {

    /**
     * This contains a point from a Ouster lidar.
     */
    struct EIGEN_ALIGN16 PointOuster {
        PCL_ADD_POINT4D;
        float intensity;
        uint32_t t;
        uint16_t reflectivity;
        uint8_t  ring;
        uint16_t ambient;
        uint32_t range;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

} // namespace fast_lio

POINT_CLOUD_REGISTER_POINT_STRUCT(fast_lio::PointOuster,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)
