#pragma once

#include <cstdint>
#include <Eigen/Core>
#include <pcl/pcl_macros.h>

namespace fast_lio {

    /**
     * This contains a point from a Velodyne lidar.
     */
    struct EIGEN_ALIGN16 PointVelodyne {
        PCL_ADD_POINT4D;
        float intensity;
        float time;
        uint16_t ring;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

} // namespace fast_lio

POINT_CLOUD_REGISTER_POINT_STRUCT(fast_lio::PointVelodyne,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
    (uint16_t, ring, ring)
)
