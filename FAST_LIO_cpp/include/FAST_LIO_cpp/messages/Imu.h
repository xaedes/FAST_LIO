#pragma once

#include <FAST_LIO_cpp/messages/Header.h>
#include <FAST_LIO_cpp/messages/Quaternion.h>
#include <FAST_LIO_cpp/messages/Vector3.h>

namespace fast_lio {

    struct Imu
    {
        Header header{};
        Quaternion orientation{};
        double orientation_covariance[9]; // Row major about x, y, z axes

        Vector3 angular_velocity{};
        double angular_velocity_covariance[9]; // Row major about x, y, z axes

        Vector3 linear_acceleration{};
        double linear_acceleration_covariance[9]; // Row major x, y z 
    };

} // namespace fast_lio

