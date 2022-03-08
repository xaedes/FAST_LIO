#pragma once

#include <vector>
#include <FAST_LIO_cpp/messages/Header.h>
#include <FAST_LIO_cpp/messages/lidars/PointLivox.h>

namespace fast_lio {

    /**
     * Livox publish pointcloud msg format.
     */
    struct PointCloudLivox
    {
        Header header;                   // ROS standard message header
        uint64_t timebase;               // The time of first point
        uint32_t point_num;              // Total number of pointclouds
        uint8_t  lidar_id;               // Lidar device id number
        uint8_t  rsvd[3];                // Reserved use
        std::vector<PointLivox> points;  // Pointcloud data
    };

} // namespace fast_lio
