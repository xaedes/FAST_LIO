#pragma once

#include <string>
#include <FAST_LIO_cpp/messages/Header.h>
#include <FAST_LIO_cpp/messages/PoseWithCovariance.h>
#include <FAST_LIO_cpp/messages/TwistWithCovariance.h>

namespace fast_lio {

    /**
     * This represents an estimate of a position and velocity in free space. The
     * pose in this message should be specified in the coordinate frame given by
     * header.frame_id. The twist in this message should be specified in the
     * coordinate frame given by the child_frame_id
     */
    struct Odometry
    {
        Header header{};
        std::string child_frame_id = "";

        PoseWithCovariance pose{};
        TwistWithCovariance twist{};
    };

} // namespace fast_lio

