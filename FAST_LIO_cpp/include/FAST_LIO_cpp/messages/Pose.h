#pragma once

#include <cstdint>
#include <FAST_LIO_cpp/messages/Point.h>
#include <FAST_LIO_cpp/messages/Quaternion.h>

namespace fast_lio {

    /**
     * A representation of pose in free space, composed of position and orientation. 
     */
    struct Pose
    {
        Point position{};
        Quaternion orientation{};
    };

} // namespace fast_lio

