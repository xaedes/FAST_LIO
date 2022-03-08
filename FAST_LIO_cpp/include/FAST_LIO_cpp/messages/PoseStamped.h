#pragma once

#include <cstdint>
#include <FAST_LIO_cpp/messages/Header.h>
#include <FAST_LIO_cpp/messages/Pose.h>

namespace fast_lio {

    /**
     * A Pose with reference coordinate frame and timestamp.
     */
    struct PoseStamped
    {
        Header header{};
        Pose pose{};
    };

} // namespace fast_lio

