#pragma once

#include <vector>
#include <FAST_LIO_cpp/messages/Header.h>
#include <FAST_LIO_cpp/messages/PoseStamped.h>

namespace fast_lio {

    /**
     * An array of poses that represents a Path for a robot to follow.
     */
    struct Path
    {
        Header header{};
        std::vector<PoseStamped> poses{};
    };

} // namespace fast_lio

