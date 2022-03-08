#pragma once

#include <cstdint>

namespace fast_lio {

    /**
     * This represents a vector in free space. It is only meant to represent a
     * direction. Therefore, it does not make sense to apply a translation to it
     */
    struct Vector3
    {
        double x = 0;
        double y = 0;
        double z = 0;
    };

} // namespace fast_lio

// TODO geometry_msgs/Vector3
