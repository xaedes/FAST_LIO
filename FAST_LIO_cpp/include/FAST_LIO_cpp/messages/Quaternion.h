#pragma once

#include <cstdint>

namespace fast_lio {

    /**
     * This represents an orientation in free space in quaternion form.
     */
    struct Quaternion
    {
        double x = 0;
        double y = 0;
        double z = 0;
        double w = 0;
    };

} // namespace fast_lio

