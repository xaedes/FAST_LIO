#pragma once

#include <cstdint>
#include <FAST_LIO_cpp/messages/Vector3.h>

namespace fast_lio {

    /**
     * This expresses velocity in free space broken into its linear and angular
     * parts.
     */
    struct Twist
    {
        Vector3 linear{};
        Vector3 angular{};
    };

} // namespace fast_lio

