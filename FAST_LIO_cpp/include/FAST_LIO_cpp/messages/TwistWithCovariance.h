#pragma once

#include <cstdint>
#include <FAST_LIO_cpp/messages/Twist.h>

namespace fast_lio {

    /**
     * This expresses velocity in free space with uncertainty.
     */
    struct TwistWithCovariance
    {
        Twist twist{};

        /*
         * Row-major representation of the 6x6 covariance matrix The orientation
         * parameters use a fixed-axis representation. In order, the parameters
         * are: (x, y, z, rotation about X axis, rotation about Y axis, rotation
         * about Z axis)
         */
        double covariance[36];
    };

} // namespace fast_lio

