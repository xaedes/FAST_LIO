#pragma once

#include <cstdint>

namespace fast_lio {

    struct Timestamp
    {
        // seconds (stamp_secs) since epoch 
        uint32_t sec  = 0;
        // nanoseconds since stamp_secs
        uint32_t nsec  = 0;
    };

} // namespace fast_lio

