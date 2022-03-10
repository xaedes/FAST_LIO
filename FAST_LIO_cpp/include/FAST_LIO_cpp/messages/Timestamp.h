#pragma once

#include <cstdint>
#include <chrono>
#include <cmath>

namespace fast_lio {

    struct Timestamp
    {
        // seconds (stamp_secs) since epoch 
        uint32_t sec  = 0;
        // nanoseconds since stamp_secs
        uint32_t nsec  = 0;

        double toSec() const
        {
            return static_cast<double>(sec) + 1e-9 * static_cast<double>(nsec);
        }

        static Timestamp now()
        {
            auto tp = std::chrono::system_clock::now();
            std::chrono::time_point<std::chrono::system_clock> tp0;
            std::chrono::duration<double> d = tp-tp0;
            return Timestamp().fromSec(d.count());
        }

        Timestamp fromSec(double sec)
        {
            Timestamp result;
            result.sec = static_cast<uint32_t>(std::trunc(sec));
            result.nsec = static_cast<uint32_t>((sec - result.sec) * 1e9);
            return result;
        }
    };

} // namespace fast_lio

