#pragma once

#include <cstdint>
#include <string>
#include <FAST_LIO_cpp/messages/Timestamp.h>

namespace fast_lio {

    struct Header
    {
        uint32_t seq = 0;
        Timestamp stamp{};
        std::string frame_id = "";
    };

} // namespace fast_lio

