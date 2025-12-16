#pragma once

#include <opencv2/core.hpp>
#include <chrono>

namespace camera {

struct Frame {
    cv::Mat image;
    std::chrono::nanoseconds timestamp;
};

}