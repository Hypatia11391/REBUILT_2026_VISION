# pragma once

# include <opencv2/core.hpp>
# include <vector>

struct DetectedTag {
    int id;
    std::array<cv.Point2f, 4> corners;
};

class AprilTagDetector {
public:
    AprilTagDetector();
    ~AprilTagDetector();

    std::vector<DetectedTag> detect(const cv::Mat& gray) const;

private:
    struct Impl;
    Impl* impl_;
};