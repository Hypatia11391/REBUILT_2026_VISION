#pragma once

#include "AprilTagDetector.h"
#include <opencv2/core.hpp>

struct TagCameraPose {
    int id;

    cv::Vec3d translation;  // meters, camera frame
    cv::Matx33d rotation;   // rotation matrix camera->tag
};

class TagPoseEstimator {
public:
    TagPoseEstimator(double fx, double fy, double cx, double cy,
                     double tagSizeMeters);

    TagCameraPose estimate(const DetectedTag& tag) const;

private:
    double fx_, fy_, cx_, cy_;
    double tagSize_;
};