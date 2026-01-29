#pragma once
#include <opencv2/core.hpp>

struct Pose3D {
    cv::Matx33d R;   // rotation
    cv::Vec3d  t;   // translation
};