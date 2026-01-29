#include "TagPoseEstimator.h"

#include <apriltag/apriltag_pose.h>

TagPoseEstimator::TagPoseEstimator(double fx, double fy,
                                   double cx, double cy,
                                   double tagSizeMeters)
    : fx_(fx), fy_(fy), cx_(cx), cy_(cy), tagSize_(tagSizeMeters)
{}

TagCameraPose TagPoseEstimator::estimate(const DetectedTag& tag) const
{
    apriltag_detection_info_t info;

    info.det = nullptr;
    info.tagsize = tagSize_;
    info.fx = fx_;
    info.fy = fy_;
    info.cx = cx_;
    info.cy = cy_;

    // Construct a fake detection to pass corners in
    apriltag_detection_t det{};
    det.id = tag.id;

    for (int i = 0; i < 4; ++i) {
        det.p[i][0] = tag.corners[i].x;
        det.p[i][1] = tag.corners[i].y;
    }

    info.det = &det;

    apriltag_pose_t pose;
    estimate_tag_pose(&info, &pose);

    TagCameraPose out;
    out.id = tag.id;

    out.translation = {
        pose.t->data[0],
        pose.t->data[1],
        pose.t->data[2]
    };

    out.rotation = cv::Matx33d(
        pose.R->data[0], pose.R->data[1], pose.R->data[2],
        pose.R->data[3], pose.R->data[4], pose.R->data[5],
        pose.R->data[6], pose.R->data[7], pose.R->data[8]
    );

    matd_destroy(pose.R);
    matd_destroy(pose.t);

    return out;
}
