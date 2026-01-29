#include "ConstantsPoseAdapters.h"
#include <cmath>

static cv::Matx33d rpyToRot(double r, double p, double y) {
    double cr = std::cos(r), sr = std::sin(r);
    double cp = std::cos(p), sp = std::sin(p);
    double cy = std::cos(y), sy = std::sin(y);

    return {
        cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr,
        sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr,
        -sp,   cp*sr,            cp*cr
    };
}

Pose3D cameraPoseInRobot(std::size_t cameraIndex) {
    const auto& c = constants::Cameras[cameraIndex];

    return Pose3D{
        rpyToRot(c.roll, c.pitch, c.yaw),
        {c.x, c.y, c.z}
    };
}

Pose3D tagPoseInGlobal(const constants::TagPose& tag) {
    return Pose3D{
        rpyToRot(tag.roll, tag.pitch, tag.yaw),
        {tag.x, tag.y, tag.z}
    };
}