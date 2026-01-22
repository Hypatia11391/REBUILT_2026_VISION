#include "RobotPoseEstimator.h"
#include "PoseMath.h"

Pose3D estimateRobotPoseFromTag(
    const Pose3D& tagInCamera,
    const Pose3D& cameraInRobot,
    const Pose3D& tagInGlobal
) {
    // Tag in robot frame
    Pose3D tagInRobot = composePose(cameraInRobot, tagInCamera);

    // Robot in tag frame
    Pose3D robotInTag = invertPose(tagInRobot);

    // Robot in global frame
    return composePose(tagInGlobal, robotInTag);
}