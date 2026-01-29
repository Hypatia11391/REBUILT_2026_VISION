#pragma once

#include "Pose3D.h"

Pose3D estimateRobotPoseFromTag(
    const Pose3D& tagInCamera,
    const Pose3D& cameraInRobot,
    const Pose3D& tagInGlobal
);