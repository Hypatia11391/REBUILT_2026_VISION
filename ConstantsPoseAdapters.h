#pragma once

#include "constants.h"
#include "Pose3D.h"

Pose3D cameraPoseInRobot(std::size_t cameraIndex);
Pose3D tagPoseInGlobal(const constants::TagPose& tag);