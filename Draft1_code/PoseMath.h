#pragma once
#include "Pose3D.h"

Pose3D invertPose(const Pose3D& p);
Pose3D composePose(const Pose3D& a, const Pose3D& b);