#include "PoseMath.h"

Pose3D invertPose(const Pose3D& p) {
    Pose3D out;
    out.R = p.R.t();
    out.t = -(out.R * p.t);
    return out;
}

Pose3D composePose(const Pose3D& a, const Pose3D& b) {
    Pose3D out;
    out.R = a.R * b.R;
    out.t = a.R * b.t + a.t;
    return out;
}