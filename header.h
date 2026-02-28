#pragma once
#include <array>
#include <Eigen/Dense>

namespace constants {

    // Define a datatype to hold camera info
    struct CameraInfo {
        float fx, fy, cx, cy; 
        Eigen::Matrix4f RobotPoseInCamera; // Fixed: Removed (4,4)
    };

    // Constants
    inline constexpr float tag_size = 0.1651f;
    inline constexpr float tagsize = 0.1651f; // Alias to match your main.cpp usage
    inline constexpr int num_cams = 2;

    // Helper to create identity or custom matrices for the array
    inline Eigen::Matrix4f make_identity() { return Eigen::Matrix4f::Identity(); }
    
    // Helper to create the specific placeholder matrix you used in CameraInfo
    inline Eigen::Matrix4f make_placeholder_pose() {
        Eigen::Matrix4f m = Eigen::Matrix4f::Zero();
        m(3,3) = 1.0f;
        return m;
    }

    // Camera Configuration
    // Fixed: Using inline const for Eigen compatibility
    inline const std::array<CameraInfo, num_cams> Cameras = {{
        {1000.f, 1000.f, 960.f, 540.f, make_placeholder_pose()},
        {1000.f, 1000.f, 960.f, 540.f, make_placeholder_pose()}
    }};

    // AprilTag Field Poses (ID = index + 1)
    // All coordinates preserved exactly from your file.
    inline const std::array<Eigen::Matrix4f, 32> AprilTagPosesInGlobal = {{
        /* ID 1  */ (Eigen::Matrix4f() << 1,0,0,4.625594f,  0,1,0,0.633222f,  0,0,1,0.889000f,  0,0,0,1).finished(),
        /* ID 2  */ (Eigen::Matrix4f() << 1,0,0,4.625594f,  0,0,1,3.437890f,  0,-1,0,1.124000f, 0,0,0,1).finished(),
        /* ID 3  */ (Eigen::Matrix4f() << 1,0,0,5.222494f,  0,1,0,3.678936f,  0,0,1,1.124000f,  0,0,0,1).finished(),
        /* ID 4  */ (Eigen::Matrix4f() << 1,0,0,5.222494f,  0,1,0,4.034536f,  0,0,1,1.124000f,  0,0,0,1).finished(),
        /* ID 5  */ (Eigen::Matrix4f() << 1,0,0,4.625594f,  0,0,-1,4.631436f, 0,1,0,1.124000f,  0,0,0,1).finished(),
        /* ID 6  */ (Eigen::Matrix4f() << 1,0,0,4.625594f,  0,1,0,7.436104f,  0,0,1,0.889000f,  0,0,0,1).finished(),
        /* ID 7  */ (Eigen::Matrix4f() << 1,0,0,4.625594f,  0,-1,0,7.436104f, 0,0,-1,0.889000f, 0,0,0,1).finished(),
        /* ID 8  */ (Eigen::Matrix4f() << 1,0,0,4.269994f,  0,0,-1,4.631436f, 0,1,0,1.124000f,  0,0,0,1).finished(),
        /* ID 9  */ (Eigen::Matrix4f() << 1,0,0,3.977894f,  0,-1,0,4.390136f, 0,0,-1,1.124000f, 0,0,0,1).finished(),
        /* ID 10 */ (Eigen::Matrix4f() << 1,0,0,3.977894f,  0,-1,0,4.034536f, 0,0,-1,1.124000f, 0,0,0,1).finished(),
        /* ID 11 */ (Eigen::Matrix4f() << 1,0,0,4.269994f,  0,0,1,3.437890f,  0,-1,0,1.124000f, 0,0,0,1).finished(),
        /* ID 12 */ (Eigen::Matrix4f() << 1,0,0,4.625594f,  0,-1,0,0.633222f, 0,0,-1,0.889000f, 0,0,0,1).finished(),
        /* ID 13 */ (Eigen::Matrix4f() << 1,0,0,0.000000f,  0,1,0,0.626364f,  0,0,1,0.552450f,  0,0,0,1).finished(),
        /* ID 14 */ (Eigen::Matrix4f() << 1,0,0,0.000000f,  0,1,0,1.058164f,  0,0,1,0.552450f,  0,0,0,1).finished(),
        /* ID 15 */ (Eigen::Matrix4f() << 1,0,0,0.000000f,  0,1,0,4.034536f,  0,0,1,0.552450f,  0,0,0,1).finished(),
        /* ID 16 */ (Eigen::Matrix4f() << 1,0,0,0.000000f,  0,1,0,4.466336f,  0,0,1,0.552450f,  0,0,0,1).finished(),
        /* ID 17 */ (Eigen::Matrix4f() << 1,0,0,11.915394f, 0,-1,0,7.436104f, 0,0,-1,0.889000f, 0,0,0,1).finished(),
        /* ID 18 */ (Eigen::Matrix4f() << 1,0,0,11.915394f, 0,0,-1,4.631436f, 0,1,0,1.124000f,  0,0,0,1).finished(),
        /* ID 19 */ (Eigen::Matrix4f() << 1,0,0,11.351514f, 0,-1,0,4.390136f, 0,0,-1,1.124000f, 0,0,0,1).finished(),
        /* ID 20 */ (Eigen::Matrix4f() << 1,0,0,11.351514f, 0,-1,0,4.034536f, 0,0,-1,1.124000f, 0,0,0,1).finished(),
        /* ID 21 */ (Eigen::Matrix4f() << 1,0,0,11.915394f, 0,0,1,3.437890f,  0,-1,0,1.124000f, 0,0,0,1).finished(),
        /* ID 22 */ (Eigen::Matrix4f() << 1,0,0,11.915394f, 0,-1,0,0.633222f, 0,0,-1,0.889000f, 0,0,0,1).finished(),
        /* ID 23 */ (Eigen::Matrix4f() << 1,0,0,11.915394f, 0,1,0,0.633222f,  0,0,1,0.889000f,  0,0,0,1).finished(),
        /* ID 24 */ (Eigen::Matrix4f() << 1,0,0,12.270994f, 0,0,1,3.437890f,  0,-1,0,1.124000f, 0,0,0,1).finished(),
        /* ID 25 */ (Eigen::Matrix4f() << 1,0,0,12.549124f, 0,1,0,3.678936f,  0,0,1,1.124000f,  0,0,0,1).finished(),
        /* ID 26 */ (Eigen::Matrix4f() << 1,0,0,12.549124f, 0,1,0,4.034536f,  0,0,1,1.124000f,  0,0,0,1).finished(),
        /* ID 27 */ (Eigen::Matrix4f() << 1,0,0,12.270994f, 0,0,-1,4.631436f, 0,1,0,1.124000f,  0,0,0,1).finished(),
        /* ID 28 */ (Eigen::Matrix4f() << 1,0,0,11.915394f, 0,1,0,7.436104f,  0,0,1,0.889000f,  0,0,0,1).finished(),
        /* ID 29 */ (Eigen::Matrix4f() << 1,0,0,16.513048f, 0,-1,0,7.416292f, 0,0,-1,0.552450f, 0,0,0,1).finished(),
        /* ID 30 */ (Eigen::Matrix4f() << 1,0,0,16.513048f, 0,-1,0,6.984492f, 0,0,-1,0.552450f, 0,0,0,1).finished(),
        /* ID 31 */ (Eigen::Matrix4f() << 1,0,0,16.513048f, 0,-1,0,4.034536f, 0,0,-1,0.552450f, 0,0,0,1).finished(),
        /* ID 32 */ (Eigen::Matrix4f() << 1,0,0,16.513048f, 0,-1,0,3.602736f, 0,0,-1,0.552450f, 0,0,0,1).finished()
    }};
}