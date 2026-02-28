# pragma once
# include <array>
# include <optional>
# include <cstddef>
# include <Eigen/Dense>

namespace constants {
    /*// Define a structure for poses
    struct Pose3D {
    Eigen::MatrixXD R(3, 3);   // rotation
    Eigen::MatrixXD R(3, 3);   // translation
    };*/

    /*// Define a datatype for tag pose
    struct TagInfo {
        int tagID;
        Eigen::Matrix4f TagPose_Global(4, 4);
    };*/

    // Define a datatype to hold camera info
    struct CameraInfo {
        float fx, fy, cx, cy; // Intrinsics
        Eigen::Matrix4f RobotPoseInCamera(4, 4); // Because of how this is used in algorithm it is faster to store it as the robot pose in the camera frame for fewer inversions
    };

    // Define the tag size (in meters)
    constexpr float tag_size = 0.1651; // [PLACEHOLDER]
    constexpr int num_cams = 2;

    // Create an array of all the cameras attached to the Pi
    constexpr std::array<CameraInfo, num_cams> Cameras{{
        // Info for camera 0
        {1000.f, 1000.f, 960.f, 540.f, // [PLACEHOLDER]
        {{0.f, 0.f, 0.f, 0.f},
         {0.f, 0.f, 0.f, 0.f},
         {0.f, 0.f, 0.f, 0.f},
         {0.f, 0.f, 0.f, 1.f}}
        },

        // Info for camera 1
        {1000.f, 1000.f, 960.f, 540.f, // [PLACEHOLDER]
        {{0.f, 0.f, 0.f, 0.f},
         {0.f, 0.f, 0.f, 0.f},
         {0.f, 0.f, 0.f, 0.f},
         {0.f, 0.f, 0.f, 1.f}}
        },
    }};

    // Create an array of all the AprilTags on the field. ID is ordered such that tag with ID = n is at index = n - 1.
    constexpr std::array<Eigen::Matrix4f, 32> AprilTagPosesInGlobal{{ // <---- There will be 32 tags
        // Pose for tag 0 (id = 1)
        {{1.f, 0.f, 0.f, 4.625594f},
         {0.f, 1.f, 0.f, 0.633222f},
         {0.f, 0.f, 1.f, 0.889f},
         {0.f, 0.f, 0.f, 1.f}},

        // Pose for tag 1 (id = 2)
        {{1.f, 0.f, 0.f, 4.625594f},
         {0.f, 0.f, 1.f, 3.437890f},
         {0.f, -1.f, 0.f, 1.124000f},
         {0.f, 0.f, 0.f, 1.f}},

        // Pose for tag 2 (id = 3)
        {{1.f, 0.f, 0.f, 5.222494f},
         {0.f, 1.f, 0.f, 3.678936f},
         {0.f, 0.f, 1.f, 1.124000f},
         {0.f, 0.f, 0.f, 1.f}},

        // Pose for tag 3 (id = 4)
        {{1.f, 0.f, 0.f, 5.222494f},
         {0.f, 1.f, 0.f, 4.034536f},
         {0.f, 0.f, 1.f, 1.124000f},
         {0.f, 0.f, 0.f, 1.f}},

        // Pose for tag 4 (id = 5)
        {{1.f, 0.f, 0.f,  4.625594f},
         {0.f, 0.f, -1.f, 4.631436f},
         {0.f, 1.f, 0.f, 1.124000f},
         {0.f, 0.f, 0.f, 1.f}},
        
        // Pose for tag 5 (id = 6)
        {{1.f, 0.f, 0.f, 4.625594f},
         {0.f, 1.f, 0.f, 7.436104f},
         {0.f, 0.f, 1.f, 0.889f},
         {0.f, 0.f, 0.f, 1.f}},

        // Pose for tag 6 (id = 7)
        {{1.f, 0.f, 0.f,  4.625594f},
         {0.f, -1.f, 0.f, 7.436104f},
         {0.f, 0.f, -1.f, 0.889f},
         {0.f, 0.f, 0.f, 1.f}},

        // Pose for tag 8 (id = 9)
        {{1.f, 0.f, 0.f,  3.977894f},
         {0.f, -1.f, 0.f, 4.390136},
         {0.f, 0.f, -1.f, 1.124000f},
         {0.f, 0.f, 0.f, 1.f}},

        // Pose for tag 9 (id = 10)
        {{1.f, 0.f, 0.f,  3.977894f},
         {0.f, -1.f, 0.f, 4.034536f},
         {0.f, 0.f, -1.f, 1.124000f},
         {0.f, 0.f, 0.f, 1.f}},

        // Pose for tag 10 (id = 11)
        {{1.f, 0.f, 0.f,  4.269994f},
         {0.f, 0.f, 1.f, 3.437890f},
         {0.f, -1.f, 0.f, 1.124000f},
         {0.f, 0.f, 0.f, 1.f}},

        // Pose for tag 11 (id = 12)
        {{1.f, 0.f, 0.f,  4.625594f},
         {0.f, -1.f, 0.f, 0.633222f},
         {0.f, 0.f, -1.f, 0.889f},
         {0.f, 0.f, 0.f, 1.f}},

        // Pose for tag 12 (id = 13)
        {{1.f, 0.f, 0.f, 0.f},
         {0.f, 1.f, 0.f, 0.626364f},
         {0.f, 0.f, 1.f, 0.55245f},
         {0.f, 0.f, 0.f, 1.f}},

        // Pose for tag 13 (id = 14)
        {{1.f, 0.f, 0.f, 0.f},
         {0.f, 1.f, 0.f, 1.058164f},
         {0.f, 0.f, 1.f, 0.55245f},
         {0.f, 0.f, 0.f, 1.f}},

        // Pose for tag 14 (id = 15)
        {{1.f, 0.f, 0.f, 0.f},
         {0.f, 1.f, 0.f, 4.034536f},
         {0.f, 0.f, 1.f, 0.55245f},
         {0.f, 0.f, 0.f, 1.f}},

        // Pose for tag 15 (id = 16)
        {{1.f, 0.f, 0.f, 0.f},
         {0.f, 1.f, 0.f, 4.466336f},
         {0.f, 0.f, 1.f, 0.55245f},
         {0.f, 0.f, 0.f, 1.f}},

        // Pose for tag 16 (id = 17)
        {{1.f, 0.f, 0.f,  11.915394f},
         {0.f, -1.f, 0.f, 7.436104f},
         {0.f, 0.f, -1.f, 0.889f},
         {0.f, 0.f, 0.f, 1.f}},
        
        // Pose for tag 17 (id = 18)
        {{1.f, 0.f, 0.f,  11.915394f},
         {0.f, 0.f, -1.f, 4.631436f},
         {0.f, 1.f, 0.f, 1.124000f},
         {0.f, 0.f, 0.f, 1.f}},

        // Pose for tag 18 (id = 19)
        {{1.f, 0.f, 0.f,  11.351514f},
         {0.f, -1.f, 0.f, 4.390136f},
         {0.f, 0.f, -1.f, 1.124000f},
         {0.f, 0.f, 0.f, 1.f}},

        // Pose for tag 19 (id = 20)
        {{1.f, 0.f, 0.f,  11.351514f},
         {0.f, -1.f, 0.f, 4.034536f},
         {0.f, 0.f, -1.f, 1.124000f},
         {0.f, 0.f, 0.f, 1.f}},

        // Pose for tag 20 (id = 21)
        {{1.f, 0.f, 0.f,  11.915394f},
         {0.f, 0.f, 1.f, 3.437890f},
         {0.f, -1.f, 0.f, 1.124000f},
         {0.f, 0.f, 0.f, 1.f}},
        
        // Pose for tag 21 (id = 22)
        {{1.f, 0.f, 0.f,  11.915394f},
         {0.f, -1.f, 0.f, 0.633222f},
         {0.f, 0.f, -1.f, 0.889f},
         {0.f, 0.f, 0.f, 1.f}},
        
        // Pose for tag 22 (id = 23)
        {{1.f, 0.f, 0.f, 11.915394f},
         {0.f, 1.f, 0.f, 0.633222f},
         {0.f, 0.f, 1.f, 0.889f},
         {0.f, 0.f, 0.f, 1.f}},

        // Pose for tag 23 (id = 24)
        {{1.f, 0.f, 0.f,  12.270994f},
         {0.f, 0.f, 1.f, 3.437890f},
         {0.f, -1.f, 0.f, 1.124000f},
         {0.f, 0.f, 0.f, 1.f}},

        // Pose for tag 24 (id = 25)
        {{1.f, 0.f, 0.f, 12.549124f},
         {0.f, 1.f, 0.f, 3.678936f},
         {0.f, 0.f, 1.f, 1.124000f},
         {0.f, 0.f, 0.f, 1.f}},

        // Pose for tag 25 (id = 26)
        {{1.f, 0.f, 0.f, 12.549124f},
         {0.f, 1.f, 0.f, 4.034536f},
         {0.f, 0.f, 1.f, 1.124000f},
         {0.f, 0.f, 0.f, 1.f}},
        
        // Pose for tag 26 (id = 27)
        {{1.f, 0.f, 0.f,  12.270994},
         {0.f, 0.f, -1.f, 4.631436f},
         {0.f, 1.f, 0.f, 1.124000f},
         {0.f, 0.f, 0.f, 1.f}},

        // Pose for tag 27 (id = 28)
        {{1.f, 0.f, 0.f, 11.915394f},
         {0.f, 1.f, 0.f, 7.436104f},
         {0.f, 0.f, 1.f, 0.889f},
         {0.f, 0.f, 0.f, 1.f}},

        // Pose for tag 28 (id = 29)
        {{1.f, 0.f, 0.f,  16.513048f},
         {0.f, -1.f, 0.f, 7.416292f},
         {0.f, 0.f, -1.f, 0.55245f},
         {0.f, 0.f, 0.f, 1.f}},

        // Pose for tag 29 (id = 30)
        {{1.f, 0.f, 0.f,  16.513048f},
         {0.f, -1.f, 0.f, 6.984492f},
         {0.f, 0.f, -1.f, 0.55245f},
         {0.f, 0.f, 0.f, 1.f}},

        // Pose for tag 30 (id = 31)
        {{1.f, 0.f, 0.f,  16.513048f},
         {0.f, -1.f, 0.f, 4.034536f},
         {0.f, 0.f, -1.f, 0.55245f},
         {0.f, 0.f, 0.f, 1.f}},

        // Pose for tag 31 (id = 32)
        {{1.f, 0.f, 0.f,  16.513048f},
         {0.f, -1.f, 0.f, 3.602736f},
         {0.f, 0.f, -1.f, 0.55245f},
         {0.f, 0.f, 0.f, 1.f}},
    }};
}