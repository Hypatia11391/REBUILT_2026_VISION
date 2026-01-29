# pragma once
# include <array>
# include <optional>
# include <cstddef>
# include <Eigen/dense>

namespace constants () {
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
        Eigen::Matrix4f RobotPose_Camera(4, 4); // Because of how this is used in algorithm it is faster to store it as the robot pose in the camera frame for fewer inversions
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
    constexpr std::array<Eigen::Matrix4f, 2> AprilTagPoses_Global{{ // <---- There will be 32 tags
        // Pose for tag 0 (id = 1)
        {{0.f, 0.f, 0.f, 0.f},
         {0.f, 0.f, 0.f, 0.f},
         {0.f, 0.f, 0.f, 0.f},
         {0.f, 0.f, 0.f, 1.f}},

        // Pose for tag 1 (id = 2)
        {{0.f, 0.f, 0.f, 0.f},
         {0.f, 0.f, 0.f, 0.f},
         {0.f, 0.f, 0.f, 0.f},
         {0.f, 0.f, 0.f, 1.f}}
    }};
}