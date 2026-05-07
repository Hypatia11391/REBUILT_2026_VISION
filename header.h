#pragma once
#include <array>
#include <Eigen/Dense>

namespace constants {

    // Define a datatype to hold camera info
    struct CameraInfo {
        cv::Matx33d intrinsics;
        cv::Vec4d distortion_coeff;
        Eigen::Matrix4d RobotPoseInCamera;
    };

    // Constants
    inline constexpr float tag_size = 0.1651f;
    //inline constexpr float tagsize = 0.1651f;
    inline constexpr int num_cams = 2;
    // Camera Configuration

    inline const std::array<CameraInfo, num_cams> Cameras = {{
// z to x x to y y to z
        /*{900.f, 900.f, 655.f, 595.f, (Eigen::Matrix4f() << 0,0,-1, 0.282f, //972.339f, 972.207f, 712.366f, 538.83f,
                                                                                           -1,0,0,  -0.15f,
                                                                                           0,1,0, -0.355f,
                                                                                           0,0,0,  1).finished()},*/
    {
        (cv::Matx33d() << 900.0, 0, 655.0, //972.339f, 972.207f, 712.366f, 538.83f,
                                0, 900.0, 595.0,
                                0, 0, 1).finished(),
        (cv::Vec4d() << 0.0, 0.0, 0.0, 0.0).finished(), // <---------- PLACEHOLDER
        (Eigen::Matrix4d() << 0,0,-1, 0.282f, 
                            -1,0,0,  -0.15f,
                            0,1,0, -0.355f,
                            0,0,0,  1).finished()
    },
// z to -x x to z
        /*{900.f, 900.f, 627.5f, 550.f, (Eigen::Matrix4f() << 0,0,1, -0.07f, //971.697f, 970.822f, 660.752f, 519.795f,
                                                                              0,-1,0,  0.155f,
                                                                              1,0,0, 0.10f,
                                                                          0,0,0,  1).finished()*/
    {
        (cv::Matx33d() << 900.0, 0, 627.0,
                                0, 900.0, 550.0,
                                0, 0, 1).finished(),
        (cv::Vec4d() << 0.0, 0.0, 0.0, 0.0).finished(), // <---------- PLACEHOLDER
        (Eigen::Matrix4d() << 0,0,1, -0.07f,
                            0,-1,0,  0.155f,
                            1,0,0, 0.10f,
                            0,0,0,  1).finished()
    }}};

    // AprilTag Field Poses (ID = index + 1)
    inline const std::array<Eigen::Matrix4f, 32> AprilTagPosesInGlobal = {{
        /* ID 1  */ (Eigen::Matrix4f() << 0,0,-1,  4.625594f,
                                          1,0,0, 0.633222f,
                                          0,-1,0, 0.889000f,
                                          0,0,0,  1).finished(),
        /* ID 2  */ (Eigen::Matrix4f() << 1,0,0,  4.625594f,  
                                          0,0,1,  3.437890f,
                                          0,-1,0, 1.124000f,
                                          0,0,0,  1).finished(),
        /* ID 3  */ (Eigen::Matrix4f() << 0,0,-1,  5.222494f,
                                          1,0,0, 3.678936f,
                                          0,-1,0, 1.124000f,
                                          0,0,0, 1).finished(),
        /* ID 4  */ (Eigen::Matrix4f() << 0,0,-1,  5.222494f,
                                          1,0,0, 4.034536f,
                                          0,-1,0, 1.124000f,
                                          0,0,0,1).finished(),
        /* ID 5  */ (Eigen::Matrix4f() << -1,0,0, 4.625594f,
                                          0,0,-1, 4.631436f,
                                          0,-1,0, 1.124000f, 
                                          0,0,0,1).finished(),
        /* ID 6  */ (Eigen::Matrix4f() << 0,0,-1,  4.625594f,
                                          1,0,0, 7.436104f,
                                          0,-1,0, 0.889000f,
                                          0,0,0,1).finished(),
        /* ID 7  */ (Eigen::Matrix4f() << 0,0,1, 4.625594f,
                                          -1,0,0,  7.436104f,
                                          0,-1,0, 0.889000f,
                                          0,0,0,1).finished(),
        /* ID 8  */ (Eigen::Matrix4f() << -1,0,0, 4.269994f,
                                          0,0,-1, 4.631436f,
                                          0,-1,0, 1.124000f,
                                          0,0,0,1).finished(),
        /* ID 9  */ (Eigen::Matrix4f() << 0,0,1, 3.977894f,
                                          -1,0,0,  4.390136f,
                                          0,-1,0, 1.124000f,
                                          0,0,0,1).finished(),
        /* ID 10 */ (Eigen::Matrix4f() << 0,0,1, 3.977894f,
                                          -1,0,0,  4.034536f,
                                          0,-1,0, 1.124000f, 
                                          0,0,0,1).finished(),
        /* ID 11 */ (Eigen::Matrix4f() << 1,0,0,  4.269994f,
                                          0,0,1,  3.437890f,
                                          0,-1,0, 1.124000f, 
                                          0,0,0,1).finished(),
        /* ID 12 */ (Eigen::Matrix4f() << 0,0,1, 4.625594f,
                                          -1,0,0,  0.633222f,
                                          0,-1,0, 0.889000f,
                                          0,0,0,1).finished(),
        /* ID 13 */ (Eigen::Matrix4f() << 0,0,-1,  0.000000f,
                                          1,0,0, 0.626364f,
                                          0,-1,0, 0.552450f,
                                          0,0,0,1).finished(),
        /* ID 14 */ (Eigen::Matrix4f() << 0,0,-1,  0.000000f,
                                          1,0,0, 1.058164f,
                                          0,-1,0, 0.552450f,
                                          0,0,0,1).finished(),
        /* ID 15 */ (Eigen::Matrix4f() << 0,0,-1,  0.000000f,
                                          1,0,0, 4.034536f,
                                          0,-1,0, 0.552450f,
                                          0,0,0,1).finished(),
        /* ID 16 */ (Eigen::Matrix4f() << 0,0,-1,  0.000000f,
                                          1,0,0, 4.466336f,
                                          0,-1,0, 0.552450f,
                                          0,0,0,1).finished(),
        /* ID 17 */ (Eigen::Matrix4f() << 0,0,1, 11.915394f,
                                          -1,0,0,  7.436104f,
                                          0,-1,0, 0.889000f,
                                          0,0,0,1).finished(),
        /* ID 18 */ (Eigen::Matrix4f() << -1,0,0, 11.915394f,
                                          0,0,-1, 4.631436f,
                                          0,-1,0, 1.124000f,  
                                          0,0,0,1).finished(),
        /* ID 19 */ (Eigen::Matrix4f() << 0,0,1, 11.351514f, 
                                          -1,0,0,  4.390136f, 
                                          0,-1,0, 1.124000f, 
                                          0,0,0,1).finished(),
        /* ID 20 */ (Eigen::Matrix4f() << 0,0,1, 11.351514f,
                                          -1,0,0,  4.034536f, 
                                          0,-1,0, 1.124000f, 
                                          0,0,0,1).finished(),
        /* ID 21 */ (Eigen::Matrix4f() << 1,0,0,  11.915394f,
                                          0,0,1,  3.437890f,  
                                          0,-1,0, 1.124000f, 
                                          0,0,0,1).finished(),
        /* ID 22 */ (Eigen::Matrix4f() << 0,0,1, 11.915394f, 
                                          -1,0,0,  0.633222f, 
                                          0,-1,0, 0.889000f, 
                                          0,0,0,1).finished(),
        /* ID 23 */ (Eigen::Matrix4f() << 0,0,-1,  11.915394f,
                                          1,0,0, 0.633222f,  
                                          0,-1,0, 0.889000f,  
                                          0,0,0,1).finished(),
        /* ID 24 */ (Eigen::Matrix4f() << 1,0,0,  12.270994f,
                                          0,0,1,  3.437890f,
                                          0,-1,0, 1.124000f, 
                                          0,0,0,1).finished(),
        /* ID 25 */ (Eigen::Matrix4f() << 0,0,-1,  12.549124f,
                                          1,0,0, 3.678936f,  
                                          0,-1,0, 1.124000f,  
                                          0,0,0,1).finished(),
        /* ID 26 */ (Eigen::Matrix4f() << 0,0,-1,  12.549124f,
                                          1,0,0,  4.034536f, 
                                          0,-1,0, 1.124000f,  
                                          0,0,0,1).finished(),
        /* ID 27 */ (Eigen::Matrix4f() << -1,0,0, 12.270994f, 
                                          0,0,-1, 4.631436f, 
                                          0,-1,0, 1.124000f,  
                                          0,0,0,1).finished(),
        /* ID 28 */ (Eigen::Matrix4f() << 0,0,-1,  11.915394f, 
                                          1,0,0, 7.436104f,  
                                          0,-1,0, 0.889000f,  
                                          0,0,0,1).finished(),
        /* ID 29 */ (Eigen::Matrix4f() << 0,0,1, 16.513048f,
                                          -1,0,0,  7.416292f, 
                                          0,-1,0, 0.552450f, 
                                          0,0,0,1).finished(),
        /* ID 30 */ (Eigen::Matrix4f() << 0,0,1, 16.513048f,
                                          -1,0,0,  6.984492f, 
                                          0,-1,0, 0.552450f, 
                                          0,0,0,1).finished(),
        /* ID 31 */ (Eigen::Matrix4f() << 0,0,1, 16.513048f, 
                                          -1,0,0,  4.034536f, 
                                          0,-1,0, 0.552450f, 
                                          0,0,0,1).finished(),
        /* ID 32 */ (Eigen::Matrix4f() << 0,0,1, 16.513048f, 
                                          -1,0,0,  3.602736f, 
                                          0,-1,0, 0.552450f, 
                                          0,0,0,1).finished()
    }};
}
