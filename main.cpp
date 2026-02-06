#include <apriltag/apriltag_pose.h>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp>

#include <atomic>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <chrono>
#include <vector>
#include <cstring>
#include <cstdint>

#include "header.h"

struct RobotPoseEstimate {
    std::optional<int64_t> timestamp;
    double err;
    Eigen::Matrix4f pose;
};

Eigen::Matrix4f poseAprilTagToEigen(const apriltag_pose_t& pose) {
    Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();

    // Map the 3x3 rotation matrix (double to float)
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> R(pose.R->data);
    mat.block<3, 3>(0, 0) = R.cast<float>();

    // Map the 3x1 translation vector (double to float)
    Eigen::Map<Eigen::Matrix<double, 3, 1>> t(pose.t->data);
    mat.block<3, 1>(0, 3) = t.cast<float>();

    return mat;
}

inline uint64_t extract_timestamp_and_ptr(const cv::Mat& frame, uint8_t*& gray_out) {
    const uint8_t* data = frame.data;

    uint64_t ts_ns;
    std::memcpy(&ts_ns, data, sizeof(uint64_t));

    gray_out = const_cast<uint8_t*>(data + sizeof(uint64_t)); // skip 8 bytes
    return ts_ns;
}

int main () {
    cv::VideoCapture cap0("/dev/video-luckfox-front", cv::CAP_V4L2);
    cv::VideoCapture cap1("/dev/video-luckfox-back", cv::CAP_V4L2);

    cap0.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap0.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
    cap0.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y','8','0','0'));

    cap1.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap1.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
    cap1.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y','8','0','0'));
    
    if (!cap0.isOpened()) {
        std::cerr << "Failed to open front camera\n";
        return -1;
    }

    if (!cap1.isOpened()) {
        std::cerr << "Failed to open back camera\n";
        return -1;
    }

    apriltag_family_t *tf = tag36h11_create();
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);

    apriltag_detection_info_t info0;
    info0.tagsize = constants::tagsize;
    info0.fx = constants::Cameras[0].fx;
    info0.fy = constants::Cameras[0].fy;
    info0.cx = constants::Cameras[0].cx;
    info0.cy = constants::Cameras[0].cy;

    apriltag_detection_info_t info1;
    info1.tagsize = constants::tagsize;
    info1.fx = constants::Cameras[1].fx;
    info1.fy = constants::Cameras[1].fy;
    info1.cx = constants::Cameras[1].cx;
    info1.cy = constants::Cameras[1].cy;

    std::vector<RobotPoseEstimate> poseEstimates;
    RobotPoseEstimate current_estimate0;
    RobotPoseEstimate current_estimate1;

while (true) {
    cv::Mat frame0, frame1;

    cap0 >> frame0;
    cap1 >> frame1;

    if (frame0.empty() || frame1.empty()) {
        std::cerr << "Frame grab failed\n";
        continue;
    }

    // Extract timestamps and grayscale pointers
    uint8_t* gray0;
    uint8_t* gray1;

    current_estimate0.timestamp = extract_timestamp_and_ptr(frame0, gray0);
    current_estimate1.timestamp = extract_timestamp_and_ptr(frame1, gray1);

    image_u8_t im0{};
    im0.width  = frame0.cols;
    im0.height = frame0.rows;
    im0.stride = frame0.cols; // Y8 = 1 byte per pixel
    im0.buf    = gray0;

    image_u8_t im1{};
    im1.width  = frame1.cols;
    im1.height = frame1.rows;
    im1.stride = frame1.cols;
    im1.buf    = gray1;

    zarray_t *detections0 = apriltag_detector_detect(td, &im0);
    zarray_t *detections1 = apriltag_detector_detect(td, &im1);

    for (int i = 0; i < zarray_size(detections0); i++) {
        apriltag_detection_t *det;
        zarray_get(detections0, i, &det);


        info0.det = det;

        apriltag_pose_t pose;
        double err = estimate_tag_pose(&info0, &pose);

        Eigen::Matrix4f tagPoseInCamera = poseAprilTagToEigen(pose);
        Eigen::Matrix4f cameraPoseInTag = tagPoseInCamera.inverse();

        Eigen::Matrix4f robotPoseInGlobal = constants::AprilTagPosesInGlobal[det.id-1] * cameraPoseInTag * constants::Cameras[0].RobotPoseInCamera;

        current_estimate0.err = err;
        current_estimate0.pose = robotPoseInGlobal;

        poseEstimates.push_back(current_estimate);
    }

    for (int i = 0; i < zarray_size(detections1); i++) {
        apriltag_detection_t *det;
        zarray_get(detections1, i, &det);

        info1.det = det;

        apriltag_pose_t pose;
        double err = estimate_tag_pose(&info1, &pose);

        Eigen::Matrix4f tagPoseInCamera = poseAprilTagToEigen(pose);
        Eigen::Matrix4f cameraPoseInTag = tagPoseInCamera.inverse();

        Eigen::Matrix4f robotPoseInGlobal = constants::AprilTagPosesInGlobal[det.id-1] * cameraPoseInTag * constants::Cameras[0].RobotPoseInCamera;

        current_estimate1.err = err;
        current_estimate1.pose = robotPoseInGlobal;

        poseEstimates.push_back(current_estimate);
    }

    //std::cout << "Detections: " << zarray_size(detections) << "\n";
    apriltag_detections_destroy(detections0);
    apriltag_detections_destroy(detections1);
    };

}