#include <libcamera/libcamera.h>
#include <libcamera/formats.h>
#include <libcamera/control_ids.h>
#include <libcamera/property_ids.h>

#include <apriltag/apriltag_pose.h>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <apriltag/common/zarray.h>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <algorithm>
#include <sys/mman.h>
#include <chrono>

#include "header.h"

using namespace libcamera;

// Structure to hold a calculated pose and its metadata
struct RobotPoseEstimate {
    uint64_t timestamp;
    double err;
    Eigen::Matrix4f pose;
};

// Helper: Converts AprilTag's pose format to an Eigen Matrix
Eigen::Matrix4f poseAprilTagToEigen(const apriltag_pose_t& pose) {
    Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
    // Map the 3x3 rotation matrix
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> R(pose.R->data);
    mat.block<3, 3>(0, 0) = R.cast<float>();
    // Map the 3x1 translation vector
    Eigen::Map<Eigen::Matrix<double, 3, 1>> t(pose.t->data);
    mat.block<3, 1>(0, 3) = t.cast<float>();
    return mat;
}

class CameraProcessor {
public:
    CameraProcessor(std::shared_ptr<Camera> camera, int id, apriltag_detector_t* td)
        : camera_(camera), id_(id), td_(td) {}

    // Main processing loop for a single camera
    void run() {
        // Initialize the specific camera constants for this instance
        const auto& cam_params = constants::Cameras[id_];
        
        std::cout << "Starting Camera Processor [" << id_ << "]\n";

        // This is where your capture logic (libcamera) would go.
        // For this example, we assume you are receiving frames in a loop:
        while (running_) {
            // 1. Get Frame from camera (Placeholder for libcamera request)
            // cv::Mat frame = captureFrame(); 

            // 2. Detect Tags
            // detections = apriltag_detector_detect(td_, image_buffer);

            // 3. Process Detections
            std::vector<RobotPoseEstimate> localEstimates;
            
            // Example Processing Loop:
            for (int i = 0; i < zarray_size(detections); i++) {
                apriltag_detection_t *det;
                zarray_get(detections, i, &det);

                // Prepare info for pose estimation using our constants
                apriltag_detection_info_t info;
                info.det = det;
                info.tagsize = constants::tagsize;
                info.fx = cam_params.fx;
                info.fy = cam_params.fy;
                info.cx = cam_params.cx;
                info.cy = cam_params.cy;

                apriltag_pose_t pose;
                double err = estimate_tag_pose(&info, &pose);

                // Calculate Global Robot Pose:
                // RobotInGlobal = TagInGlobal * CameraInTag * RobotInCamera
                Eigen::Matrix4f tagInCamera = poseAprilTagToEigen(pose);
                Eigen::Matrix4f cameraInTag = tagInCamera.inverse();
                Eigen::Matrix4f robotPoseInGlobal = constants::AprilTagPosesInGlobal[det->id-1] * cameraInTag * cam_params.RobotPoseInCamera;

                localEstimates.push_back({0, err, robotPoseInGlobal});
            }

            // Log results safely
            {
                std::lock_guard<std::mutex> lock(output_mutex);
                // std::cout << "Cam " << id_ << " found " << localEstimates.size() << " tags.\n";
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10)); 
        }
    }

    void stop() { running_ = false; }

private:
    std::shared_ptr<Camera> camera_;
    int id_;
    apriltag_detector_t* td_;
    std::atomic<bool> running_{true};
    static std::mutex output_mutex;
};

std::mutex CameraProcessor::output_mutex;

int main() {
    // 1. Initialize libcamera Manager
    std::unique_ptr<CameraManager> cm = std::make_unique<CameraManager>();
    if (cm->start()) {
        std::cerr << "Failed to start camera manager\n";
        return -1;
    }

    // 2. Identify and Sort Cameras
    std::vector<std::shared_ptr<Camera>> cameras = cm->cameras();
    if (cameras.size() < 2) {
        std::cerr << "Found " << cameras.size() << " cameras. Need 2 for front/back config.\n";
        // return -1; // Uncomment for production
    }

    // 3. Setup AprilTag Detector (shared across threads)
    apriltag_family_t *tf = tag36h11_create();
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = 1.0;
    td->nthreads = 4;

    // 4. Initialize Processors (Camera 0 = Front, Camera 1 = Back)
    // We use std::thread to run them in parallel
    std::vector<std::thread> threads;
    
    // Safety check for indices
    int num_to_run = std::min((int)cameras.size(), constants::num_cams);
    
    std::vector<std::unique_ptr<CameraProcessor>> processors;
    for(int i = 0; i < num_to_run; ++i) {
        processors.push_back(std::make_unique<CameraProcessor>(cameras[i], i, td));
        threads.emplace_back(&CameraProcessor::run, processors.back().get());
    }

    std::cout << "Vision System Active. Press Ctrl+C to stop.\n";

    // 5. Main Control Loop
    while (true) {
        // Corrected spelling of 'milliseconds'
        std::this_thread::sleep_for(std::chrono::milliseconds(250)); 
        
        // You can add logic here to aggregate estimates from both cameras
        // or communicate with a NetworkTables/ROS server.
    }

    // Cleanup (usually handled by OS on exit, but good practice)
    for(auto& t : threads) {
        if(t.joinable()) t.join();
    }
    
    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);

    return 0;
}