#include "CameraDevice.h"
#include "AprilTagDetector.h"
#include "constants.h"
#include "TagPoseEstimator.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>
#include <chrono>

#include <opencv2/opencv.hpp>

#include "constants.h"
#include "CameraDevice.h"
//#include "AprilTagEstimator.h" // <----- Error. TagPoseEstimator? Fix syntax.

int main() {
    using namespace std::chrono;

    // Create two camera devices
    CameraDevice cam0(0);
    CameraDevice cam1(1);

    if (!cam0.open()) {
        std::cerr << "Failed to open camera 0" << std::endl;
        return 1;
    }

    if (!cam1.open()) {
        std::cerr << "Failed to open camera 1" << std::endl;
        return 1;
    }

    AprilTagDetector detector;

    // Create AprilTag pose estimators
    TagPoseEstimator est0(
        constants::Cameras[0].fx,
        constants::Cameras[0].fy,
        constants::Cameras[0].cx,
        constants::Cameras[0].cy,
        constants::tag_size
    );

    TagPoseEstimator est1( // <---- Changed from AprilTagEstimator due to probably synstax error
        constants::Cameras[1].fx,
        constants::Cameras[1].fy,
        constants::Cameras[1].cx,
        constants::Cameras[1].cy,
        constants::tag_size
    );

    // Processing loop
    while (true) {

        // Initialize frame and timestamp variables
        cv::Mat frame0, frame1;
        steady_clock::time_point ts0, ts1;

        // Grab frames and timestamps
        if (!cam0.read(frame0, ts0)) { // <--- probably sysntax error. Read method undefined? Or is it built in? Probably need waitForFrame.
            std::cerr << "Failed to read frame from camera 0" << std::endl;
            continue;
        }

        if (!cam1.read(frame1, ts1)) { // <--- probably sysntax error. Read method undefined? Or is it built in? Probably need waitForFrame.
            std::cerr << "Failed to read frame from camera 1" << std::endl;
            continue;
        }

        // Detect AprilTags in each frame
        auto detections0 = est0.detect(frame0); // <--- possible sysntax error? detect method not explicitly called by the detector class. Called by the TagPoseEstimator type class?
        auto detections1 = est1.detect(frame1);

        // Estimate poses in CAMERA FRAME
        auto poses0 = est0.estimatePoses(detections0); // <--- possible sysntax error? detect method not explicitly called by the detector class. Called by the TagPoseEstimator type class?
        auto poses1 = est1.estimatePoses(detections1);

        /*// Draw results on images
        est0.drawDetections(frame0, detections0);
        est1.drawDetections(frame1, detections1);*/

        // Print pose results with timestamps
        auto ts0_ms = duration_cast<milliseconds>(ts0.time_since_epoch()).count();
        auto ts1_ms = duration_cast<milliseconds>(ts1.time_since_epoch()).count();

        std::cout << "Camera 0 timestamp(ms): " << ts0_ms << std::endl;
        for (const auto& p : poses0) {
            std::cout << "  Tag " << p.id
                      << " position(cam frame): [" << p.x << ", " << p.y << ", " << p.z << "]"
                      << " orientation(rpy): [" << p.roll << ", " << p.pitch << ", " << p.yaw << "]"
                      << std::endl;
        }

        std::cout << "Camera 1 timestamp(ms): " << ts1_ms << std::endl;
        for (const auto& p : poses1) {
            std::cout << "  Tag " << p.id
                      << " position(cam frame): [" << p.x << ", " << p.y << ", " << p.z << "]"
                      << " orientation(rpy): [" << p.roll << ", " << p.pitch << ", " << p.yaw << "]"
                      << std::endl;
        }

        // Show windows
        cv::imshow("Camera 0", frame0);
        cv::imshow("Camera 1", frame1);

        // Exit when user presses 'q'
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    cam0.close();
    cam1.close();

    return 0;
}
