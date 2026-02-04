#include <chrono>
#include <vector>

#include "header.h"

#include <libcamera/libcamera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/request.h>

#include <apriltag/apriltag_pose.h>

#include <Eigen/Dense>

#include <atomic>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

struct LatestFrame {
    std::mutex mtx;
    std::condition_variable cv;
    std::shared_ptr<libcamera::Request> req;
    bool ready = false;
};

struct RobotPoseEstimate {
    std::optional<int64_t> timestamp;
    double err;
    Eigen::Matrix4f pose;
};

void cameraThread(libcamera::Camera *cam, LatestFrame &latest) {
    libcamera::FrameBufferAllocator allocator(cam);

    for (auto &stream : cam->streams())
        allocator.allocate(stream);

    cam->start();

    // Create & queue one request per buffer
    std::vector<std::unique_ptr<libcamera::Request>> requests;
    for (unsigned i = 0; i < 4; ++i) {
        auto req = cam->createRequest();
        for (auto &[stream, buffers] : allocator.buffers()) {
            req->addBuffer(stream, buffers[i].get());
        }
        cam->queueRequest(req.get());
        requests.push_back(std::move(req));
    }

    while (true) {
        libcamera::Request *completed = cam->waitForRequest();

        if (!completed)
            break;

        {
            std::lock_guard lock(latest.mtx);
            latest.req.reset(completed, [](libcamera::Request *) {});
            latest.ready = true;
        }
        latest.cv.notify_one();

        cam->queueRequest(completed);
    }
}

libcamera::Camera *bindCameraToPort(libcamera::CameraManager &cm, const std::string &portTag) {
    for (auto &cam : cm.cameras()) {
        const std::string &id = cam->id();
        std::cout << "Detected camera ID: " << id << '\n';
        if (id.find(portTag) != std::string::npos)
            return cam.get();
    }
    return nullptr;
}

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

int main() {
    libcamera::CameraManager cm;
    cm.start();

    // Deterministic CSI binding
    libcamera::Camera *cam0 = bindCameraToPort(cm, "csi0");
    libcamera::Camera *cam1 = bindCameraToPort(cm, "csi1");

    if (!cam0 || !cam1) {
        std::cerr << "ERROR: Could not bind both cameras to CSI ports\n";
        return 1;
    }

    // Configure streams
    auto cfg0 = cam0->generateConfiguration({ libcamera::StreamRole::VideoRecording });
    auto cfg1 = cam1->generateConfiguration({ libcamera::StreamRole::VideoRecording });

    cfg0->at(0).pixelFormat = libcamera::formats::Y8; // Check if y8 configures successfully. Also try GREY. If that doesn't work will try Y10, but that is more work elsewere.
    cfg1->at(0).pixelFormat = libcamera::formats::Y8;
    cfg0->at(0).size = {1280, 1080};
    cfg1->at(0).size = {1280, 1080};
    cfg0->at(0).bufferCount = 4;
    cfg1->at(0).bufferCount = 4;

    cam0->configure(cfg0.get());
    cam1->configure(cfg1.get());

    LatestFrame latest0, latest1;

    std::jthread t0(cameraThread, cam0, std::ref(latest0));
    std::jthread t1(cameraThread, cam1, std::ref(latest1));

    apriltag_detector_t *td = apriltag_detector_create();

    apriltag_detection_info_t info0;
    info0.det = det;
    info0.tagsize = constants::tagsize;
    info0.fx = constants::Cameras[0].fx;
    info0.fy = constants::Cameras[0].fy;
    info0.cx = constants::Cameras[0].cx;
    info0.cy = constants::Cameras[0].cy;

    apriltag_detection_info_t info1;
    info1.det = det;
    info1.tagsize = constants::tagsize;
    info1.fx = constants::Cameras[1].fx;
    info1.fy = constants::Cameras[1].fy;
    info1.cx = constants::Cameras[1].cx;
    info1.cy = constants::Cameras[1].cy;

    std::vector<RobotPoseEstimate> poseEstimates;
    RobotPoseEstimate current_estimate;

    while (true) {
        {
            std::unique_lock lock(latest0.mtx);
            latest0.cv.wait(lock, [&]{ return latest0.ready; });
            auto req = latest0.req;
            latest0.ready = false;

            auto ts = req->metadata().get(libcamera::Request::BufferMetadata::Timestamp);
            std::cout << "Cam0 timestamp: " << ts << " ns\n";
            current_estimate.timestamp = ts;

            const auto &buffers = latest0.req->buffers();
            const libcamera::FrameBuffer *buffer = buffers.begin()->second;

            const auto &plane = buffer->planes()[0];
            int fd = plane.fd.get();
            size_t length = plane.length;

            void *mem = mmap(nullptr, length, PROT_READ, MAP_SHARED, fd, 0);
            if (mem == MAP_FAILED)
                return;

            uint8_t *gray = static_cast<uint8_t*>(mem);

            image_u8_t im{};
            im.width  = width;
            im.height = height;
            im.stride = width;     // Y8 = 1 byte per pixel
            im.buf    = gray;

            zarray_t *detections = apriltag_detector_detect(td, im);

            for (int i = 0; i < zarray_size(detections); i++) {
                apriltag_detection_t *det;
                zarray_get(detections, i, &det);

                apriltag_pose_t pose;
                double err = estimate_tag_pose(&info0, &pose);

                Eigen::Matrix4f tagPoseInCamera = poseAprilTagToEigen(&pose);
                Eigen::Matrix4f cameraPoseInTag = tagPoseInCamera.inverse();

                Eigen::Matrix4f robotPoseInGlobal = constants::AprilTagPosesInGlobal[det.id-1] * cameraPoseInTag * constants::Cameras[0].RobotPoseInCamera

                current_estimate.err = err;
                current_estimate.pose = robotPoseInGlobal;

                poseEstimates.push_back(current_estimate);
            }
        }

        {
            std::unique_lock lock(latest1.mtx);
            latest1.cv.wait(lock, [&]{ return latest1.ready; });
            auto req = latest1.req;
            latest1.ready = false;

            auto ts = req->metadata().get(libcamera::Request::BufferMetadata::Timestamp);
            std::cout << "Cam1 timestamp: " << ts << " ns\n";
            current_estimate.timestamp = ts;

            const auto &buffers = latest1.req->buffers();
            const libcamera::FrameBuffer *buffer = buffers.begin()->second;

            const auto &plane = buffer->planes()[0];
            int fd = plane.fd.get();
            size_t length = plane.length;

            void *mem = mmap(nullptr, length, PROT_READ, MAP_SHARED, fd, 0);
            if (mem == MAP_FAILED)
                return;

            uint8_t *gray = static_cast<uint8_t*>(mem);

            image_u8_t im{};
            im.width  = width;
            im.height = height;
            im.stride = width;     // Y8 = 1 byte per pixel
            im.buf    = gray;

            zarray_t *detections = apriltag_detector_detect(td, im);

            for (int i = 0; i < zarray_size(detections); i++) {
                apriltag_detection_t *det;
                zarray_get(detections, i, &det);

                apriltag_pose_t pose;
                double err = estimate_tag_pose(&info1, &pose);

                Eigen::Matrix4f tagPoseInCamera = poseAprilTagToEigen(&pose);
                Eigen::Matrix4f cameraPoseInTag = tagPoseInCamera.inverse();

                Eigen::Matrix4f robotPoseInGlobal = constants::AprilTagPosesInGlobal[det.id-1] * cameraPoseInTag * constants::Cameras[1].RobotPoseInCamera

                current_estimate.err = err;
                current_estimate.pose = robotPoseInGlobal;

                poseEstimates.push_back(current_estimate);
            }

            //Figure out how to send the estimates to the roborio.
        }

        // Read Frame

        // Detect apriltags

        // Estimate poses of AprilTags

        // For each detection estimate the global robot pose and uncertainty
            // Invert tag pose (becomes camera in Tag frame)

            // Convert to global frame (Becomes camera in global)

            // Convert to robot in global

            // Estimate uncertainty
                // Range, uncertainty (pixels), ?etc?
            
            // Append to vector of RobotPose structs "vector<RobotPose> RobotPoses;"

        // Send Robot Poses to RoboRio
    }

    // Clean up threads

    cm.stop();
    return 0;
}