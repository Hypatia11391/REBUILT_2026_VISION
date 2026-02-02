#include <chrono>
#include <vector>

#include "header.h"

#include <libcamera/libcamera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/request.h>

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

    while (true) {
        {
            std::unique_lock lock(latest0.mtx);
            latest0.cv.wait(lock, [&]{ return latest0.ready; });
            auto req = latest0.req;
            latest0.ready = false;

            auto ts = req->metadata().get(libcamera::Request::BufferMetadata::Timestamp);
            std::cout << "Cam0 timestamp: " << ts << " ns\n";
        }

        {
            std::unique_lock lock(latest1.mtx);
            latest1.cv.wait(lock, [&]{ return latest1.ready; });
            auto req = latest1.req;
            latest1.ready = false;

            auto ts = req->metadata().get(libcamera::Request::BufferMetadata::Timestamp);
            std::cout << "Cam1 timestamp: " << ts << " ns\n";
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