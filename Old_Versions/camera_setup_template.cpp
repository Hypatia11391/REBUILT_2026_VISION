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
    std::shared_ptr<Request> req;
    bool ready = false;
};

void cameraThread(Camera *cam, LatestFrame &latest) {
    libcamera::FrameBufferAllocator allocator(cam);

    for (auto &stream : cam->streams())
        allocator.allocate(stream);

    cam->start();

    // Create & queue one request per buffer
    std::vector<std::unique_ptr<Request>> requests;
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
            latest.req.reset(completed, [](Request *) {});
            latest.ready = true;
        }
        latest.cv.notify_one();

        cam->queueRequest(completed);
    }
}

Camera *bindCameraToPort(CameraManager &cm, const std::string &portTag) {
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
    Camera *cam0 = bindCameraToPort(cm, "csi0");
    Camera *cam1 = bindCameraToPort(cm, "csi1");

    if (!cam0 || !cam1) {
        std::cerr << "ERROR: Could not bind both cameras to CSI ports\n";
        return 1;
    }

    // Configure streams
    auto cfg0 = cam0->generateConfiguration({ StreamRole::VideoRecording });
    auto cfg1 = cam1->generateConfiguration({ StreamRole::VideoRecording });

    cfg0->at(0).pixelFormat = formats::YUV420;
    cfg1->at(0).pixelFormat = formats::YUV420;
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

            auto ts = req->metadata().get(Request::BufferMetadata::Timestamp);
            std::cout << "Cam0 timestamp: " << ts << " ns\n";
        }

        {
            std::unique_lock lock(latest1.mtx);
            latest1.cv.wait(lock, [&]{ return latest1.ready; });
            auto req = latest1.req;
            latest1.ready = false;

            auto ts = req->metadata().get(Request::BufferMetadata::Timestamp);
            std::cout << "Cam1 timestamp: " << ts << " ns\n";
        }

        // Your processing happens here (always newest frame only)
    }

    cm.stop();
    return 0;
}