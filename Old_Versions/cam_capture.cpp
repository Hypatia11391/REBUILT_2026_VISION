#include <libcamera/libcamera.h>
#include <libcamera/formats.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <sys/mman.h>
#include <vector>
#include <string>
#include <filesystem>
#include <thread>

using namespace libcamera;
namespace fs = std::filesystem;

int main() {
    std::string dir = "calibration_images_cam0";
    fs::create_directory(dir);

    std::unique_ptr<CameraManager> cm = std::make_unique<CameraManager>();
    cm->start();

    auto cameras = cm->cameras();
    if (cameras.empty()) return -1;

    // Use the first camera (index 0)
    std::shared_ptr<Camera> camera = cameras[0];
    camera->acquire();

    std::unique_ptr<CameraConfiguration> config = camera->generateConfiguration({StreamRole::VideoRecording});
    StreamConfiguration &streamConfig = config->at(0);
    streamConfig.size.width = 1456;
    streamConfig.size.height = 1088;
    streamConfig.pixelFormat = formats::R8; // Using R8 as we discussed

    config->validate();
    camera->configure(config.get());

    FrameBufferAllocator *allocator = new FrameBufferAllocator(camera);
    allocator->allocate(streamConfig.stream());

    // Map buffers
    std::map<FrameBuffer *, uint8_t *> mappedBuffers;
    for (const std::unique_ptr<FrameBuffer> &buffer : allocator->buffers(streamConfig.stream())) {
        size_t length = buffer->planes()[0].length;
        void *memory = mmap(NULL, length, PROT_READ, MAP_SHARED, buffer->planes()[0].fd.get(), 0);
        mappedBuffers[buffer.get()] = static_cast<uint8_t*>(memory);
    }

    camera->start();

    int captured = 0;
    const int total_needed = 100;

    std::cout << "Starting capture of " << total_needed << " images..." << std::endl;

    while (captured < total_needed) {
        std::unique_ptr<Request> request = camera->createRequest();
        request->addBuffer(streamConfig.stream(), allocator->buffers(streamConfig.stream())[0].get());
        
        // Note: In a real loop, you'd queue all buffers, but for a 
        // simple capture script, we can do them one by one.
        camera->queueRequest(request.get());

        // Wait for a frame (simplified for this script)
        // In a production app use the signal, here we'll just wait a moment
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Use the stride we talked about!
        cv::Mat frame(1088, 1456, CV_8UC1, mappedBuffers[allocator->buffers(streamConfig.stream())[0].get()], streamConfig.stride);
        
        std::string filename = dir + "/img_" + std::to_string(captured) + ".jpg";
        cv::imwrite(filename, frame);
        
        std::cout << "\rSaved: " << filename << std::flush;
        captured++;
    }

    std::cout << "\nFinished! Images saved to /" << dir << std::endl;
    
    camera->stop();
    camera->release();
    cm->stop();

    return 0;
}