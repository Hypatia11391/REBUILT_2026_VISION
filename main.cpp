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
#include <optional>

#include "header.h"

using namespace libcamera;

// Structure to hold a calculated pose and its metadata
struct RobotPoseEstimate {
    std::optional<uint64_t> timestamp;
    double err;
    Eigen::Matrix4f pose;
};

// Helper to convert AprilTag pose to Eigen matrix
Eigen::Matrix4f poseAprilTagToEigen(const apriltag_pose_t& pose) {
    Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> R(pose.R->data);
    mat.block<3, 3>(0, 0) = R.cast<float>();
    Eigen::Map<Eigen::Matrix<double, 3, 1>> t(pose.t->data);
    mat.block<3, 1>(0, 3) = t.cast<float>();
    return mat;
}

class CameraProcessor {
public:
    CameraProcessor(std::shared_ptr<Camera> cam, int id, apriltag_detector_t* td) 
        : camera_(cam), id_(id), td_(td) {}

    void run() {
        if (camera_->acquire()) return;

        // Configure camera for 1456x1088 YUV420
        std::unique_ptr<CameraConfiguration> config = camera_->generateConfiguration({StreamRole::VideoRecording});
        StreamConfiguration &streamConfig = config->at(0);
        streamConfig.size.width = 1456; 
        streamConfig.size.height = 1088;
        streamConfig.pixelFormat = formats::YUV420; 

        if (config->validate() == CameraConfiguration::Invalid) return;
        camera_->configure(config.get());

        // Allocate Buffers
        allocator_ = new FrameBufferAllocator(camera_);
        Stream *stream = streamConfig.stream();
        allocator_->allocate(stream);

        // Map buffers to CPU memory
        const std::vector<std::unique_ptr<FrameBuffer>> &buffers = allocator_->buffers(stream);
        for (const std::unique_ptr<FrameBuffer> &buffer : buffers) {
            void *memory = mmap(NULL, buffer->planes()[0].length, PROT_READ, MAP_SHARED, buffer->planes()[0].fd.get(), 0);
            mappedBuffers_[buffer.get()] = static_cast<uint8_t*>(memory);
            
            std::unique_ptr<Request> request = camera_->createRequest();
            request->addBuffer(stream, buffer.get());
            requests_.push_back(std::move(request));
        }

        camera_->requestCompleted.connect(this, &CameraProcessor::requestComplete);
        camera_->start();

        for (auto &request : requests_) {
            camera_->queueRequest(request.get());
        }
    }

    void requestComplete(Request *request) {
        if (request->status() == Request::RequestCancelled) return;

        const FrameBuffer *buffer = request->findBuffer(request->stream());
        uint8_t *data = mappedBuffers_[buffer];

        // Extract Timestamp from metadata
        uint64_t timestamp = request->metadata().get(controls::SensorTimestamp);

        // Prepare image for AprilTag (using Y-plane/Grayscale)
        image_u8_t im{
            .width = 1456,
            .height = 1088,
            .stride = 1456,
            .buf = data
        };

        processDetections(&im, timestamp);

        request->reuse(Request::ReuseBuffers);
        camera_->queueRequest(request);
    }

private:
    void processDetections(image_u8_t* im, uint64_t ts) {
        // Corrected: Use 'detections' instead of 'detections0'
        zarray_t *detections = apriltag_detector_detect(td_, im);

        apriltag_detection_info_t info;
        info.tagsize = constants::tag_size; 
        info.fx = constants::Cameras[id_].fx;
        info.fy = constants::Cameras[id_].fy;
        info.cx = constants::Cameras[id_].cx;
        info.cy = constants::Cameras[id_].cy;

        std::vector<RobotPoseEstimate> poseEstimates;

        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);

            // Safety: Check if tag ID is within our known field tags (1-32)
            if (det->id < 1 || det->id > 32) continue;

            info.det = det; // Corrected: info0 -> info
            apriltag_pose_t pose;
            double err = estimate_tag_pose(&info, &pose);

            Eigen::Matrix4f tagPoseInCamera = poseAprilTagToEigen(pose);
            Eigen::Matrix4f cameraPoseInTag = tagPoseInCamera.inverse();

            // Calculate global pose using the camera-specific offset
            Eigen::Matrix4f robotPoseInGlobal = constants::AprilTagPosesInGlobal[det->id-1] * cameraPoseInTag * constants::Cameras[id_].RobotPoseInCamera;

            RobotPoseEstimate current_estimate; // Corrected: current_estimate0 -> current_estimate
            current_estimate.timestamp = ts;
            current_estimate.err = err;
            current_estimate.pose = robotPoseInGlobal;
            
            poseEstimates.push_back(current_estimate);
            
            // Output detection for debugging
            std::lock_guard<std::mutex> lock(output_mutex);
            std::cout << "Cam " << id_ << " detected Tag " << det->id << " with error " << err << std::endl;
        }
        
        apriltag_detections_destroy(detections);
    }

    std::shared_ptr<Camera> camera_;
    int id_;
    apriltag_detector_t *td_;
    FrameBufferAllocator *allocator_;
    std::map<const FrameBuffer *, uint8_t *> mappedBuffers_;
    std::vector<std::unique_ptr<Request>> requests_;
    static std::mutex output_mutex;
};

std::mutex CameraProcessor::output_mutex;

int main() {
    // 1. Initialize Camera Manager
    std::unique_ptr<CameraManager> cm = std::make_unique<CameraManager>();
    cm->start();

    // 2. Identify and Sort Cameras for Consistency
    std::vector<std::shared_ptr<Camera>> cameras = cm->cameras();
    std::sort(cameras.begin(), cameras.end(), [](auto const &a, auto const &b) {
        return a->id() < b->id(); 
    });

    if (cameras.size() < 2) {
        std::cerr << "Need 2 cameras, found " << cameras.size() << "\n";
        return -1;
    }

    // 3. Setup AprilTag Detector
    apriltag_family_t *tf = tag36h11_create();
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = 1.0;
    td->nthreads = 4;

    // 4. Start Capture
    CameraProcessor frontCam(cameras[0], 0, td);
    CameraProcessor backCam(cameras[1], 1, td);

    frontCam.run();
    backCam.run();

    std::cout << "Vision System Active. Processing frames...\n";

    // 5. Main Control Loop
    while (true) {
        // Corrected spelling: milliseconds
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }

    // Cleanup
    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);
    return 0;
}