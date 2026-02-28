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
#include <map>

#include "header.h"

using namespace libcamera;

struct RobotPoseEstimate {
    std::optional<uint64_t> timestamp;
    double err;
    Eigen::Matrix4f pose;
};

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
        : camera_(cam), id_(id), td_(td), stream_(nullptr) {}

    void run() {
        if (camera_->acquire()) {
            std::cerr << "Failed to acquire camera " << id_ << "\n";
            return;
        }

        std::unique_ptr<CameraConfiguration> config = camera_->generateConfiguration({StreamRole::VideoRecording});
        StreamConfiguration &streamConfig = config->at(0);
        streamConfig.size.width = 1456; 
        streamConfig.size.height = 1088;
        streamConfig.pixelFormat = formats::YUV420; 

        if (config->validate() == CameraConfiguration::Invalid) return;
        camera_->configure(config.get());

        // Store the stream pointer so we can use it in the callback
        stream_ = streamConfig.stream();

        allocator_ = new FrameBufferAllocator(camera_);
        allocator_->allocate(stream_);

        const std::vector<std::unique_ptr<FrameBuffer>> &buffers = allocator_->buffers(stream_);
        for (const std::unique_ptr<FrameBuffer> &buffer : buffers) {
            void *memory = mmap(NULL, buffer->planes()[0].length, PROT_READ, MAP_SHARED, buffer->planes()[0].fd.get(), 0);
            mappedBuffers_[buffer.get()] = static_cast<uint8_t*>(memory);
            
            std::unique_ptr<Request> request = camera_->createRequest();
            request->addBuffer(stream_, buffer.get());
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

        // FIX: Use the stored stream_ pointer instead of calling request->stream()
        const FrameBuffer *buffer = request->findBuffer(stream_);
        if (!buffer) return;

        uint8_t *data = mappedBuffers_[buffer];
        uint64_t timestamp = static_cast<uint64_t>(request->metadata().get(controls::SensorTimestamp));

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
        zarray_t *detections = apriltag_detector_detect(td_, im);
        
        apriltag_detection_info_t info;
        info.tagsize = constants::tag_size; 
        info.fx = constants::Cameras[id_].fx;
        info.fy = constants::Cameras[id_].fy;
        info.cx = constants::Cameras[id_].cx;
        info.cy = constants::Cameras[id_].cy;

        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);

            if (det->id < 1 || det->id > 32) continue;

            info.det = det;
            apriltag_pose_t pose;
            estimate_tag_pose(&info, &pose);

            Eigen::Matrix4f tagPoseInCamera = poseAprilTagToEigen(pose);
            Eigen::Matrix4f cameraPoseInTag = tagPoseInCamera.inverse();
            Eigen::Matrix4f robotPoseInGlobal = constants::AprilTagPosesInGlobal[det->id-1] * cameraPoseInTag * constants::Cameras[id_].RobotPoseInCamera;

            std::lock_guard<std::mutex> lock(output_mutex);
            std::cout << "Cam " << id_ << " | Tag " << det->id << " detected at Global Pose:\n" << robotPoseInGlobal << "\n" << std::endl;
        }
        apriltag_detections_destroy(detections);
    }

    std::shared_ptr<Camera> camera_;
    int id_;
    apriltag_detector_t *td_;
    Stream *stream_; // Added to track the active stream
    FrameBufferAllocator *allocator_;
    std::map<const FrameBuffer *, uint8_t *> mappedBuffers_;
    std::vector<std::unique_ptr<Request>> requests_;
    static std::mutex output_mutex;
};

std::mutex CameraProcessor::output_mutex;

int main() {
    std::unique_ptr<CameraManager> cm = std::make_unique<CameraManager>();
    cm->start();

    std::vector<std::shared_ptr<Camera>> cameras = cm->cameras();
    std::sort(cameras.begin(), cameras.end(), [](auto const &a, auto const &b) {
        return a->id() < b->id(); 
    });

    if (cameras.size() < 2) {
        std::cerr << "Need 2 cameras, found " << cameras.size() << "\n";
        return -1;
    }

    apriltag_family_t *tf = tag36h11_create();
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = 1.0;
    td->nthreads = 4;

    CameraProcessor frontCam(cameras[0], 0, td);
    CameraProcessor backCam(cameras[1], 1, td);

    frontCam.run();
    backCam.run();

    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }

    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);
    return 0;
}