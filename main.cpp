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
#include <map>
#include <optional>

#include "header.h"

using namespace libcamera;

class VisualCameraProcessor {
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

    public:
    VisualCameraProcessor(std::shared_ptr<Camera> cam, int id, apriltag_detector_t* td) 
        : camera_(cam), id_(id), td_(td), stream_(nullptr) {}

    void run() {
        if (camera_->acquire()) return;

        std::unique_ptr<CameraConfiguration> config = camera_->generateConfiguration({StreamRole::VideoRecording});
        StreamConfiguration &streamConfig = config->at(0);
        streamConfig.size.width = 1456; 
        streamConfig.size.height = 1088;
        streamConfig.pixelFormat = formats::R10; 
        
        if (config->validate() == CameraConfiguration::Invalid) return;
        camera_->configure(config.get());

        stream_ = streamConfig.stream();
        stride_ = streamConfig.stride;
        allocator_ = new FrameBufferAllocator(camera_);
        allocator_->allocate(stream_);

        // Iterate through buffers and map the TOTAL length of all planes
        for (const std::unique_ptr<FrameBuffer> &buffer : allocator_->buffers(stream_)) {
            // R8 is a single plane, but we map the whole plane length for safety
            size_t length = buffer->planes()[0].length;
            void *memory = mmap(NULL, length, PROT_READ, MAP_SHARED, 
                                buffer->planes()[0].fd.get(), 0);
            
            if (memory == MAP_FAILED) {
                std::cerr << "Fatal: mmap failed" << std::endl;
                continue;
            }
            mappedBuffers_[buffer.get()] = static_cast<uint8_t*>(memory);
                
                std::unique_ptr<Request> request = camera_->createRequest();
                request->addBuffer(stream_, buffer.get());
                requests_.push_back(std::move(request));
            }

            camera_->requestCompleted.connect(this, &VisualCameraProcessor::requestComplete);
            camera_->start();

            for (auto &request : requests_) camera_->queueRequest(request.get());
        }

    void requestComplete(Request *request) {
        if (request->status() == Request::RequestCancelled) return;

        const FrameBuffer *buffer = request->findBuffer(stream_);
        if (!buffer || mappedBuffers_.find(buffer) == mappedBuffers_.end()) return; // Add check

        uint8_t *data = mappedBuffers_[buffer];

        auto metadata = request->metadata().get(controls::SensorTimestamp);
        uint64_t timestamp = metadata ? static_cast<uint64_t>(*metadata) : 0;

        // 1. Create a Grayscale OpenCV Mat from the Y-plane
        cv::Mat gray(1088, 1456, CV_8UC1, data, stride_);

        cv::Mat processed;

        // 2. Thresholding
        cv::threshold(gray, processed, 50.0, 255.0, cv::THRESH_BINARY);

        cv::imshow("Thresholded", processed);

        // 4. Update AprilTag to use the THRESHOLDED data
        image_u8_t im{
                .width = processed.cols,
                .height = processed.rows,
                .stride = (int)processed.step, 
                .buf = processed.data
            };

        // 3. AprilTag Detection
        std::vector<RobotPoseEstimate> pose_estimates = processDetections(&im, timestamp);

        // DEBUG: Print if anything is found at all
        /*if (zarray_size(detections) > 0) {
            std::cout << "Detected " << zarray_size(detections) << " tags!" << std::endl;
        }*/

        request->reuse(Request::ReuseBuffers);
        camera_->queueRequest(request);
    }

private:
    std::vector<RobotPoseEstimate> processDetections(image_u8_t* im, uint64_t ts) {
        std::vector<RobotPoseEstimate> poseEstimates;
        RobotPoseEstimate current_estimate;

        zarray_t *detections = apriltag_detector_detect(td_, im);
        
        apriltag_detection_info_t info;
        info.tagsize = constants::tag_size; 
        info.fx = constants::Cameras[id_].fx;
        info.fy = constants::Cameras[id_].fy;
        info.cx = constants::Cameras[id_].cx;
        info.cy = constants::Cameras[id_].cy;

        //std::cout << zarray_size(detections) << std::endl;

        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);

            if (det->id < 1 || det->id > 32) continue;

            info.det = det;
            apriltag_pose_t pose;
            double err = estimate_tag_pose(&info, &pose);

            Eigen::Matrix4f tagPoseInCamera = poseAprilTagToEigen(pose);
            Eigen::Matrix4f cameraPoseInTag = tagPoseInCamera.inverse();
            Eigen::Matrix4f robotPoseInGlobal = constants::AprilTagPosesInGlobal[det->id-1] * cameraPoseInTag * constants::Cameras[id_].RobotPoseInCamera;

            double range = std::sqrt(tagPoseInCamera(0, 3)*tagPoseInCamera(0, 3)
                                     + tagPoseInCamera(1, 3)*tagPoseInCamera(1, 3)
                                     + tagPoseInCamera(2, 3)*tagPoseInCamera(2, 3));

            current_estimate.err = err * range;
            current_estimate.pose = robotPoseInGlobal;
            current_estimate.timestamp = ts;

            poseEstimates.push_back(current_estimate);
            
            std::lock_guard<std::mutex> lock(output_mutex);
            //DEBUG print
            std::cout << "Cam " << id_ << " | Tag " << det->id << " detected at Global Pose:\n" << robotPoseInGlobal << "\n" << std::endl;
        }
        apriltag_detections_destroy(detections);

        return poseEstimates;
    }
    
    std::shared_ptr<Camera> camera_;
    int id_;
    apriltag_detector_t *td_;
    Stream *stream_;
    unsigned int stride_;
    FrameBufferAllocator *allocator_;
    std::map<const FrameBuffer *, uint8_t *> mappedBuffers_;
    std::vector<std::unique_ptr<Request>> requests_;
    static std::mutex output_mutex;
};

std::mutex VisualCameraProcessor::output_mutex;

int main() {
    std::unique_ptr<CameraManager> cm = std::make_unique<CameraManager>();
    cm->start();

    auto cameras = cm->cameras();
    if (cameras.empty()) return -1;

    apriltag_family_t *tf = tag36h11_create();
    apriltag_detector_t *td0 = apriltag_detector_create();
    apriltag_detector_add_family(td0, tf);

    td0->quad_decimate = 1.0;
    td0->quad_sigma = 0.0;
    td0->nthreads = 2;   // may need to adjust this
    td0->refine_edges = 1;

    apriltag_detector_t *td1 = apriltag_detector_create();
    apriltag_detector_add_family(td1, tf);

    td1->quad_decimate = 1.0;
    td1->quad_sigma = 0.0;
    td1->nthreads = 2;   // may need to adjust this
    td1->refine_edges = 1;

    VisualCameraProcessor Cam0(cameras[0], 0, td0);
    VisualCameraProcessor Cam1(cameras[1], 1, td1);

    Cam0.run();
    Cam1.run();

    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}