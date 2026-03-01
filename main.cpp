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
        streamConfig.pixelFormat = formats::YUV420; 
        camera_->configure(config.get());

        this->stride_ = streamConfig.stride;

        if (config->validate() == CameraConfiguration::Invalid) return;
        camera_->configure(config.get());

        stream_ = streamConfig.stream();
        stride_ = streamConfig.stride;
        allocator_ = new FrameBufferAllocator(camera_);
        allocator_->allocate(stream_);

        for (const std::unique_ptr<FrameBuffer> &buffer : allocator_->buffers(stream_)) {
            void *memory = mmap(NULL, buffer->planes()[0].length, PROT_READ, MAP_SHARED, buffer->planes()[0].fd.get(), 0);
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
        uint8_t *data = mappedBuffers_[buffer];

        // 1. Create a Grayscale OpenCV Mat from the Y-plane
        cv::Mat gray(1088, 1456, CV_8UC1, data, stride_);
        
        // 1. Create a processing Mat
        cv::Mat thresholded;

        // 2. Apply Adaptive Thresholding
        cv::threshold(gray, thresholded, 100.0, 255.0, cv::THRESH_BINARY);

        // 3. (Optional) Show the thresholded view to see what the computer sees
        cv::imshow("Threshold", thresholded);

        // 4. Update AprilTag to use the THRESHOLDED data
        image_u8_t im{
            .width = thresholded.cols,
            .height = thresholded.rows,
            .stride = (int)thresholded.step, 
            .buf = thresholded.data
        };

        //zarray_t *detections = apriltag_detector_detect(td_, &im);

        // 2. Convert to BGR so we can draw colored lines/text
        cv::Mat visual;
        cv::cvtColor(gray, visual, cv::COLOR_GRAY2BGR);

        // 3. AprilTag Detection
        //image_u8_t im{ .width = 1456, .height = 1088, .stride = 1456, .buf = data };
        zarray_t *detections = apriltag_detector_detect(td_, &im);

        // DEBUG: Print if anything is found at all
        if (zarray_size(detections) > 0) {
            std::cout << "Detected " << zarray_size(detections) << " tags!" << std::endl;
        }

        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);

            // DRAWING LOGIC:
            // Draw lines between the 4 corners of the tag
            line(visual, cv::Point(det->p[0][0], det->p[0][1]), cv::Point(det->p[1][0], det->p[1][1]), cv::Scalar(0, 0, 255), 2);
            line(visual, cv::Point(det->p[1][0], det->p[1][1]), cv::Point(det->p[2][0], det->p[2][1]), cv::Scalar(0, 255, 0), 2);
            line(visual, cv::Point(det->p[2][0], det->p[2][1]), cv::Point(det->p[3][0], det->p[3][1]), cv::Scalar(255, 0, 0), 2);
            line(visual, cv::Point(det->p[3][0], det->p[3][1]), cv::Point(det->p[0][0], det->p[0][1]), cv::Scalar(255, 255, 0), 2);

            // Label the Tag ID
            std::string text = "ID: " + std::to_string(det->id);
            cv::putText(visual, text, cv::Point(det->c[0]-5, det->c[1]-5), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
        }

        std::vector<RobotPoseEstimate> poseEstimates = processDetections(&im);

        // 4. Show the frame in a window named after the Camera ID
        cv::imshow("Camera " + std::to_string(id_), visual);
        cv::waitKey(1); // Required for HighGUI to refresh the window

        apriltag_detections_destroy(detections);
        request->reuse(Request::ReuseBuffers);
        camera_->queueRequest(request);
    }

private:
    std::vector<RobotPoseEstimate> processDetections(image_u8_t* im, /*uint64_t ts*/) {
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
            //current_estimate.timestamp = ts;

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
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);

    td->quad_decimate = 1.0;
    td->quad_sigma = 0.0;
    td->nthreads = 4;   // may need to adjust this down
    td->refine_edges = 1;

    // Note: For testing, we run one processor. 
    // OpenCV HighGUI (imshow) works best when called from a single thread 
    // or when waitKey is handled correctly.
    VisualCameraProcessor testCam(cameras[0], 0, td);
    testCam.run();

    std::cout << "Test mode active. Close window or press Ctrl+C to exit.\n";
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}