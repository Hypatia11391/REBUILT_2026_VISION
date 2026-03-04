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

#include <cstring>
#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <algorithm>
#include <sys/mman.h>
#include <chrono>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "header.h"

using namespace libcamera;

class VisualCameraProcessor {
    struct RobotPoseEstimate {
        std::optional<uint64_t> timestamp;
        double err_translation;
        double err_rotation;
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
    VisualCameraProcessor(std::shared_ptr<Camera> cam,
        int id,
        apriltag_detector_t* td,
        std::vector<RobotPoseEstimate> globalPoseEstimates,
        std::mutex globalPoseEstimateMutex
    )
        : camera_(cam), id_(id), td_(td), stream_(nullptr) globalPoseEstimates_(globalPoseEstimates) globalPoseEstimateMutex_(globalPoseEstimateMutex) {}

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
        uint64_t timestamp = static_cast<uint64_t>(request->metadata().get(controls::SensorTimestamp).value());

        cv::Mat gray(1088, 1456, CV_8UC1, data, stride_);
        
        cv::Mat thresholded;

        cv::threshold(gray, thresholded, 100.0, 255.0, cv::THRESH_BINARY);

        image_u8_t im{
            .width = thresholded.cols,
            .height = thresholded.rows,
            .stride = (int)thresholded.step, 
            .buf = thresholded.data
        };

        /*//zarray_t *detections = apriltag_detector_detect(td_, &im);

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
        }*/

        std::vector<RobotPoseEstimate> poseEstimates = processDetections(&im, timestamp);

        {
            std::unique_lock lock(globalPoseEstimateMutex_);
            globalPoseEstimates_.insert(globalPoseEstimates_.end(),poseEstimates.start(), poseEstimates.end());
        }

        /*// 4. Show the frame in a window named after the Camera ID
        cv::imshow("Camera " + std::to_string(id_), visual);
        cv::waitKey(1); // Required for HighGUI to refresh the window
        apriltag_detections_destroy(detections);*/

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

        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);

            if (det->id < 1 || det->id > 32) continue;

            info.det = det;
            apriltag_pose_t pose;
            double err = estimate_tag_pose(&info, &pose);

            Eigen::Matrix4f tagPoseInCamera = poseAprilTagToEigen(pose);
            //std::cout << tagPoseInCamera << std::endl;
            Eigen::Matrix4f cameraPoseInTag = tagPoseInCamera.inverse();
            Eigen::Matrix4f robotPoseInGlobal = constants::AprilTagPosesInGlobal[det->id - 1] * cameraPoseInTag * constants::Cameras[id_].RobotPoseInCamera;

            double range = std::sqrt(tagPoseInCamera(0, 3)*tagPoseInCamera(0, 3)
                                     + tagPoseInCamera(1, 3)*tagPoseInCamera(1, 3)
                                     + tagPoseInCamera(2, 3)*tagPoseInCamera(2, 3));

            current_estimate.err_translation = err * range; // Note apriltag_pose_t err measurements are to variable to be useful. Use static proportionality const.
            current_estimate.err_rotation = err;
            current_estimate.pose = robotPoseInGlobal;
            current_estimate.timestamp = ts;

            poseEstimates.push_back(current_estimate);
            
            std::lock_guard<std::mutex> lock(output_mutex);
            //DEBUG print
            std::cout << "@ time t = " << static_cast<int64_t>(current_estimate.timestamp.value_or(0)) << ", cam " << id_ << " | Tag " << det->id << " detected Global Pose:\n" << robotPoseInGlobal << std::endl;

            //std::cout << "Range: " << range << std::endl;
            std::cout << "Err: " << current_estimate.err << "\n" << std::endl;
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
    std::vector<RobotPoseEstimate>& globalPoseEstimates_;
    std::mutex& globalPoseEstimateMutex_;
    static std::mutex output_mutex;
};

std::mutex VisualCameraProcessor::output_mutex;

std::vector<RobotPoseEstimate> globalPoseEstimates;
std::mutex globalPoseEstimateMutex;

std::atomic<bool> isRunning = true;


void netThread(std::vector<RobotPoseEstimate>& globalPoseEstimates,std::mutex& globalPoseEstimateMutex) {
    int socket = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in serverAddress;
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(8080);
    serverAddress.sin_addr.s_addr = INADDR_ANY;

    connect(socket, (struct sockaddr*)&serverAddress, sizeof(serverAddress));


    while (isRunning) {
        while (true) {
            unique_lock lock(globalPoseEstimateMutex);
            if (globalPoseEstimates.size() > 0) {
                break;
            }
        }
        unique_lock lock(globalPoseEstimateMutex);
        for (const RobotPoseEstimate& poseEstimate : globalPoseEstimates) {
            struct structData {
                double matrix[16],
                double translationErr,
                double rotationErr,
                uint64_t timestamp
            } posePacketStruct;

            std::vector<double> vec(poseEstimate.pose.size());
            Eigen::Map<Eigen::MatrixXf>(vec.data(), poseEstimate.pose.rows(), poseEstimate.pose.cols()) = poseEstimate.pose;

            rawPosePacket.matrix = vec.data();
            rawPosePacket.translationErr = poseEstimate.err_translation;
            rawPosePacket.rotationErr = poseEstimate.err_rotation;
            rawPosePacket.timestamp = staticposeEstimate.timestamp.value();

            union{
                structData structPacket;
                unsigned char raw[sizeof(double) * 16 + sizeof(double) * 2 + sizeof(uint64_t)];
            } posePacket;

            posePacket.structPacket = posePacketStruct;

            sendFull(
                clientSocket,
                posePacket.raw,
                sizeof(double) * 16 + sizeof(double) * 2 + sizeof(uint64_t)
            );
        }
    }

    close(socket);
}

bool sendFull(int fd, char* message, int size) {
    // prepare to send request
    const char* ptr = request.c_str();
    int nleft = request.length();
    int nwritten;
    // loop to be sure it is all sent
    while (nleft) {
        if ((nwritten = send(fd, ptr, nleft, 0)) < 0) {
            if (errno == EINTR) {
                // the socket call was interrupted -- try again
                continue;
            } else {
                // an error occurred, so break out
                perror("write");
                return false;
            }
        } else if (nwritten == 0) {
            // the socket is closed
            return false;
        }
        nleft -= nwritten;
        ptr += nwritten;
    }
    return true;
}

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
    td->nthreads = 2;   // may need to adjust this
    td->refine_edges = 1;

    VisualCameraProcessor Cam0(cameras[0], 0, td);
    Cam0.run();

    VisualCameraProcessor Cam1(cameras[1], 1, td);
    Cam1.run();

    std::thread networkThread(netThread, &globalPoseEstimates,&globalPoseEstimateMutex);
    networkThread.detach();

    //std::cout << "Test mode active. Close window or press Ctrl+C to exit.\n";
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    isRunning = false;

    return 0;
}