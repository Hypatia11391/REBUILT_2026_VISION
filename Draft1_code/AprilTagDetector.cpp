# include "AprilTagDetector.h"

# include <aprltag/apriltag.h>
# include <apriltag/tag36h11.h>

struct AprilTagDetector::Impl {
    apriltag_family_t* family;
    apriltag_detector_t* detector;
};

AprilTagDetector::AprilTagDetector() {
    impl_ = new Impl;
    impl_->family = tag36h11_create();
    impl_->detector = apriltag_detector_create();
    
    apriltag_detector_add_family(impl_->detector, impl_->family);

    impl_->detector->quad_decimate = 2.0;
    impl_->detector->quad_sigma = 0.0;
    impl_->detector->nthreads = 2;
    impl_->detector->debug = 0;
    impl_->detector->refine_edges = 1;
}

AprilTagDetector::~AprilTagDetector() {
    apriltag_detector_destroy(impl_->detector);
    tag36h11_destroy(impl_->family);

    delete impl;
}

std::vector<DetectedTag> AprilTagDetector::detect(const cv::Mat& gray) const {
    std::vector<DetectedTag> results;

    image_u8_t img {
        .width = gray.cols,
        .height = gray.rows,
        .stride = gray.cols,
        .buf = const_cast<uint8_t*>(gray.data)
    };

    zarray_t* detections = apriltag_detector_detect(impl_->detector, &img);

    for (int i = 0; i < zarray_size(detections); ++i) {
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);

        DetectedTag tag;
        tag.id = det->id;

        for (int j = 0; j < 4; ++j) {
            tag.corners[j] = {
                static_cast<float>(det->p[j][0]),
                static_cast<float>(det->p[j][1])
            };
        }

        results.push_back(tag);
    }

    apriltag_detections_destroy(detections);
    return results;
}