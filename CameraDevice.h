# pragma once

# include <opencv2/core.hpp>
# include <memory>
# include "Frame.h"

namespace camera {

class CameraDevice{
public:
    CameraDevice(int cameraIndex, int width, int height);
    ~CameraDevice();

    void start();
    void stop();

    // Return the latest frame (thread safe)
    Frame waitForFrame();

private:
    struct Impl;
    Impl* impl_;
};

}