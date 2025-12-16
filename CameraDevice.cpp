# include "CameraDevice.h"

# include <libcamera/libcamera.h>
# include <mutex>

using namespace libcamera;

namespace camera {

struct CameraDevice::Impl {
    std::mutex frameMutex;
    std::condition_variable cv;

    Frame latestFrame;
    bool newFrameAvailable = false;
};

static void requestComplete(Request* request) {
    if (request->status() != Request::RequestComplete)
        return;

    auto ts = request->metadata().get(controls::SensorTimestamp);
    if (!ts)
        return;

    std::chrono::nanoseconds timestamp{*ts};

    {
        std::lock_guard lock(impl->mutex);
        impl->latestFrame.image = image;
        impl->latestFrame.timestamp = timestamp;
        impl->newFrameAvailable = true;
    }
    impl->cv.notify_one();
}

Frame CameraDevice::waitForFrame()
{
    std::unique_lock lock(impl->mutex);
    impl->cv.wait(lock, [&] {return impl_->newFrameAvailable; });

    impl_->newFrameAvailable = false;
    return impl_->latestFrame;
}

}