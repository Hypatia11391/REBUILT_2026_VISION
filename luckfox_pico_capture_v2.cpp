#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cstdint>
#include <ctime>
#include <iostream>
#include <vector>

// Correction: Use /dev/video11 for ISP-processed output on Luckfox Pico
// /dev/video1 is often the raw subdev which won't give you a usable image
const char* CAM_DEV = "/dev/video11"; 
const char* UVC_DEV = "/dev/video0"; // The Gadget node

int main() {
    int cam_fd = open(CAM_DEV, O_RDWR);
    int uvc_fd = open(UVC_DEV, O_RDWR); // Must be O_RDWR for V4L2

    // 1. Set Camera Format (ISG1321 is 1280x800)
    v4l2_format fmt{};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = 1280;
    fmt.fmt.pix.height = 800;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12; // Standard for Luckfox ISP
    ioctl(cam_fd, VIDIOC_S_FMT, &fmt);

    // 2. Request & Map Buffers (Standard V4L2 pattern)
    // ... [Request buffers for both cam_fd and uvc_fd] ...

    while (true) {
        v4l2_buffer buf{};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        
        // Dequeue from Camera
        ioctl(cam_fd, VIDIOC_DQBUF, &buf);

        // --- TIMESTAMP HACK ---
        // Your Pi 5 code expects the first 8 bytes to be a uint64_t timestamp.
        // We overwrite the first 8 pixels of the Y-plane.
        uint64_t ts = get_time_ns(); 
        memcpy(buffer_pointers[buf.index], &ts, sizeof(ts));

        // Write to UVC Gadget
        // If using standard gadget, simple write() works, 
        // but it's better to use VIDIOC_QBUF on uvc_fd
        write(uvc_fd, buffer_pointers[buf.index], buf.bytesused);

        // Re-queue
        ioctl(cam_fd, VIDIOC_QBUF, &buf);
    }
}