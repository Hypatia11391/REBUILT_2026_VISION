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

inline uint64_t get_time_ns()
{
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return uint64_t(ts.tv_sec) * 1000000000ULL + uint64_t(ts.tv_nsec);
}

int main()
{
    const char* cam_dev = "/dev/video1";   // Sensor on LuckFox
    const char* uvc_dev = "/dev/video0";   // UVC gadget device

    int cam_fd = open(cam_dev, O_RDWR);
    if (cam_fd < 0) { perror("open cam"); return 1; }

    int uvc_fd = open(uvc_dev, O_WRONLY);
    if (uvc_fd < 0) { perror("open uvc"); return 1; }

    // ---- Set capture format ----
    v4l2_format fmt{};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width  = 1280;
    fmt.fmt.pix.height = 800;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    ioctl(cam_fd, VIDIOC_S_FMT, &fmt);

    // ---- Request buffers ----
    v4l2_requestbuffers req{};
    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    ioctl(cam_fd, VIDIOC_REQBUFS, &req);

    std::vector<void*> buffers(req.count);
    std::vector<size_t> sizes(req.count);

    for (int i = 0; i < req.count; ++i)
    {
        v4l2_buffer buf{};
        buf.type = req.type;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        ioctl(cam_fd, VIDIOC_QUERYBUF, &buf);

        buffers[i] = mmap(nullptr, buf.length,
                          PROT_READ | PROT_WRITE, MAP_SHARED,
                          cam_fd, buf.m.offset);
        sizes[i] = buf.length;

        ioctl(cam_fd, VIDIOC_QBUF, &buf);
    }

    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(cam_fd, VIDIOC_STREAMON, &type);

    std::cout << "Streaming from LuckFox → USB..." << std::endl;

    while (true)
    {
        v4l2_buffer buf{};
        buf.type = type;
        buf.memory = V4L2_MEMORY_MMAP;
        ioctl(cam_fd, VIDIOC_DQBUF, &buf);

        uint8_t* frame = static_cast<uint8_t*>(buffers[buf.index]);

        // ---- Embed timestamp into first 8 bytes ----
        uint64_t ts_ns = get_time_ns();
        std::memcpy(frame, &ts_ns, sizeof(ts_ns));

        // ---- Send to UVC gadget ----
        ssize_t written = write(uvc_fd, frame, buf.bytesused);
        if (written < 0) perror("write uvc");

        ioctl(cam_fd, VIDIOC_QBUF, &buf);
    }

    return 0;
}
