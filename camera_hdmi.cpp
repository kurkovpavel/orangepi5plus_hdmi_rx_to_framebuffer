#include "camera_hdmi.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <opencv2/opencv.hpp>
#include <memory>
#include <string>
#include <stdexcept>
#include <iostream>

#define VIDEO_MAX_PLANES 8

struct CameraHDMI::Impl {
    int fd;
    int width;
    int height;
    v4l2_format fmt;
    v4l2_dv_timings timings;
    void* buffers[VIDEO_MAX_PLANES];
    v4l2_buffer buf;
    bool streaming;

    Impl(int fd, int width, int height) 
        : fd(fd), width(width), height(height), streaming(false) {
        memset(buffers, 0, sizeof(buffers));
    }

    ~Impl() {
        if (streaming) {
            v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            ioctl(fd, VIDIOC_STREAMOFF, &type);
        }

        for (int i = 0; i < fmt.fmt.pix_mp.num_planes; i++) {
            if (buffers[i] != MAP_FAILED) {
                munmap(buffers[i], buf.m.planes[i].length);
            }
        }

        if (fd >= 0) {
            close(fd);
        }
    }

    bool set_format(__u32 pixfmt) {
        v4l2_format fmt = {};
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        fmt.fmt.pix_mp.pixelformat = pixfmt;
        fmt.fmt.pix_mp.width = width;
        fmt.fmt.pix_mp.height = height;
        fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
        fmt.fmt.pix_mp.num_planes = 1;
        fmt.fmt.pix_mp.plane_fmt[0].bytesperline = 0;
        fmt.fmt.pix_mp.plane_fmt[0].sizeimage = 0;

        if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
            if (ioctl(fd, VIDIOC_G_FMT, &fmt) < 0){
                std::cout << "VIDIOC_S_FMT and VIDIOC_G_FMT are not supported " << std::endl;
            }
        }

        this->fmt = fmt;
        std::cout << "Streaming format: " 
                  << reinterpret_cast<const char*>(&fmt.fmt.pix_mp.pixelformat)
                  << " " << fmt.fmt.pix_mp.width << "x" << fmt.fmt.pix_mp.height 
                  << std::endl;
        return true;
    }

    bool initialize() {
        // Query capabilities
        v4l2_capability cap = {};
        if (ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) {
            perror("VIDIOC_QUERYCAP failed");
            return false;
        }

        std::cout << "Driver: " << cap.driver << "\n"
                  << "Card: " << cap.card << "\n"
                  << "Bus info: " << cap.bus_info << "\n"
                  << "Capabilities: 0x" << std::hex << cap.capabilities << std::dec 
                  << std::endl;

        if (!set_format(V4L2_PIX_FMT_NV12)) {
            return false;
        }

        // Get current format
        memset(&fmt, 0, sizeof(fmt));
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        
        if (ioctl(fd, VIDIOC_G_FMT, &fmt) < 0) {
            perror("Failed to get current format");
            return false;
        }

        std::cout << "Current format: " 
                  << reinterpret_cast<const char*>(&fmt.fmt.pix_mp.pixelformat)
                  << " (" << fmt.fmt.pix_mp.width << "x" << fmt.fmt.pix_mp.height << ")\n"
                  << "Pixel format: 0x" << std::hex << fmt.fmt.pix_mp.pixelformat << std::dec << "\n"
                  << "Field: " << fmt.fmt.pix_mp.field << "\n"
                  << "Colorspace: " << fmt.fmt.pix_mp.colorspace << std::endl;

        // Request buffers
        v4l2_requestbuffers req = {};
        req.count = 1;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        req.memory = V4L2_MEMORY_MMAP;

        if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
            perror("Failed to request buffers");
            return false;
        }

        // Map the buffer
        v4l2_plane planes[VIDEO_MAX_PLANES] = {};
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = 0;
        buf.length = fmt.fmt.pix_mp.num_planes;
        buf.m.planes = planes;

        if (ioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {
            perror("Failed to query buffer");
            return false;
        }

        for (int i = 0; i < fmt.fmt.pix_mp.num_planes; i++) {
            buffers[i] = mmap(nullptr, buf.m.planes[i].length, 
                             PROT_READ | PROT_WRITE, MAP_SHARED,
                             fd, buf.m.planes[i].m.mem_offset);
            
            if (buffers[i] == MAP_FAILED) {
                perror("Failed to mmap buffer");
                for (int j = 0; j < i; j++) {
                    munmap(buffers[j], buf.m.planes[j].length);
                }
                return false;
            }
        }

        // Queue the buffer
        for (unsigned int i = 0; i < req.count; ++i) {
            v4l2_buffer bufq = {};
            v4l2_plane planesq[VIDEO_MAX_PLANES] = {};
            
            bufq.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            bufq.memory = V4L2_MEMORY_MMAP;
            bufq.index = i;
            bufq.length = fmt.fmt.pix_mp.num_planes;
            bufq.m.planes = planesq;

            for (int j = 0; j < fmt.fmt.pix_mp.num_planes; j++) {
                bufq.m.planes[j].bytesused = fmt.fmt.pix_mp.plane_fmt[j].sizeimage;
            }

            if (ioctl(fd, VIDIOC_QBUF, &bufq) < 0) {
                perror("Failed to queue buffer");
                for (int j = 0; j < fmt.fmt.pix_mp.num_planes; j++) {
                    munmap(buffers[j], buf.m.planes[j].length);
                }
                return false;
            }
        }

        return true;
    }
};

std::unique_ptr<CameraHDMI> CameraHDMI::create(int device_index, int width, int height) {
    std::string device = "/dev/video" + std::to_string(device_index);
    int fd = open(device.c_str(), O_RDWR);
    if (fd < 0) {
        perror("Failed to open device");
        return nullptr;
    }

    auto camera = std::unique_ptr<CameraHDMI>(new CameraHDMI(fd, width, height));
    if (!camera->pimpl->initialize()) {
        return nullptr;
    }

    return camera;
}

CameraHDMI::CameraHDMI(int fd, int width, int height) 
    : pimpl(std::make_unique<Impl>(fd, width, height)) {}

CameraHDMI::~CameraHDMI() = default;

bool CameraHDMI::start() {
    if (!pimpl) return false;

    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (ioctl(pimpl->fd, VIDIOC_STREAMON, &type) < 0) {
        perror("Failed to start streaming");
        return false;
    }

    pimpl->streaming = true;
    return true;
}

bool CameraHDMI::capture_frame(cv::Mat& frame) {
    if (!pimpl || !pimpl->streaming) return false;

    // Reset buffer info
    memset(&pimpl->buf, 0, sizeof(pimpl->buf));
    pimpl->buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    pimpl->buf.memory = V4L2_MEMORY_MMAP;
    pimpl->buf.index = 0;
    pimpl->buf.length = pimpl->fmt.fmt.pix_mp.num_planes;
    
    // Allocate aligned memory for planes
    const size_t planes_size = sizeof(v4l2_plane) * pimpl->fmt.fmt.pix_mp.num_planes;
    void* planes_mem = aligned_alloc(alignof(v4l2_plane), planes_size);
    if (!planes_mem) {
        std::cerr << "Failed to allocate aligned memory for planes" << std::endl;
        return false;
    }
    pimpl->buf.m.planes = static_cast<v4l2_plane*>(planes_mem);

    // Dequeue buffer
    if (ioctl(pimpl->fd, VIDIOC_DQBUF, &pimpl->buf) < 0) {
        perror("Failed to dequeue buffer");
        free(planes_mem);
        return false;
    }

    // Process frame based on pixel format
    bool result = false;
    const int width = pimpl->fmt.fmt.pix_mp.width;
    const int height = pimpl->fmt.fmt.pix_mp.height;
    uchar* yuv_data = static_cast<uchar*>(pimpl->buffers[0]);

    switch (pimpl->fmt.fmt.pix_mp.pixelformat) {
        /* ---------- 4:2:0, semi-planar ---------- */
        case V4L2_PIX_FMT_NV12: {
            cv::Mat nv12(height + height/2, width, CV_8UC1, yuv_data);
            cv::cvtColor(nv12, frame, cv::COLOR_YUV2BGR_NV12);
            result = true;
            break;
        }
        case V4L2_PIX_FMT_NV21: {
            cv::Mat nv21(height + height/2, width, CV_8UC1, yuv_data);
            cv::cvtColor(nv21, frame, cv::COLOR_YUV2BGR_NV21);
            result = true;
            break;
        }
        case V4L2_PIX_FMT_NV24: {  // 4:4:4, semi-planar
            cv::Mat nv24(height * 2, width, CV_8UC1, yuv_data);
            cv::cvtColor(nv24, frame, cv::COLOR_YUV2BGR_YV12);  // Approximation
            result = true;
            break;
        }

        /* ---------- 4:2:2, semi-planar ---------- */
        case V4L2_PIX_FMT_NV16: {  // Y plane, interleaved UV
            cv::Mat nv16(height * 2, width, CV_8UC1, yuv_data);
            cv::cvtColor(nv16, frame, cv::COLOR_YUV2BGR_UYVY);  // Approximation
            result = true;
            break;
        }
        case V4L2_PIX_FMT_NV61: {  // Y plane, interleaved VU
            cv::Mat nv61(height * 2, width, CV_8UC1, yuv_data);
            cv::cvtColor(nv61, frame, cv::COLOR_YUV2BGR_YVYU);  // Approximation
            result = true;
            break;
        }

        /* ---------- 4:2:2, packed ---------- */
        case V4L2_PIX_FMT_YUYV: {
            cv::Mat yuyv(height, width, CV_8UC2, yuv_data);
            cv::cvtColor(yuyv, frame, cv::COLOR_YUV2BGR_YUYV);
            result = true;
            break;
        }
        case V4L2_PIX_FMT_YVYU: {
            cv::Mat yvyu(height, width, CV_8UC2, yuv_data);
            cv::cvtColor(yvyu, frame, cv::COLOR_YUV2BGR_YVYU);
            result = true;
            break;
        }
        case V4L2_PIX_FMT_UYVY: {
            cv::Mat uyvy(height, width, CV_8UC2, yuv_data);
            cv::cvtColor(uyvy, frame, cv::COLOR_YUV2BGR_UYVY);
            result = true;
            break;
        }
        case V4L2_PIX_FMT_VYUY: {
            // Convert VYUY to UYVY by swapping components
            cv::Mat vyuy(height, width, CV_8UC2, yuv_data);
            cv::Mat uyvy;
            cv::cvtColor(vyuy, uyvy, cv::COLOR_YUV2BGR_UYVY);  // First convert to BGR
            cv::cvtColor(uyvy, frame, cv::COLOR_BGR2YUV);      // Then back to YUV (UYVY)
            result = true;
            break;
        }

        /* ---------- 4:2:0, planar ---------- */
        case V4L2_PIX_FMT_YUV420: {
            cv::Mat yuv420(height + height/2, width, CV_8UC1, yuv_data);
            cv::cvtColor(yuv420, frame, cv::COLOR_YUV2BGR_I420);
            result = true;
            break;
        }
        case V4L2_PIX_FMT_YVU420: {
            cv::Mat yvu420(height + height/2, width, CV_8UC1, yuv_data);
            cv::cvtColor(yvu420, frame, cv::COLOR_YUV2BGR_YV12);
            result = true;
            break;
        }

        /* ---------- RGB formats ---------- */
        case V4L2_PIX_FMT_RGB24: {
            cv::Mat rgb(height, width, CV_8UC3, yuv_data);
            cv::cvtColor(rgb, frame, cv::COLOR_RGB2BGR);
            result = true;
            break;
        }
        case V4L2_PIX_FMT_BGR24: {
            frame = cv::Mat(height, width, CV_8UC3, yuv_data);
            result = true;
            break;
        }
        case V4L2_PIX_FMT_XRGB32: {
            cv::Mat xrgb(height, width, CV_8UC4, yuv_data);
            cv::cvtColor(xrgb, frame, cv::COLOR_BGRA2BGR);
            result = true;
            break;
        }
        case V4L2_PIX_FMT_XBGR32: {
            cv::Mat xbgr(height, width, CV_8UC4, yuv_data);
            cv::cvtColor(xbgr, frame, cv::COLOR_RGBA2BGR);
            result = true;
            break;
        }

        default: {
            std::cerr << "Unsupported pixel format: 0x" 
                     << std::hex << pimpl->fmt.fmt.pix_mp.pixelformat << std::dec 
                     << std::endl;
            // Try to dump the frame anyway for debugging
            if (pimpl->fmt.fmt.pix_mp.num_planes == 1) {
                frame = cv::Mat(height, width, CV_8UC1, yuv_data);
                result = true;
            }
            break;
        }
    }

    // Requeue buffer
    if (ioctl(pimpl->fd, VIDIOC_QBUF, &pimpl->buf) < 0) {
        perror("Failed to requeue buffer");
    }

    // Free aligned memory
    free(planes_mem);

    return result;

}


