#ifndef CAMERA_HDMI_H
#define CAMERA_HDMI_H

#include <string>
#include <memory>
#include <opencv2/core/core.hpp>

class CameraHDMI {
public:
    // Constructor with device index and resolution
    static std::unique_ptr<CameraHDMI> create(int device_index = 0, 
                                            int width = 1920, 
                                            int height = 1080);
    
    ~CameraHDMI();

    // No copying allowed
    CameraHDMI(const CameraHDMI&) = delete;
    CameraHDMI& operator=(const CameraHDMI&) = delete;

    // Frame capture functions
    bool start();
    bool capture_frame(cv::Mat& frame);

private:
    // Private constructor - use create() instead
    CameraHDMI(int fd, int width, int height);

    // Internal implementation details
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

#endif // CAMERA_HDMI_H