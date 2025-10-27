#include "camera_hdmi.h"
#include <opencv2/opencv.hpp>
#include "framebuffer.h"
#include <iostream>
#include <memory>

int main() {
    // Create camera with device index 0 and resolution 1920x1080
    auto camera = CameraHDMI::create(0, 1920, 1080);
    if (!camera) {
        std::cerr << "Failed to initialize camera" << std::endl;
        return 1;
    }

    // Start capturing
    if (!camera->start()) {
        std::cerr << "Failed to start camera" << std::endl;
        return 1;
    }

    Framebuffer* global_fb = nullptr;
    Framebuffer fb;
    global_fb = &fb;    

    // Initialize framebuffer
    if (!fb.init()) {
        std::cerr << "Cannot init framebuffer" << std::endl;
        return 1;
    }

    // Start display thread
    fb.startDisplayThread();


    // Capture loop
    while (true) {
        cv::Mat frame;
        if (!camera->capture_frame(frame)) {
            std::cerr << "Failed to capture frame" << std::endl;
            break;
        }

        fb.pushFrame(frame);
        if (cv::waitKey(1) == 'q') break;
    }

    if (global_fb) {
        global_fb->stop();
    }
    return 0;
}