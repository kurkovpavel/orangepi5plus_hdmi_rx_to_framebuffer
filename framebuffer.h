#ifndef FRAMEBUFFER_H
#define FRAMEBUFFER_H

#include <opencv2/opencv.hpp>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <cstdint>

// Forward declarations for Linux framebuffer structures
struct fb_var_screeninfo;
struct fb_fix_screeninfo;

class Framebuffer {
public:
    Framebuffer();
    ~Framebuffer();
    
    bool init();
    void startDisplayThread();
    void pushFrame(const cv::Mat& frame);
    void stop();

    // Get framebuffer dimensions
    int getWidth() const;
    int getHeight() const;
    int getBpp() const;

private:
    void displayThreadFunc();

    int fbfd;
    char* fbp;
    fb_var_screeninfo* vinfo;
    fb_fix_screeninfo* finfo;
    long int screen_size;
    
    // Threading components
    std::thread display_thread;
    std::queue<cv::Mat> frame_queue;
    std::mutex queue_mutex;
    std::condition_variable queue_cond;
    std::atomic<bool> stop_thread;

    // Store dimensions for easy access
    int width;
    int height;
    int bits_per_pixel;
    void bind_thread_to_core(std::thread& thread, int core_id) {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(core_id, &cpuset);
        
        pthread_t native_handle = thread.native_handle();
        if (pthread_setaffinity_np(native_handle, sizeof(cpu_set_t), &cpuset) != 0) {
            // Handle error (optional)
            perror("Failed to set thread affinity");
        }
    }    
};

#endif // FRAMEBUFFER_H