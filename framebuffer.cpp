#include "framebuffer.h"
#include <iostream>
#include <fcntl.h>
#include <sys/mman.h>
#include <linux/fb.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstring>

constexpr int MAX_QUEUE_SIZE = 1; // Max frames to buffer

Framebuffer::Framebuffer() : 
    fbfd(-1), 
    fbp(nullptr), 
    vinfo(new fb_var_screeninfo), 
    finfo(new fb_fix_screeninfo), 
    screen_size(0), 
    stop_thread(false),
    width(0),
    height(0),
    bits_per_pixel(0) 
{
    memset(vinfo, 0, sizeof(fb_var_screeninfo));
    memset(finfo, 0, sizeof(fb_fix_screeninfo));
}

Framebuffer::~Framebuffer() {
    stop();
    
    if (fbp) {
        munmap(fbp, screen_size);
        fbp = nullptr;
    }
    if (fbfd != -1) {
        close(fbfd);
        fbfd = -1;
    }
    
    delete vinfo;
    delete finfo;
}

bool Framebuffer::init() {
    fbfd = open("/dev/fb0", O_RDWR);
    if (fbfd == -1) {
        std::cerr << "Error: cannot open framebuffer device" << std::endl;
        return false;
    }

    if (ioctl(fbfd, FBIOGET_FSCREENINFO, finfo) == -1) {
        std::cerr << "Error reading fixed information" << std::endl;
        close(fbfd);
        fbfd = -1;
        return false;
    }

    if (ioctl(fbfd, FBIOGET_VSCREENINFO, vinfo) == -1) {
        std::cerr << "Error reading variable information" << std::endl;
        close(fbfd);
        fbfd = -1;
        return false;
    }

    width = vinfo->xres;
    height = vinfo->yres;
    bits_per_pixel = vinfo->bits_per_pixel;
    screen_size = width * height * bits_per_pixel / 8;
    
    fbp = (char*)mmap(0, screen_size, PROT_READ | PROT_WRITE, MAP_SHARED, fbfd, 0);
    
    if ((long)fbp == -1) {
        std::cerr << "Error: failed to map framebuffer device to memory" << std::endl;
        close(fbfd);
        fbfd = -1;
        return false;
    }

    std::cout << "Framebuffer initialized: " << width << "x" << height 
              << " (" << bits_per_pixel << " bpp)" << std::endl;
    
    return true;
}

void Framebuffer::startDisplayThread() {
    stop_thread = false;
    display_thread = std::thread(&Framebuffer::displayThreadFunc, this);
    bind_thread_to_core(display_thread, 7);
}

void Framebuffer::pushFrame(const cv::Mat& frame) {
    std::unique_lock<std::mutex> lock(queue_mutex);
    // Drop oldest frame if queue is full
    if (frame_queue.size() >= MAX_QUEUE_SIZE) {
        frame_queue.pop();
    }
    frame_queue.push(frame.clone()); // Clone to ensure data ownership
    lock.unlock();
    queue_cond.notify_one();
}

void Framebuffer::stop() {
    {
        std::unique_lock<std::mutex> lock(queue_mutex);
        stop_thread = true;
    }
    queue_cond.notify_all();
    if (display_thread.joinable()) {
        display_thread.join();
    }
}

int Framebuffer::getWidth() const {
    return width;
}

int Framebuffer::getHeight() const {
    return height;
}

int Framebuffer::getBpp() const {
    return bits_per_pixel;
}

void Framebuffer::displayThreadFunc() {
    const int fb_line_length = finfo->line_length;
    const int fb_xoffset = vinfo->xoffset;
    const int fb_yoffset = vinfo->yoffset;
    
    // Create a Mat that directly maps to framebuffer memory for supported formats
    cv::Mat fb_mat;
    if (bits_per_pixel == 32) {
        fb_mat = cv::Mat(height, width, CV_8UC4, fbp, fb_line_length);
    } else if (bits_per_pixel == 24) {
        fb_mat = cv::Mat(height, width, CV_8UC3, fbp, fb_line_length);
    }
    
    while (true) {
        cv::Mat frame;
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            queue_cond.wait(lock, [this] { 
                return !frame_queue.empty() || stop_thread; 
            });
            
            if (stop_thread) break;
            
            frame = frame_queue.front();
            frame_queue.pop();
        }
        
        if (!fbp || frame.empty()) continue;

        // Use OpenCV's optimized path for 32bpp and 24bpp
        if ((bits_per_pixel == 32 || bits_per_pixel == 24) && !fb_mat.empty()) {
            cv::Mat processed_frame;
            
            // Resize if needed
            if (frame.size() != fb_mat.size()) {
                cv::resize(frame, processed_frame, fb_mat.size());
            } else {
                processed_frame = frame;
            }
            
            // Convert color space
            if (bits_per_pixel == 32) {
                // Convert BGR to BGRA (adding alpha channel)
                cv::cvtColor(processed_frame, processed_frame, cv::COLOR_BGR2BGRA);
            } else if (bits_per_pixel == 24) {
                // Convert BGR to RGB for 24bpp
                cv::cvtColor(processed_frame, processed_frame, cv::COLOR_BGR2RGB);
            }
            
            // Copy to framebuffer using OpenCV's optimized memcpy
            processed_frame.copyTo(fb_mat);
        }
        else if (bits_per_pixel == 16) {
            // Optimized manual implementation for 16bpp (RGB565)
            cv::Mat rgb_frame;
            
            // Resize and convert to RGB
            if (frame.cols != width || frame.rows != height) {
                cv::resize(frame, rgb_frame, cv::Size(width, height));
                cv::cvtColor(rgb_frame, rgb_frame, cv::COLOR_BGR2RGB);
            } else {
                cv::cvtColor(frame, rgb_frame, cv::COLOR_BGR2RGB);
            }
            
            const uchar* frame_data = rgb_frame.data;
            const int frame_step = rgb_frame.step;
            
            // Optimized 16bpp copy
            for (int y = 0; y < height; y++) {
                const uchar* src_row = frame_data + y * frame_step;
                char* dst_row = fbp + (y + fb_yoffset) * fb_line_length;
                
                for (int x = 0; x < width; x++) {
                    const uchar* src_pixel = src_row + x * 3;
                    unsigned short* dst_pixel = 
                        reinterpret_cast<unsigned short*>(dst_row + (x + fb_xoffset) * 2);
                    
                    // Convert RGB888 to RGB565
                    unsigned short b = src_pixel[2] >> 3;
                    unsigned short g = src_pixel[1] >> 2;
                    unsigned short r = src_pixel[0] >> 3;
                    *dst_pixel = (r << 11) | (g << 5) | b;
                }
            }
        }
        else if (bits_per_pixel == 8) {
            // Handle 8bpp grayscale
            cv::Mat gray_frame;
            
            if (frame.cols != width || frame.rows != height) {
                cv::resize(frame, gray_frame, cv::Size(width, height));
            } else {
                gray_frame = frame;
            }
            
            // Convert to grayscale if not already
            if (gray_frame.channels() > 1) {
                cv::cvtColor(gray_frame, gray_frame, cv::COLOR_BGR2GRAY);
            }
            
            const uchar* frame_data = gray_frame.data;
            const int frame_step = gray_frame.step;
            
            // Direct copy for 8bpp
            for (int y = 0; y < height; y++) {
                const uchar* src_row = frame_data + y * frame_step;
                char* dst_row = fbp + (y + fb_yoffset) * fb_line_length;
                
                for (int x = 0; x < width; x++) {
                    dst_row[x + fb_xoffset] = src_row[x];
                }
            }
        }
        else {
            // Fallback for unsupported formats - convert to 32bpp and use OpenCV
            cv::Mat temp_frame;
            if (frame.cols != width || frame.rows != height) {
                cv::resize(frame, temp_frame, cv::Size(width, height));
            } else {
                temp_frame = frame;
            }
            
            cv::cvtColor(temp_frame, temp_frame, cv::COLOR_BGR2BGRA);
            
            // Manual copy for unsupported formats
            const uchar* src_data = temp_frame.data;
            const int src_step = temp_frame.step;
            const int bytes_pp = bits_per_pixel / 8;
            
            for (int y = 0; y < height; y++) {
                const uchar* src_row = src_data + y * src_step;
                char* dst_row = fbp + (y + fb_yoffset) * fb_line_length;
                
                for (int x = 0; x < width; x++) {
                    char* dst_pixel = dst_row + (x + fb_xoffset) * bytes_pp;
                    const uchar* src_pixel = src_row + x * 4; // BGRA source
                    
                    // Simple copy - format conversion would be needed here
                    memcpy(dst_pixel, src_pixel, std::min(bytes_pp, 4));
                }
            }
        }
    }
}