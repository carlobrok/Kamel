#ifndef UTIL_H
#define UTIL_H

#include <thread>
#include <chrono>
#include <opencv2/opencv.hpp>

inline void log_timing(int64 & tlast, const char* message) {
	std::cout << message << (cv::getTickCount() - tlast) / cv::getTickFrequency() * 1000.0 << " ms" << std::endl;
	tlast = cv::getTickCount();
}

bool inMat(cv::Point & p, int w, int h);
bool inMat(cv::Point & p, cv::Mat & img);

inline void thread_delay(int64_t ms) {
	std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

inline void thread_delay_micros(int64_t us) {
	std::this_thread::sleep_for(std::chrono::microseconds(us));
}

void init_clock(void);
double_t cur_ms(void);

#endif
