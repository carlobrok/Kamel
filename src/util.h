#ifndef UTIL_H
#define UTIL_H

#include <thread>
#include <chrono>
#include <opencv2/opencv.hpp>


bool inMat(cv::Point & p, int w, int h);
bool inMat(cv::Point & p, cv::Mat & img);

inline void thread_delay(int64_t ms) {
	std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

inline void thread_delay_micros(int64_t us) {
	std::this_thread::sleep_for(std::chrono::microseconds(us));
}

inline std::chrono::steady_clock::time_point get_cur_time() {
	return std::chrono::steady_clock::now();
}

int get_ms_since(std::chrono::steady_clock::time_point t_prev);
int get_ms_diff(std::chrono::steady_clock::time_point t_start, std::chrono::steady_clock::time_point t_stop);
long get_time_since_epoch();

#endif
