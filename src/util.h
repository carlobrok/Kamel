#ifndef _UTIL_H
#define _UTIL_H

#include "opencv2/opencv.hpp"

inline void log_timing(int64 & tlast, const char* message) {
	std::cout << message << (cv::getTickCount() - tlast) / cv::getTickFrequency() * 1000.0 << " ms" << std::endl;
	tlast = cv::getTickCount();
}

void init_sensorlog();

void log_sensordata(std::vector<cv::Point> & line_points, int & grstate, cv::Point & grcenter, cv::Mat & img_output);

bool inMat(cv::Point p, int w, int h);

#endif
