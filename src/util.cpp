#include <opencv2/opencv.hpp>
#include <chrono>
#include "util.h"

// Returns true if the given point is between (0,0) and (cols,rows)
bool inMat(cv::Point & p, int cols, int rows) {
	if (p.x >= 0 && p.x < cols && p.y >= 0 && p.y < rows)
		return true;
	else
		return false;
}

// Returns true of the given point is between (0,0) and (img.cols,img.rows)
bool inMat(cv::Point & p, cv::Mat & img) {
	if (p.x >= 0 && p.x < img.cols && p.y >= 0 && p.y < img.rows)
		return true;
	else
		return false;
}

int get_ms_since(std::chrono::steady_clock::time_point t_prev) {
	return std::chrono::duration_cast<std::chrono::milliseconds>(get_cur_time() - t_prev).count();
}

int get_ms_diff(std::chrono::steady_clock::time_point t_start, std::chrono::steady_clock::time_point t_stop) {
	return std::chrono::duration_cast<std::chrono::milliseconds>(t_stop - t_start).count();
}

long get_time_since_epoch() {
	return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
