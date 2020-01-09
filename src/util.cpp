#include <opencv2/opencv.hpp>
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

int64_t start_clock;

// resets start_clock to current tickcount
void init_clock(void) {
	start_clock = cv::getTickCount();
}

// returns passed time in ms since init_clock()
double_t cur_ms(void) {
	return (cv::getTickCount() - start_clock) / cv::getTickFrequency() * 1000.0;
}
