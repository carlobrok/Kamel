#include "util.h"

bool inMat(cv::Point p, int cols, int rows) {
	if (p.x >= 0 && p.x < cols && p.y >= 0 && p.y < rows)
	{
		return true;
	} else {
		return false;
	}
}

int64_t start_clock;

void init_clock() {
	start_clock = cv::getTickCount();
}

double_t cur_sec() {
	return (cv::getTickCount() - start_clock) / cv::getTickFrequency() * 1.0000;
}

