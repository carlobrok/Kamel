#include "util.h"

bool inMat(cv::Point p, int cols, int rows) {
	if (p.x >= 0 && p.x < cols && p.y >= 0 && p.y < rows)
	{
		return true;
	} else {
		return false;
	}
}
