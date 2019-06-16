#ifndef _LINE_H
#define _LINE_H

#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

//void binarize_line();
//void ellipse_calc();

float line_radiant(Point & p, int rows, int cols);

void sepatare_line(Mat & hsv, Mat & bin_sw);

void line_calc(Mat & img_rgb, Mat & hsv, Mat & bin_sw, Mat & bin_gr, vector<Point> & line_points);


#endif
