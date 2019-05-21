#ifndef _GRUEN_H
#define _GRUEN_H

#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

//Point gruen_center(vector <Point> contour_left, vector <Point> contour_right);
Point gruen_center(vector <Point> contour);

Point rotate_point(Point & origin_point, Point & rotating_point, float rotation, float length_factor);
Point rotated_point_lenght(Point & origin_point, float rotation, float length);

int gruen_check_normal(Mat & img_rgb, Mat & bin_sw, Mat & bin_gr, vector<Point> contour);

void separate_gruen(Mat & hsv, Mat & bin_gr);
int gruen_state(Mat & img_rgb, Mat & img_hsv, Mat & bin_sw, Mat & bin_gr);
float ratio_black_points(Point & origin, Point & destination, Mat & bin_sw, Mat & bin_gr, Mat & img_rgb);


#endif
