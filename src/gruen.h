#ifndef _GRUEN_H
#define _GRUEN_H

#include <mutex>
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

void set_gruen_data(Point grcenter, int grstate);
void get_gruen_data(Point grcenter, int grstate);

Point gruen_center(vector <Point> cont1, vector <Point> cont2);
Point gruen_center(Point p1, Point p2);
Point gruen_center(vector <Point> contour);

Point rotate_point(Point & origin_point, Point & rotating_point, float rotation, float length_factor);
Point rotated_point_lenght(Point & origin_point, float rotation, float length);

int gruen_check_normal(Mat & img_rgb, Mat & bin_sw, Mat & bin_gr, vector<Point> contour);

void separate_gruen(Mat & hsv, Mat & bin_gr);
void gruen_calc(Mat & img_rgb, Mat & img_hsv, Mat & bin_sw, Mat & bin_gr, int & grstate, Point & grcenter);
float ratio_black_points(Point & origin, Point & destination, Mat & bin_sw, Mat & bin_gr, Mat & img_rgb);


#endif
