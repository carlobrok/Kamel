#ifndef GRUEN_H
#define GRUEN_H

#include "opencv2/opencv.hpp"

void set_gruen_data(cv::Point & grcenter, int & grstate);
void get_gruen_data(cv::Point & grcenter, int & grstate);

cv::Point gruen_center(std::vector <cv::Point> cont1, std::vector <cv::Point> cont2);
cv::Point gruen_center(cv::Point p1, cv::Point p2);
cv::Point gruen_center(std::vector <cv::Point> contour);

cv::Point rotate_point(cv::Point & origin_point, cv::Point & rotating_point, float rotation, float length_factor);
cv::Point rotated_point_lenght(cv::Point & origin_point, float rotation, float length);

int gruen_check_normal(Mat & img_rgb, Mat & bin_sw, Mat & bin_gr, std::vector<cv::Point> contour);

void separate_gruen(Mat & hsv, Mat & bin_gr);
void gruen_calc(Mat & img_rgb, Mat & img_hsv, Mat & bin_sw, Mat & bin_gr, int & grstate, cv::Point & grcenter);
float ratio_black_points(cv::Point & origin, cv::Point & destination, Mat & bin_sw, Mat & bin_gr, Mat & img_rgb);


#endif
