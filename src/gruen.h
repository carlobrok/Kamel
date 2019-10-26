#ifndef GRUEN_H
#define GRUEN_H

#include <opencv2/opencv.hpp>

void set_gruen_data(cv::Point & grcenter, int & grstate);
void set_gruen_data(cv::Point grcenter, int grstate);
void get_gruen_data(cv::Point & grcenter, int & grstate);

cv::Point gruen_center(std::vector <cv::Point> & cont1, std::vector <cv::Point> & cont2);
cv::Point gruen_center(cv::Point & p1, cv::Point & p2);
cv::Point gruen_center(std::vector <cv::Point> & contour);

cv::Point rotate_point(cv::Point & origin_point, cv::Point & rotating_point, float rotation, float length_factor);
cv::Point rotated_point_lenght(cv::Point & origin_point, float rotation, float length);

int gruen_check_normal(cv::Mat & img_rgb, cv::Mat & bin_sw, cv::Mat & bin_gr, std::vector<cv::Point> & contour);

void separate_gruen(cv::Mat & hsv, cv::Mat & bin_gr);
void gruen_calc(cv::Mat & img_rgb, cv::Mat & img_hsv, cv::Mat & bin_sw, cv::Mat & bin_gr);
float ratio_black_points(cv::Point & origin, cv::Point & destination, cv::Mat & bin_sw, cv::Mat & bin_gr, cv::Mat & img_rgb);


#endif
