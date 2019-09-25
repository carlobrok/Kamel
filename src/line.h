#ifndef LINE_H
#define LINE_H

#include <vector>
#include "opencv2/opencv.hpp"


void set_line_data(std::vector<cv::Point> & prim_line_points, std::vector<cv::Point> & sec_line_points);
void get_line_data(std::vector<cv::Point> & prim_line_points, std::vector<cv::Point> & sec_line_points);

float line_radiant(cv::Point & p, int rows, int cols);

void sepatare_line(cv::Mat & hsv, cv::Mat & bin_sw);

void line_calc(cv::Mat & img_rgb, cv::Mat & hsv, cv::Mat & bin_sw, cv::Mat & bin_gr, bool do_separate_line = false);


#endif
