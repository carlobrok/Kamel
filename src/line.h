#ifndef LINE_H
#define LINE_H

#include <vector>
#include <opencv2/opencv.hpp>
#include <cstdio>			// uint.._t

// Parameter zur Erkennung der Linie

#define ELLIPSE_THICKNESS 30		// thickness of the ellipse in pixels
#define ELLIPSE_BAR_HEIGHT 100   // Höhe der Balken links und rechts unter der halben Ellipse
#define ELLIPSE_HEIGHT 75

extern uint8_t THRESH_BLACK;
#define LOW_BLACK cv::Scalar(0, 0, 0)
#define HIGH_BLACK cv::Scalar(180, 255, THRESH_BLACK)



void init_line_ellipse();

void sepatare_line(cv::Mat & hsv, cv::Mat & bin_sw);

void set_line_data(std::vector<cv::Point> & line_points);
void get_line_data(std::vector<cv::Point> & line_points);

void line_calc(cv::Mat & img_rgb, cv::Mat & hsv, cv::Mat & bin_sw, cv::Mat & bin_gr, bool do_separate_line = false);


#endif
