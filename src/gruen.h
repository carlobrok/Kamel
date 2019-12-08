#ifndef GRUEN_H
#define GRUEN_H

#include <opencv2/opencv.hpp>



// Parameter zur Erkennung des Grünpunktes

#define LOWER_GREEN cv::Scalar(50, 60, 15)
#define UPPER_GREEN cv::Scalar(90, 255, 115)

#define GRUEN_NICHT 0
#define GRUEN_BEIDE 3
#define GRUEN_LINKS 1
#define GRUEN_RECHTS 2

#define NUM_ITERATIONS_BLACK_POINTS 30


// Datenübergabe multithreading
void set_gruen_data(cv::Point & grcenter, int & grstate);
void set_gruen_data(cv::Point grcenter, int grstate);
void get_gruen_data(cv::Point & grcenter, int & grstate);

// Mittelpunkt des grünen Punktes erkennen
cv::Point gruen_center(std::vector <cv::Point> & cont1, std::vector <cv::Point> & cont2);
cv::Point gruen_center(cv::Point & p1, cv::Point & p2);
cv::Point gruen_center(std::vector <cv::Point> & contour);

//
cv::Point rotate_point(cv::Point & origin_point, cv::Point & rotating_point, float rotation, float length_factor);
cv::Point rotated_point_lenght(cv::Point & origin_point, float rotation, float length);
int gruen_check_normal(cv::Mat & img_rgb, cv::Mat & bin_sw, cv::Mat & bin_gr, std::vector<cv::Point> & contour);
float ratio_black_points(cv::Point & origin, cv::Point & destination, cv::Mat & bin_sw, cv::Mat & bin_gr, cv::Mat & img_rgb);

// Auswerten Doppelgrünpunkt
struct double_green_point;
void swap(double_green_point *xp, double_green_point *yp);
void sort_green_pairs(double_green_point arr[], int n);

// Auswerten und binarisieren des Grünpunktes
void gruen_calc(cv::Mat & img_rgb, cv::Mat & img_hsv, cv::Mat & bin_sw, cv::Mat & bin_gr);
void separate_gruen(cv::Mat & hsv, cv::Mat & bin_gr);



#endif
