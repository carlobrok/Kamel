#include "opencv2/opencv.hpp"
#include "config.h"
#include "math.h"

#include "line.h"
#include "util.h"

Mat bin_ellipse;

void init_line_ellipse(Mat & img_rgb) {
	bin_ellipse = Mat(img_rgb.rows, img_rgb.cols, CV_8U, Scalar(0));
	ellipse(bin_ellipse, Point(img_rgb.cols/2, img_rgb.rows), Size(img_rgb.cols/2-ELLIPSE_THICKNESS/2, img_rgb.cols/3), 0, 180, 360, Scalar(255), ELLIPSE_THICKNESS);
}

float line_radiant(Point & p, int rows, int cols) {
	return atan2(p.y - rows, p.x - cols/2)  * 180 / CV_PI + 90;
}

void sepatare_line(Mat & hsv, Mat & bin_sw) {
	bool abgefragte_punkte[hsv.cols][hsv.rows];
	for(int x = 0; x < hsv.cols; x++) {
		for(int y = 0; y < hsv.rows; y++) {
			abgefragte_punkte[x][y] = false;
		}
	}
	vector<Point2i> neue_punkte;
	vector<Point2i> schwarze_punkte;

	int near_mitte = -1;
	Point2i p_near_mitte;
	for(int i = 0; i < hsv.cols; i++) {
		int color = (int)bin_sw.at<uchar>(hsv.rows-1,i);
		if(near_mitte == -1) {
			p_near_mitte.y = hsv.rows-1;
			p_near_mitte.x = i;
			near_mitte = abs(i-hsv.cols/2);
		}
		if(abs(i-hsv.cols/2) > near_mitte) {
			cout << "Weiter entfernt als nähester Punkt" << endl;
			i = hsv.cols;
		} else if(abs(i-hsv.cols/2) < near_mitte && color == 255) {
			p_near_mitte.x = i;
			schwarze_punkte.push_back(p_near_mitte);
			near_mitte = abs(i-hsv.cols/2);
		}
	}

	cout << "P_near_mitte: " << p_near_mitte << endl;

	neue_punkte.push_back(p_near_mitte);


	long timing_find = 0;

	bool check_for_points = true;
	while(check_for_points) {

		vector<Point2i> temp_neue_punkte;

		for(unsigned int i = 0; i < neue_punkte.size(); i++) {

			//				cout << "Center point: " << neue_punkte[i] << endl;

			Point2i point_left = neue_punkte[i];
			point_left.x = point_left.x - 1;
			//				cout << "Point left in Mat. x:" << point_left.x << " y: " << point_left.y << endl;

			int64_t ts = cv::getTickCount();
			if(inMat(point_left, hsv.cols, hsv.rows)) {

				if(abgefragte_punkte[point_left.x][point_left.y] == false) {
					int color = (int)bin_sw.at<uchar>(point_left.y,point_left.x);
					if(color == 255) {
						schwarze_punkte.push_back(point_left);
						temp_neue_punkte.push_back(point_left);
					}
					abgefragte_punkte[point_left.x][point_left.y] = true;
				}
			}

			Point2i point_right = neue_punkte[i];
			point_right.x = point_right.x + 1;

			if(inMat(point_right, hsv.cols, hsv.rows)) {
				//					cout << "Point right in Mat. x:" << point_right.x << " y: " << point_right.y << endl;
				if(abgefragte_punkte[point_right.x][point_right.y] == false) {
					int color = (int)bin_sw.at<uchar>(point_right.y,point_right.x);
					if(color == 255) {
						schwarze_punkte.push_back(point_right);
						temp_neue_punkte.push_back(point_right);
					}
					abgefragte_punkte[point_right.x][point_right.y] = true;
				}
			}

			Point2i point_over = neue_punkte[i];
			point_over.y = point_over.y - 1;

			if(inMat(point_over, hsv.cols, hsv.rows)) {
				//					cout << "Point over in Mat. " << point_over << endl;
				if(abgefragte_punkte[point_over.x][point_over.y] == false) {
					int color = (int)bin_sw.at<uchar>(point_over.y,point_over.x);
					if(color == 255) {
						schwarze_punkte.push_back(point_over);
						temp_neue_punkte.push_back(point_over);
					}
					abgefragte_punkte[point_over.x][point_over.y] = true;
				}
			}

			Point2i point_under = neue_punkte[i];
			point_under.y = point_under.y + 1;

			if(inMat(point_under, hsv.cols, hsv.rows)) {
				//					cout << "Point under in Mat. " << point_under << endl;
				if(abgefragte_punkte[point_under.x][point_under.y] == false) {
					int color = (int)bin_sw.at<uchar>(point_under);
					if(color == 255) {
						schwarze_punkte.push_back(point_under);
						temp_neue_punkte.push_back(point_under);
					}
					abgefragte_punkte[point_under.x][point_under.y] = true;
				}
			}
			timing_find += getTickCount() - ts;
		}
		/*cout << "Neue schwarze Punkte: " << endl;
				for(unsigned int i = 0; i < temp_neue_punkte.size(); i++) {
					cout << i << ": " << temp_neue_punkte[i] << endl;
				}*/

		if(temp_neue_punkte.size() == 0) {
			check_for_points = false;
		}
		neue_punkte = temp_neue_punkte;
	}

	cout << "1" << endl;
	bin_sw = Scalar(0);

	for(unsigned int i = 0; i < schwarze_punkte.size(); i++) {
		bin_sw.at<uchar>(schwarze_punkte[i]) = 255;
	}
}

void line_calc(Mat & img_rgb, Mat & hsv, Mat & bin_sw, Mat & bin_gr, vector<Point> & line_points) {
	inRange(hsv, Scalar(0, 0, 0), Scalar(180, 255, THRESH_BLACK), bin_sw);
	bin_sw -= bin_gr;
	morphologyEx(bin_sw, bin_sw, MORPH_OPEN, getStructuringElement(MORPH_ELLIPSE, Size(3,3)));

	if(bin_ellipse.empty()) {
		init_line_ellipse(img_rgb);
	}
	Mat bin_intersection;
	//sepatare_line(hsv, bin_sw);


	bitwise_and(bin_sw, bin_ellipse, bin_intersection);
	vector< vector<Point> > contours_line;

#ifdef VISUAL_DEBUG
	findContours(bin_intersection, contours_line, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	drawContours(img_rgb, contours_line, -1, Scalar(50, 140, 200), 1);
#endif


	line_points.clear();
	for (unsigned int i = 0; i < contours_line.size(); ++i) {
		Moments m = moments(contours_line[i]);
		//cout << "Cont " << i << "; size: " << m.m00 << " |";

		if(m.m00 < 300) {
			contours_line.erase(contours_line.begin()+i);
			i--;
			//cout << ", too small > erasing";
		} else {
			Point mitte;
			mitte.x = m.m10/m.m00;
			mitte.y = m.m01/m.m00;
			line_points.push_back(mitte);

#ifdef VISUAL_DEBUG
			circle(img_rgb, mitte, 1, Scalar(50, 90, 200),2);
#endif
		}

	}
	//cout << endl;

}
