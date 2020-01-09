#include <mutex>
#include <math.h>				// atan2
#include <opencv2/opencv.hpp>

#include "config.h"
#include "line.h"
#include "util.h"

std::mutex line_mutex;

std::vector<cv::Point> m_line_points;			// global line points holding vector

cv::Mat bin_ellipse;
cv::Mat bin_intersection;

// set line_points buffer
void set_line_data(std::vector<cv::Point> & line_points) {
	std::lock_guard<std::mutex> m_lock(line_mutex);
	m_line_points = line_points;
}

// get line_points buffer
void get_line_data(std::vector<cv::Point> & line_points) {
	std::lock_guard<std::mutex> m_lock(line_mutex);
	line_points = m_line_points;
}

void init_line_ellipse(cv::Mat & img_rgb) {
	bin_ellipse = cv::Mat(img_rgb.rows, img_rgb.cols, CV_8U, cv::Scalar(0));
	cv::ellipse(bin_ellipse, cv::Point(img_rgb.cols/2, img_rgb.rows), cv::Size(img_rgb.cols/2-ELLIPSE_THICKNESS/2, img_rgb.cols/3), 0, 180, 360, cv::Scalar(255), ELLIPSE_THICKNESS);
}

/*
 * Gibt das Bogenmaß zwischen dem mittleren Punkt der unteren Bildreihe und des übergebenen Linepoints zurück
 * rows: Höhe des Bildes in Zeilen
 * cols: Breite des Bildes in Spalten
 */
float line_radiant(cv::Point & p, int rows, int cols) {
	return atan2(p.y - rows, p.x - cols/2)  * 180 / CV_PI + 90;
}

/*
 * Gibt das Bogenmaß zwischen dem mittleren Punkt der unteren Bildreihe und des übergebenen Linepoints zurück
 * img: input image mit dem die Auswertung statt findet
 */
float line_radiant(cv::Point & p, cv::Mat & img) {
	return atan2(p.y - img.rows, p.x - img.cols/2)  * 180 / CV_PI + 90;
}

/*
 * Schreibt nur die schwarzen Flächen in bin_sw, welche direkten Kontakt zur Ausgangsfläche haben.
 * Die Ausgangsfläche wird bestimmt durch den Punkt, der am nähesten zum Punkt P(480|320), bei einem Format von
 * 640x480p, liegt und sich in der unteren Reihe befindet.
 */
void sepatare_line(cv::Mat & hsv, cv::Mat & bin_sw) {
	bool abgefragte_punkte[hsv.cols][hsv.rows];
	for(int x = 0; x < hsv.cols; x++) {
		for(int y = 0; y < hsv.rows; y++) {
			abgefragte_punkte[x][y] = false;
		}
	}
	std::vector<cv::Point2i> neue_punkte;
	std::vector<cv::Point2i> schwarze_punkte;

	int near_mitte = -1;
	cv::Point2i p_near_mitte;
	for(int i = 0; i < hsv.cols; i++) {
		int color = (int)bin_sw.at<uchar>(hsv.rows-1,i);
		if(near_mitte == -1) {
			p_near_mitte.y = hsv.rows-1;
			p_near_mitte.x = i;
			near_mitte = abs(i-hsv.cols/2);
		}
		if(abs(i-hsv.cols/2) > near_mitte) {
			std::cout << "Weiter entfernt als nähester Punkt" << std::endl;
			i = hsv.cols;
		} else if(abs(i-hsv.cols/2) < near_mitte && color == 255) {
			p_near_mitte.x = i;
			schwarze_punkte.push_back(p_near_mitte);
			near_mitte = abs(i-hsv.cols/2);
		}
	}

	std::cout << "P_near_mitte: " << p_near_mitte << std::endl;

	neue_punkte.push_back(p_near_mitte);


	long timing_find = 0;

	bool check_for_points = true;
	while(check_for_points) {

		std::vector<cv::Point2i> temp_neue_punkte;

		for(unsigned int i = 0; i < neue_punkte.size(); i++) {

			//				cout << "Center point: " << neue_punkte[i] << endl;

			cv::Point2i point_left = neue_punkte[i];
			point_left.x = point_left.x - 1;
			//				cout << "Point left in cv::Mat. x:" << point_left.x << " y: " << point_left.y << endl;

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

			cv::Point2i point_right = neue_punkte[i];
			point_right.x = point_right.x + 1;

			if(inMat(point_right, hsv.cols, hsv.rows)) {
				//					cout << "cv::Point right in cv::Mat. x:" << point_right.x << " y: " << point_right.y << endl;
				if(abgefragte_punkte[point_right.x][point_right.y] == false) {
					int color = (int)bin_sw.at<uchar>(point_right.y,point_right.x);
					if(color == 255) {
						schwarze_punkte.push_back(point_right);
						temp_neue_punkte.push_back(point_right);
					}
					abgefragte_punkte[point_right.x][point_right.y] = true;
				}
			}

			cv::Point2i point_over = neue_punkte[i];
			point_over.y = point_over.y - 1;

			if(inMat(point_over, hsv.cols, hsv.rows)) {
				//					cout << "cv::Point over in cv::Mat. " << point_over << endl;
				if(abgefragte_punkte[point_over.x][point_over.y] == false) {
					int color = (int)bin_sw.at<uchar>(point_over.y,point_over.x);
					if(color == 255) {
						schwarze_punkte.push_back(point_over);
						temp_neue_punkte.push_back(point_over);
					}
					abgefragte_punkte[point_over.x][point_over.y] = true;
				}
			}

			cv::Point2i point_under = neue_punkte[i];
			point_under.y = point_under.y + 1;

			if(inMat(point_under, hsv.cols, hsv.rows)) {
				//					cout << "cv::Point under in cv::Mat. " << point_under << endl;
				if(abgefragte_punkte[point_under.x][point_under.y] == false) {
					int color = (int)bin_sw.at<uchar>(point_under);
					if(color == 255) {
						schwarze_punkte.push_back(point_under);
						temp_neue_punkte.push_back(point_under);
					}
					abgefragte_punkte[point_under.x][point_under.y] = true;
				}
			}
			timing_find += cv::getTickCount() - ts;
		}

		if(temp_neue_punkte.size() == 0) {
			check_for_points = false;
		}
		neue_punkte = temp_neue_punkte;
	}

	bin_sw = cv::Scalar(0);

	for(unsigned int i = 0; i < schwarze_punkte.size(); i++) {
		bin_sw.at<uchar>(schwarze_punkte[i]) = 255;
	}
}

/*
 * Hauptfunktion zum auswerten der linepoints
 * Die ausgewerteten prim_line_points und sec_line_points werden zwischen gespeichert und können
 * mit get_line_data abgerufen werden.
 */
void line_calc(cv::Mat & img_rgb, cv::Mat & hsv, cv::Mat & bin_sw, cv::Mat & bin_gr, bool do_separate_line) {

	// if(do_separate_line) {
	// 	sepatare_line(hsv, bin_sw);
	// }

	inRange(hsv, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, THRESH_BLACK), bin_sw);			// alles schwarze als weiß in bin_sw schreiben

	// Grünpunkt wird oft auch als schwarz erkannt, aktuelle Matrix des Grünpunktes, bin_gr, von bin_sw subtrahieren.
	// Somit ist alles, was auf bin_gr weiß ist auf bin_sw schwarz
	bin_sw -= bin_gr;
	cv::morphologyEx(bin_sw, bin_sw, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3)));

	if(bin_ellipse.empty()) {
		init_line_ellipse(img_rgb);
	}

	bin_intersection.release();

	// if(do_separate_line) {
	// 	sepatare_line(hsv, bin_sw);
	// }

	cv::bitwise_and(bin_sw, bin_ellipse, bin_intersection);
	std::vector< std::vector<cv::Point> > contours_line;

#ifdef VISUAL_DEBUG
	findContours(bin_intersection, contours_line, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	drawContours(img_rgb, contours_line, -1, cv::Scalar(50, 140, 200), 1);
#endif


	line_points.clear();
	for (unsigned int i = 0; i < contours_line.size(); ++i) {
		cv::Moments m = cv::moments(contours_line[i]);

		if(m.m00 < 300) {
			contours_line.erase(contours_line.begin()+i);
			i--;
		} else {
			cv::Point mitte;
			mitte.x = m.m10/m.m00;
			mitte.y = m.m01/m.m00;
			line_points.push_back(mitte);

#ifdef VISUAL_DEBUG
			cv::circle(img_rgb, mitte, 1, cv::Scalar(50, 90, 200),2);
#endif
		}

	}
}
