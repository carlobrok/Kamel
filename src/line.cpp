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

// initialisieren der Ellipsen-Masken
// ! Nur 1 mal aufrufen !
void init_line_ellipse() {
	// Beide masken komplett schwarz malen, alle Werte im Bild auf cv::Scalar(0)
	bin_ellipse = cv::Mat(IMG_HEIGHT, IMG_WIDTH, CV_8U, cv::Scalar(0));

	// Weiße Halbellipsen auf Masken zeichnen
	cv::ellipse(bin_ellipse, cv::Point(IMG_WIDTH/2, IMG_HEIGHT - ELLIPSE_BAR_HEIGHT), cv::Size(IMG_WIDTH/2 - ELLIPSE_THICKNESS/2, ELLIPSE_HEIGHT), 0, 180, 360, cv::Scalar(255), ELLIPSE_THICKNESS);

	// Balken links und rechts zeichnen für bin_prim_ellipse
	cv::Rect left_rect(0 , IMG_HEIGHT - ELLIPSE_BAR_HEIGHT, ELLIPSE_THICKNESS, ELLIPSE_BAR_HEIGHT);		// neues rechteck für links
	cv::Rect right_rect(IMG_WIDTH - ELLIPSE_THICKNESS, IMG_HEIGHT - ELLIPSE_BAR_HEIGHT, ELLIPSE_THICKNESS, ELLIPSE_BAR_HEIGHT);		// neues rechteck für rechts
	cv::rectangle(bin_ellipse, left_rect, cv::Scalar(255), cv::FILLED);			// Rechteck links weiß auf bin_prim_ellipse zeichnen
	cv::rectangle(bin_ellipse, right_rect, cv::Scalar(255), cv::FILLED);		// Rechteck rechts weiß auf bin_prim_ellipse zeichnen
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


bool inRange(cv::Vec3b pixel_color, cv::Scalar low, cv::Scalar high) {
	return low[0] <= pixel_color[0] && pixel_color[0] <= high[0] && low[1] <= pixel_color[1] && pixel_color[1] <= high[1] && low[2] <= pixel_color[2] && pixel_color[2] <= high[2];
}

/*
 * WICHTIG: Das Bild muss eine Auflösung haben, wie sie als IMG_WIDTH und IMG_HEIGHT in der config.h definiert ist!
 *

/*
 * Schreibt nur die schwarzen Flächen in bin_sw, welche direkten Kontakt zur Ausgangsfläche haben.
 * Die Ausgangsfläche wird bestimmt durch den Punkt, der am nähesten zum Punkt P(480|320), bei einem Format von
 * 640x480p, liegt und sich in der unteren Reihe befindet.
 */
void sepatare_line(cv::Mat & hsv, cv::Mat & bin_sw) {
	bool abgefragte_punkte[IMG_WIDTH][IMG_HEIGHT];
	for(int x = 0; x < IMG_WIDTH; x++) {
		for(int y = 0; y < IMG_HEIGHT; y++) {
			abgefragte_punkte[x][y] = false;
		}
	}
	std::vector<cv::Point2i> neue_punkte;
	std::vector<cv::Point2i> schwarze_punkte;

	int near_mitte = -1;
	cv::Point2i p_near_mitte;
	for(int i = 0; i < IMG_WIDTH; i++) {
		//int color = (int)bin_sw.at<uchar>(IMG_HEIGHT-1,i);

		if(near_mitte == -1) {
			p_near_mitte.y = IMG_HEIGHT-1;
			p_near_mitte.x = i;
			near_mitte = abs(i-IMG_WIDTH/2);
		}
		if(abs(i-IMG_WIDTH/2) > near_mitte) {
			std::cout << "Weiter entfernt als nähester Punkt" << std::endl;
			i = IMG_WIDTH;
		} else if(abs(i-IMG_WIDTH/2) < near_mitte && inRange(hsv.at<cv::Vec3b>(IMG_HEIGHT-1,i), LOW_BLACK, HIGH_BLACK)) {
			p_near_mitte.x = i;
			schwarze_punkte.push_back(p_near_mitte);
			near_mitte = abs(i-IMG_WIDTH/2);
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
			if(inMat(point_left, IMG_WIDTH, IMG_HEIGHT)) {

				if(abgefragte_punkte[point_left.x][point_left.y] == false) {
					//int color = (int)bin_sw.at<uchar>(point_left.y,point_left.x);
					if(inRange(hsv.at<cv::Vec3b>(point_left), LOW_BLACK, HIGH_BLACK)) {
						schwarze_punkte.push_back(point_left);
						temp_neue_punkte.push_back(point_left);
					}
					abgefragte_punkte[point_left.x][point_left.y] = true;
				}
			}

			cv::Point2i point_right = neue_punkte[i];
			point_right.x = point_right.x + 1;

			if(inMat(point_right, IMG_WIDTH, IMG_HEIGHT)) {
				//					cout << "cv::Point right in cv::Mat. x:" << point_right.x << " y: " << point_right.y << endl;
				if(abgefragte_punkte[point_right.x][point_right.y] == false) {
					//int color = (int)bin_sw.at<uchar>(point_right.y,point_right.x);
					if(inRange(hsv.at<cv::Vec3b>(point_right), LOW_BLACK, HIGH_BLACK)) {
						schwarze_punkte.push_back(point_right);
						temp_neue_punkte.push_back(point_right);
					}
					abgefragte_punkte[point_right.x][point_right.y] = true;
				}
			}

			cv::Point2i point_over = neue_punkte[i];
			point_over.y = point_over.y - 1;

			if(inMat(point_over, IMG_WIDTH, IMG_HEIGHT)) {
				//					cout << "cv::Point over in cv::Mat. " << point_over << endl;
				if(abgefragte_punkte[point_over.x][point_over.y] == false) {
					//int color = (int)bin_sw.at<uchar>(point_over.y,point_over.x);
					if(inRange(hsv.at<cv::Vec3b>(point_over), LOW_BLACK, HIGH_BLACK)) {
						schwarze_punkte.push_back(point_over);
						temp_neue_punkte.push_back(point_over);
					}
					abgefragte_punkte[point_over.x][point_over.y] = true;
				}
			}

			cv::Point2i point_under = neue_punkte[i];
			point_under.y = point_under.y + 1;

			if(inMat(point_under, IMG_WIDTH, IMG_HEIGHT)) {
				//					cout << "cv::Point under in cv::Mat. " << point_under << endl;
				if(abgefragte_punkte[point_under.x][point_under.y] == false) {
					//int color = (int)bin_sw.at<uchar>(point_under);
					if(inRange(hsv.at<cv::Vec3b>(point_under), LOW_BLACK, HIGH_BLACK)) {
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

	inRange(hsv, LOW_BLACK, HIGH_BLACK, bin_sw);			// alles schwarze als weiß in bin_sw schreiben

	// Grünpunkt wird oft auch als schwarz erkannt, aktuelle Matrix des Grünpunktes, bin_gr, von bin_sw subtrahieren.
	// Somit ist alles, was auf bin_gr weiß ist auf bin_sw schwarz
	bin_sw -= bin_gr;
	cv::morphologyEx(bin_sw, bin_sw, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3)));

	if(bin_ellipse.empty()) {
		init_line_ellipse(img_rgb);
	}

	bin_intersection.release();


	cv::bitwise_and(bin_sw, bin_ellipse, bin_intersection);			


	std::vector< std::vector<cv::Point> > contours_line; // vector, der alle Konturen aus bin_prim_intersection enthält

	findContours(bin_prim_intersection, contours_line, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);	// alle Konturen in bin_prim_intersection finden und in contours_line schreiben

#ifdef VISUAL_DEBUG
	drawContours(img_rgb, contours_line, -1, cv::Scalar(50, 180, 180), 1);		// alle Konturen aus contours_line auf img_rgb malen
#endif

	std::vector<cv::Point> l_line_points;			// local line points holding vector

	// Prim. ellipse
	for (unsigned int i = 0; i < contours_line.size(); ++i) {		// vector contours_line durchlaufen

		cv::Moments m = cv::moments(contours_line[i]);						// Moments von aktueller Kontur in m schreiben

		if(m.m00 < 800) {				// Konturen, die eine Fläche kleiner als 300px haben werden ignoriert und gelöscht
			contours_line.erase(contours_line.begin() + i);			// Kontur i löschen
			i--;							// counter i einen runter setzen, da sonst die nächste Kontur übersprungen wird
		} else {
			std::cout << "contour size: " << m.m00 << std::endl;
			// Punkt der Mitte berechnen und an l_line_points anhängen
			cv::Point mitte;
			mitte.x = m.m10/m.m00;
			mitte.y = m.m01/m.m00;
			l_line_points.push_back(mitte);

#ifdef VISUAL_DEBUG
			cv::circle(img_rgb, mitte, 1, cv::Scalar(50, 90, 200),2);			// Mitte der Kontur auf Bild img_rgb malen
#endif
		}
	}



	set_line_data(l_line_points);			// Die neuen prim_line_points und sec_line_points in die globalen Variablen schreiben
}
