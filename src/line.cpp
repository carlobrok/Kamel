#include <mutex>
#include <math.h>				// atan2
#include <opencv2/opencv.hpp>

#include "config.h"
#include "line.h"
#include "util.h"

std::mutex line_mutex;

std::vector<cv::Point> m_prim_line_points;			// global line points holding vector
std::vector<cv::Point> m_sec_line_points;			// global line points holding vector

cv::Mat bin_prim_ellipse;								// Maske mit primärer Ellipse
cv::Mat bin_prim_intersection;					// Überschneidungsmatrix von bin_sw und bin_prim_ellipse
cv::Mat bin_sec_ellipse;								// Maske mit sekundärer Ellipse
cv::Mat bin_sec_intersection;						// Überschneidungsmatrix von bin_sw und bin_sec_ellipse

// set line_points buffer
void set_line_data(std::vector<cv::Point> & prim_line_points, std::vector<cv::Point> & sec_line_points) {
	std::lock_guard<std::mutex> m_lock(line_mutex);			// mutex locken, zugriff auf die nächsten Variablen sperren
	m_prim_line_points = prim_line_points;			// prim_line_points in buffer speichern
	m_sec_line_points = sec_line_points;				// sec_line_points in buffer speichern
}

// get line_points buffer
void get_line_data(std::vector<cv::Point> & prim_line_points, std::vector<cv::Point> & sec_line_points) {
	std::lock_guard<std::mutex> m_lock(line_mutex);			// mutex locken, zugriff auf die nächsten Variablen sperren
	prim_line_points = m_prim_line_points;			// buffer zurückgeben in Referenz prim_line_points
	sec_line_points = m_sec_line_points;				// buffer zurückgeben in Referenz sec_line_points
}

// initialisieren der Ellipsen-Masken
// ! Nur 1 mal aufrufen !
void init_line_ellipse(cv::Mat & img_rgb) {
	// Beide masken komplett schwarz malen, alle Werte im Bild auf cv::Scalar(0)
	bin_prim_ellipse = cv::Mat(img_rgb.rows, img_rgb.cols, CV_8U, cv::Scalar(0));
	bin_sec_ellipse = cv::Mat(img_rgb.rows, img_rgb.cols, CV_8U, cv::Scalar(0));

	// Weiße Halbellipsen auf Masken zeichnen
	cv::ellipse(bin_prim_ellipse, cv::Point(img_rgb.cols/2, img_rgb.rows * 3/4), cv::Size(img_rgb.cols/2-ELLIPSE_THICKNESS/2, img_rgb.cols/3), 0, 180, 360, cv::Scalar(255), ELLIPSE_THICKNESS);
	cv::ellipse(bin_sec_ellipse, cv::Point(img_rgb.cols/2, img_rgb.rows), cv::Size(img_rgb.cols/2- ELLIPSE_THICKNESS/2 - ELLIPSE_THICKNESS, img_rgb.cols/3), 0, 180, 360, cv::Scalar(255), ELLIPSE_THICKNESS);

	// Balken links und rechts zeichnen für bin_prim_ellipse
	cv::Rect left_rect(0 , img_rbg.rows * 3/4, ELLIPSE_THICKNESS, img_rbg.rows * 1/4);		// neues rechteck für links
	cv::Rect right_rect(img_rbg.cols - ELLIPSE_THICKNESS, img_rbg.rows * 3/4, ELLIPSE_THICKNESS, img_rbg.rows * 1/4);		// neues rechteck für rechts
	cv::rectangle(bin_prim_ellipse, left_rect, cv::Scalar(255), cv::FILLED);			// Rechteck links weiß auf bin_prim_ellipse zeichnen
	cv::rectangle(bin_sec_ellipse, right_rect, cv::Scalar(255), cv::FILLED);		// Rechteck rechts weiß auf bin_sec_ellipse zeichnen
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

	cv::morphologyEx(bin_sw, bin_sw, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3)));	// alle kleinstflecken entfernen

	if(bin_prim_ellipse.empty() || bin_sec_ellipse.empty()) {			//Wenn diese Matritzen leer sind
		init_line_ellipse(img_rgb);					//  Ellipsen initialisieren
	}

	bin_prim_intersection.release();		// Überschneidungsmatrix leeren
	bin_sec_intersection.release();			// Überschneidungsmatrix leeren

	// Maske bin_prim_ellipse / bin_sec_ellipse auf bin_sw anwenden
	// Output: bin_prim_intersection / bin_sec_intersection
	cv::bitwise_and(bin_sw, bin_prim_ellipse, bin_prim_intersection);
	cv::bitwise_and(bin_sw, bin_sec_ellipse, bin_sec_intersection);

	std::vector< std::vector<cv::Point> > prim_contours_line; // vector, der alle Konturen aus bin_prim_intersection enthält
	std::vector< std::vector<cv::Point> > sec_contours_line;	// vector, der alle Konturen aus bin_sec_intersection enthält

	findContours(bin_prim_intersection, prim_contours_line, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);	// alle Konturen in bin_prim_intersection finden und in prim_contours_line schreiben
	findContours(bin_sec_intersection, sec_contours_line, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);	// alle Konturen in bin_sec_intersection finden und in sec_contours_line schreiben

#ifdef VISUAL_DEBUG
	drawContours(img_rgb, prim_contours_line, -1, cv::Scalar(50, 180, 180), 1);		// alle Konturen aus prim_contours_line auf img_rbg malen
	drawContours(img_rgb, sec_contours_line, -1, cv::Scalar(50, 140, 200), 1);		// alle Konturen aus sec_contours_line auf img_rbg malen
#endif

	std::vector<cv::Point> l_prim_line_points;			// local line points holding vector
	std::vector<cv::Point> l_sec_line_points;			// local line points holding vector

	// Prim. ellipse
	for (unsigned int i = 0, cv::Moments m; i < prim_contours_line.size(); ++i) {		// vector prim_contours_line durchlaufen
		m = cv::moments(prim_contours_line[i]);						// Moments von aktueller Kontur in m schreiben

		if(m.m00 < 300) {				// Konturen, die eine Fläche kleiner als 300px haben werden ignoriert und gelöscht
			prim_contours_line.erase(prim_contours_line.begin() + i);			// Kontur i löschen
			i--;							// counter i einen runter setzen, da sonst die nächste Kontur übersprungen wird
		} else {
			// Punkt der Mitte berechnen und an l_prim_line_points anhängen
			cv::Point mitte;
			mitte.x = m.m10/m.m00;
			mitte.y = m.m01/m.m00;
			l_prim_line_points.push_back(mitte);

#ifdef VISUAL_DEBUG
			cv::circle(img_rgb, mitte, 1, cv::Scalar(50, 90, 200),2);			// Mitte der Kontur auf Bild img_rbg malen
#endif
		}
	}

	// Sec. ellipse
	for (unsigned int i = 0, cv::Moments m; i < sec_contours_line.size(); ++i) {		// vector sec_contours_line durchlaufen
		m = cv::moments(sec_contours_line[i]);					// Moments von aktueller Kontur in m schreiben

		if(m.m00 < 300) {				// Konturen, die eine Fläche kleiner als 300px haben werden ignoriert und gelöscht
			sec_contours_line.erase(sec_contours_line.begin()+i);					// Kontur i löschen
			i--;						// counter i einen runter setzen, da sonst die nächste Kontur übersprungen wird
		} else {
			// Punkt der Mitte berechnen und an l_sec_line_points anhängen
			cv::Point mitte;
			mitte.x = m.m10/m.m00;
			mitte.y = m.m01/m.m00;
			l_sec_line_points.push_back(mitte);

	#ifdef VISUAL_DEBUG
			cv::circle(img_rgb, mitte, 1, cv::Scalar(50, 90, 200),2);					// Mitte der Kontur auf Bild img_rbg malen
	#endif
		}
	}


	set_line_data(l_prim_line_points, l_sec_line_points);			// Die neuen prim_line_points und sec_line_points in die globalen Variablen schreiben
}
