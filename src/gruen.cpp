#include <mutex>
#include <opencv2/opencv.hpp>
#include <math.h>

#include "gruen.h"
#include "util.h"
#include "config.h"


std::mutex gruen_mutex;				// mutex for global green values

cv::Point m_grcenter(0,0);
int m_grstate = GRUEN_NICHT;

cv::Scalar lower_gruen = LOWER_GRUEN_DEFAULT;
cv::Scalar upper_gruen = UPPER_GRUEN_DEFAULT;

void set_gruen_range(cv::Scalar lower, cv::Scalar upper) {
	lower_gruen = lower;
	upper_gruen = upper;
}


void set_gruen_data(cv::Point & grcenter, int & grstate) {
	std::lock_guard<std::mutex> m_lock(gruen_mutex);				// lock gruen_mutex
	m_grcenter = grcenter;																	// write to grcenter buffer
	m_grstate =  grstate;																		// write to grstate buffer
}

void set_gruen_data(cv::Point grcenter, int grstate) {
	std::lock_guard<std::mutex> m_lock(gruen_mutex);				// lock gruen_mutex
	m_grcenter = grcenter;																	// write to grcenter buffer
	m_grstate =  grstate;																		// write to grstate buffer
}

void get_gruen_data(cv::Point & grcenter, int & grstate) {
	std::lock_guard<std::mutex> m_lock(gruen_mutex);				// lock gruen_mutex
	grcenter = m_grcenter;														// write grcenter buffer to variable passed by reference
	grstate = m_grstate;															// write grstate buffer to variable passed by reference
}


/*
 * returns the center of two green point contours
 */
cv::Point gruen_center(std::vector <cv::Point> & cont1, std::vector <cv::Point> & cont2) {
	cv::Moments m1 = cv::moments(cont1);
	cv::Moments m2 = cv::moments(cont2);

	return cv::Point((m1.m10 / m1.m00 + m2.m10 / m2.m00) / 2, (m1.m01 / m1.m00 + m2.m01 / m2.m00) / 2);
	// x = (x1 + x2) / 2; y = (y1 + y2) / 2
}


/*
 * returns the center of two points
 */
cv::Point gruen_center(cv::Point & p1, cv::Point & p2) {
	return cv::Point((p1.x + p2.x) / 2, (p1.y + p2.y) / 2);
	// x = (x1 + x2) / 2; y = (y1 + y2) / 2
}

cv::Point gruen_center(std::vector<cv::Point> & contour) {
	cv::Moments m = cv::moments(contour);
	return cv::Point(m.m10 / m.m00, m.m01 / m.m00);
}


/*
 * Gibt den Winkel zwischen der Geraden der Punkte p1-p2 und der Horizontalen zurück
 */
float line_angle(cv::Point & p1, cv::Point & p2) {
	return atan2(p1.y - p2.y, p1.x - p2.x)  * 180 / CV_PI;
}

/*
 * Gibt die Distanz des unteren mittleren Punktes zum gegebenen Punkt zurück
 */
float point_distance(cv::Point p) {
	return sqrt((p.x-IMG_WIDTH/2)*(p.x-IMG_WIDTH/2) + (p.y-IMG_HEIGHT)*(p.y-IMG_HEIGHT));
}

/* gibt einen Punkt zurück, der von rotating_point um origin_point gedreht wird.
	rotation ist dabei der Winkel in Grad.
 	Die Distanz, die der neue Punkt vom Punkt origin entfernt ist, wird mit length_factor angegeben.
*/
cv::Point rotate_point(cv::Point & origin_point, cv::Point & rotating_point, float rotation, float length_factor = 1) {
	float rad = CV_PI / 180 * rotation;			// bogenmaß von rotation berechnen

	cv::Point new_point;

	new_point.x = ((rotating_point.x - origin_point.x) * cos(rad) - (rotating_point.y - origin_point.y) * sin(rad)) * length_factor + origin_point.x;
	new_point.y = ((rotating_point.x - origin_point.x) * sin(rad) + (rotating_point.y - origin_point.y) * cos(rad)) * length_factor + origin_point.y;

	return new_point;
}


/*
cv::Point rotated_point_lenght(cv::Point & origin_point, float rotation, float length) {
	float rad = CV_PI / 180 * rotation;

	cv::Point temporary_point = cv::Point(origin_point.x, origin_point.y + length);
	cv::Point new_point;

	new_point.x = (temporary_point.x - origin_point.x) * cos(rad) - (temporary_point.y - origin_point.y) * sin(rad) + origin_point.x;
	new_point.y = (temporary_point.x - origin_point.x) * sin(rad) + (temporary_point.y - origin_point.y) * cos(rad) + origin_point.y;

	return new_point;
}*/


/*
 * Checkt bei einem grünen Punkt vertikal nach oben den anteil der Schwarzen linie.
 */
int gruen_check_normal(cv::Mat & img_rgb, cv::Mat & bin_sw, cv::Mat & bin_gr, std::vector<cv::Point> & contour) {

	cv::Moments m = cv::moments(contour);
	float point_distance = sqrt(m.m00) * 1.2;			// Länge, in die geprüft wird ist: Wurzel aus der Fläche des Grünen Punktes * 1.2

	cv::Point mittegr = gruen_center(contour);		// Mitte des grünen Punktes
	cv::Point top_checkpoint = mittegr;						// Prüfpunkt oberhalb des grünen Punktes
	top_checkpoint.y -= point_distance;						// y um point_distance nach oben verschieben

	if(ratio_black_points(mittegr, top_checkpoint, bin_sw, bin_gr, img_rgb) > 0.5) {		// wenn mehr als 50% der geprüften Punkte schwarz ist
		cv::Point left_checkpoint = mittegr;		// linker Prüfpunkt
		cv::Point right_checkpoint = mittegr;		// rechter Prüfpunkt
		left_checkpoint.x -= point_distance;		// nach links verschieben
		right_checkpoint.x += point_distance;		// nach rechts verschieben

		float ratio_left = ratio_black_points(mittegr, left_checkpoint, bin_sw, bin_gr, img_rgb);		// Anteil an schwarz links
		float ratio_right = ratio_black_points(mittegr, right_checkpoint, bin_sw, bin_gr, img_rgb);	// Anteil an schwarz rechts

		if(ratio_left > ratio_right) {		// links ist mehr schwarz
			return GRUEN_RECHTS;
		} else if(ratio_right > ratio_left) {			// rechts ist mehr schwarz
			return GRUEN_LINKS;
		}
	}

	return GRUEN_NICHT;
}


/* Checkt, wie viele schwarze Punkte sich zwischen origin und destination befinden.
 *
 * Maximaler Wert: Anzahl an checks, also NUM_ITERATIONS_BLACK_POINTS
 *
 * Rückgabewert: Verhältnis der Punkte, die Schwarz waren.
 * Wert zwischen 0 und 1
 */
float ratio_black_points(cv::Point & origin, cv::Point & destination, cv::Mat & bin_sw, cv::Mat & bin_gr, cv::Mat & img_rgb) {
	int black_pixels = 0;
	int green_pixels = 0;
	cv::Point cur_pixel;
	int color_sw;
	int color_gr;

	for(int i = 1; i < NUM_ITERATIONS_BLACK_POINTS; i++) {
		cur_pixel.x = origin.x + (float)i/30 * (destination.x - origin.x);
		cur_pixel.y = origin.y + (float)i/30 * (destination.y - origin.y);

		if(inMat(cur_pixel, bin_sw)) {

			color_sw = (int)bin_sw.at<uchar>(cur_pixel);
			color_gr = (int)bin_gr.at<uchar>(cur_pixel);

			//cout << "gr_punkt: " << id << "; Pixel: " << cur_pixel << " bin_colors:  sw:" << color_sw << " gr:" << color_gr << endl;

			if(color_sw == 255 && color_gr == 0) {
				black_pixels++;

	#ifdef VISUAL_DEBUG
				cv::circle(img_rgb, cur_pixel, 1, cv::Scalar(50, 50, 50), 2);
	#endif
			} else if(color_gr == 255) {
				green_pixels++;
	#ifdef VISUAL_DEBUG
				cv::circle(img_rgb, cur_pixel, 1, cv::Scalar(20, 150, 20), 2);
	#endif
			} else {
	#ifdef VISUAL_DEBUG
				cv::circle(img_rgb, cur_pixel, 1, cv::Scalar(200,200,200), 2);
	#endif
			}
		} else {
			i = NUM_ITERATIONS_BLACK_POINTS;
		}
	}

	return (float) black_pixels / (NUM_ITERATIONS_BLACK_POINTS - 1 - green_pixels);
}




/*
 * FUNKTIONEN ZUM ANALYSIEREN DES DOPPELTEN GRÜNEN Punktes   	========================================
 */

struct double_green_point {
	cv::Point p1;
	cv::Point p2;
	float distance;
	float angle;
	float absolute_angle;
};

// Tauschen der Werte von double_green_point
void swap(double_green_point *xp, double_green_point *yp)
{
    double_green_point temp = *xp;
    *xp = *yp;
    *yp = temp;
}

// bubble sort for struct double_green_point
void sort_green_angle(double_green_point arr[], int n)
{
    int i, j;
    for (i = 0; i < n-1; i++)

    // Last i elements are already in place
    for (j = 0; j < n-i-1; j++)
        if (arr[j].angle > arr[j+1].angle)
            swap(&arr[j], &arr[j+1]);
}

void sort_green_dist(double_green_point arr[], int n)
{
    int i, j;
    for (i = 0; i < n-1; i++)

    // Last i elements are already in place
    for (j = 0; j < n-i-1; j++)
        if (arr[j].distance > arr[j+1].distance)
            swap(&arr[j], &arr[j+1]);
}

// =====================================================


cv::Scalar pairs_color[6] = {cv::Scalar(0, 153, 0), cv::Scalar(0, 122, 51), cv::Scalar(0, 92, 102), cv::Scalar(0, 61, 153), cv::Scalar(0, 31, 204), cv::Scalar(0,0,255)};


void gruen_calc(cv::Mat & img_rgb, cv::Mat & img_hsv, cv::Mat & bin_sw, cv::Mat & bin_gr) {

	std::vector< std::vector<cv::Point> > contg;

	//morphologyEx(img_bingr, img_bingr, MORPH_OPEN, getStructuringElement(MORPH_ELLIPSE, Size(10,10)));
	cv::findContours(bin_gr, contg, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);							// alle Konturen extrahieren


	// zu kleine Konturen löschen
	for(unsigned int i = 0; i < contg.size(); i++) {
		cv::Moments m = cv::moments(contg[i]);
		if(m.m00 < 800) {
			contg.erase(contg.begin() + i--);
		}
	}

#ifdef VISUAL_DEBUG
	cv::drawContours(img_rgb, contg, -1, cv::Scalar(50, 230, 50), 1);		// Alle übrigen Konturen ausgeben
#endif



	// 2 oder mehr grüne Punkte
	if(contg.size() > 1) {

		/*
		 * Mehr als 1 Grüner Punkt
		 */



		// Wenn der unterschied der Fläche kleiner als 1500 ist, oder die Fläche beider grünen Punkte größer als 3500px ist
		/*if(abs(m1.m00 - m2.m00) < 1500 || (m1.m00 > 3500 && m2.m00 > 3500)) {

		}*/


		// Mittelpunkte bestimmen
		cv::Moments m1, m2;
		cv::Point p1, p2;


		// Wenn mehr als 2 Grüne Punkte vorhanden sind müssen die relevanten bzw. der relevante grüne Punkt gefunden werden.
		// Zuerst wird der Winkel aller grüner Punkte zueinander berechnet, der Winkel, welcher am horizontalsten ist, ist der Entscheidende
		// Nach diesem wird green_pair_index festgelegt

		if(contg.size() > 2) {

			int pairs = (contg.size()*(contg.size()-1)/2);

			std::cout << "have more than green 1 point; calcutating " << pairs << " pairs of points" << std::endl;

			double_green_point point_pairs[pairs];
			int pair_counter = 0;

			float angle;

			for(int i = 0; i < contg.size()-1; i++) {
				m1 = cv::moments(contg[i]);
				p1 = cv::Point(m1.m10 / m1.m00, m1.m01 / m1.m00);

				for(int n = i+1; n < contg.size(); n++) {
					m2 = cv::moments(contg[n]);
					p2 = cv::Point(m2.m10 / m2.m00, m2.m01 / m2.m00);

					std::cout << i << " " << n << " / " << p1 << " / " << p2 << std::endl;
					std::cout << "m00: " << m1.m00 << " / " << m2.m00 << std::endl;

					angle = line_angle(p1, p2);

					if(angle > 90) {
					  angle -= 180;
					}

					point_pairs[pair_counter].absolute_angle = angle;
					std::cout << "Absolute angle: " << angle << std::endl;

					angle = abs(angle);
					std::cout << "Relative angle (angle to horizontal): " << angle << "°" << std::endl;

					point_pairs[pair_counter].p1 = p1;
					point_pairs[pair_counter].p2 = p2;
					point_pairs[pair_counter].angle = angle;
					point_pairs[pair_counter].distance = point_distance(gruen_center(p1,p2));

					pair_counter++;

					cv::line(img_rgb, p1, p2, cv::Scalar(255,0,100), 2);
		    }
		  }



			if(contg.size() == 3) {
				sort_green_dist(point_pairs, pairs);
				p1 = point_pairs[0].p1;
				p2 = point_pairs[0].p2;
			}

			else if(contg.size() == 4) {
				sort_green_angle(point_pairs, pairs);

				std::cout << std::endl << "#0 to #1: " << abs(point_pairs[0].absolute_angle - point_pairs[1].absolute_angle) << std::endl;
				std::cout << "#0 to #2: " << abs(point_pairs[0].absolute_angle - point_pairs[2].absolute_angle) << std::endl;
				std::cout << "#1 to #2: " << abs(point_pairs[1].absolute_angle - point_pairs[2].absolute_angle) << std::endl;

				for(int i = 0; i < pairs; i++) {
					cv::line(img_rgb, point_pairs[i].p1, point_pairs[i].p2, pairs_color[i], 2);
				}

				if(abs(point_pairs[0].absolute_angle - point_pairs[1].absolute_angle) < 15) {
					std::cout << "#0 and #1 are parallel" << std::endl;
					if(point_pairs[0].distance > point_pairs[1].distance) {
						p1 = point_pairs[1].p1;
						p2 = point_pairs[1].p2;
					} else {
						p1 = point_pairs[0].p1;
						p2 = point_pairs[0].p2;
					}
				}

				else if(abs(point_pairs[0].absolute_angle - point_pairs[2].absolute_angle) < 15) {
					std::cout << "#0 and #2 are parallel" << std::endl;
					if(point_pairs[0].distance > point_pairs[2].distance) {
						p1 = point_pairs[2].p1;
						p2 = point_pairs[2].p2;
					} else {
						p1 = point_pairs[0].p1;
						p2 = point_pairs[0].p2;
					}
				}

				else if(abs(point_pairs[1].absolute_angle - point_pairs[2].absolute_angle) < 15) {
					std::cout << "#1 and #2 are parallel" << std::endl;
					if(point_pairs[1].distance > point_pairs[2].distance) {
						p1 = point_pairs[2].p1;
						p2 = point_pairs[2].p2;
					} else {
						p1 = point_pairs[1].p1;
						p2 = point_pairs[1].p2;
					}
				}

				else {
					std::cout << "Something went WRONG!!" << std::endl;
				}
			}

		} else {
			m1 = cv::moments(contg[0]);
			m2 = cv::moments(contg[1]);

			p1 = cv::Point(m1.m10 / m1.m00, m1.m01 / m1.m00);
			p2 = cv::Point(m2.m10 / m2.m00, m2.m01 / m2.m00);
		}



#ifdef VISUAL_DEBUG
		cv::line(img_rgb, p1, p2, cv::Scalar(255,0,100), 2);
#endif

		// Wenn die Höhendifferenz der Punkte klein genug ist bzw. die Punkte au etwa gleicher Höhe sind
		if(abs(p1.y-p2.y < sqrt((m1.m00 + m2.m00) / 2))) {
			std::cout << "The height diference is small enough" << std::endl;
			// Punkte richtig anordnen, wenn nötig
			if(p1.x > p2.x) {
				cv::Point p_tmp = p2;
				p2 = p1;
				p1 = p_tmp;
			}

			cv::Point p1_new = rotate_point(p1, p2, -135, 0.8);
			cv::Point p2_new = rotate_point(p2, p1, 135, 0.8);

			float ratio_left_point = ratio_black_points(p1, p1_new, bin_sw, bin_gr, img_rgb);
			float ratio_right_point = ratio_black_points(p2, p2_new, bin_sw, bin_gr, img_rgb);

#ifdef DEBUG_GRUEN
			std::cout << "Ratio Blackpoints:	Links: " << ratio_left_point
					<< "		Rechts: " << ratio_right_point << std::endl;
#endif

			if(ratio_left_point > 0.4 && ratio_right_point > 0.4) {
				// update green data
				set_gruen_data(gruen_center(p1_new, p2_new), GRUEN_BEIDE);
#ifdef VISUAL_DEBUG
				cv::circle(img_rgb, p1, 2, cv::Scalar(0,255,0), 2);
				cv::circle(img_rgb, p2, 2, cv::Scalar(0,255,0), 2);
				cv::circle(img_rgb, p1_new, 2, cv::Scalar(0,0,255), 2);
				cv::circle(img_rgb, p2_new, 2, cv::Scalar(0,0,255), 2);
#endif
				return;
			} else {
#ifdef VISUAL_DEBUG
				cv::circle(img_rgb, p1, 2, cv::Scalar(0,0,255), 2);
				cv::circle(img_rgb, p2, 2, cv::Scalar(0,0,255), 2);
#endif
				std::cout << "Not enough black over both points" << std::endl;
			}
		}

		int index_nearest = 0;
		double nearest_distance = -1;
		cv::Point nearest_point;

		for(unsigned int i = 0; i < contg.size(); i++) {
			cv::Point center = gruen_center(contg[i]);
			float distance_to_point = point_distance(center);

			if(distance_to_point < nearest_distance || nearest_distance == -1) {
				nearest_distance = distance_to_point;
				index_nearest = i;
				nearest_point = center;
			}
		}

		set_gruen_data(nearest_point, gruen_check_normal(img_rgb, bin_sw, bin_gr, contg[index_nearest]));
#ifdef VISUAL_DEBUG
		cv::circle(img_rgb, nearest_point, 2, cv::Scalar(255,0,0), 2);
#endif
		return;

	}

	// Nur 1 Grüner Punkt
	else if(contg.size() == 1){
		set_gruen_data(gruen_center(contg[0]), gruen_check_normal(img_rgb, bin_sw, bin_gr, contg[0]));
		return;
	}

	// Kein grüner Punkt
	set_gruen_data(cv::Point(0,0), GRUEN_NICHT);		// kein grüner Punkt oder Bedingungen nicht erfüllt
	return;
}


/*
 * Alle bereiche, wo grün vorhanden ist in bin_gr schreiben.
 */
void separate_gruen(cv::Mat & hsv, cv::Mat & bin_gr) {
	cv::inRange(hsv, lower_gruen, upper_gruen, bin_gr);
	cv::morphologyEx(bin_gr, bin_gr, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));
}
