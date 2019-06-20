#include "gruen.h"
#include "util.h"
#include "config.h"

Point gruen_center(vector <Point> cont1, vector <Point> cont2) {
	Moments m1 = moments(cont1);
	Moments m2 = moments(cont2);

	return Point((m1.m10 / m1.m00 + m2.m10 / m2.m00) / 2, (m1.m01 / m1.m00 + m2.m01 / m2.m00) / 2);
}

Point gruen_center(Point p1, Point p2) {
	return Point((p1.x + p2.x) / 2, (p1.y + p2.y) / 2);
}

Point gruen_center(vector<Point> contour) {
	Moments m = moments(contour);
	return Point(m.m10 / m.m00, m.m01 / m.m00);
}

Point rotate_point(Point & origin_point, Point & rotating_point, float rotation, float length_factor) {
	float rad = CV_PI / 180 * rotation;

	Point new_point;

	new_point.x = ((rotating_point.x - origin_point.x) * cos(rad) - (rotating_point.y - origin_point.y) * sin(rad)) * length_factor + origin_point.x;
	new_point.y = ((rotating_point.x - origin_point.x) * sin(rad) + (rotating_point.y - origin_point.y) * cos(rad)) * length_factor + origin_point.y;

	return new_point;
}


/*
Point rotated_point_lenght(Point & origin_point, float rotation, float length) {
	float rad = CV_PI / 180 * rotation;

	Point temporary_point = Point(origin_point.x, origin_point.y + length);
	Point new_point;

	new_point.x = (temporary_point.x - origin_point.x) * cos(rad) - (temporary_point.y - origin_point.y) * sin(rad) + origin_point.x;
	new_point.y = (temporary_point.x - origin_point.x) * sin(rad) + (temporary_point.y - origin_point.y) * cos(rad) + origin_point.y;

	return new_point;
}*/


/*
 * Checkt bei einem grünen Punkt vertikal nach oben den anteil der Schwarzen linie.
 */

int gruen_check_normal(Mat & img_rgb, Mat & bin_sw, Mat & bin_gr, vector<Point> contour) {

	Moments m = moments(contour);
	float point_distance = sqrt(m.m00) * 1.2;

	Point mittegr = gruen_center(contour);
	Point top_checkpoint = mittegr;
	top_checkpoint.y -= point_distance;

	if(ratio_black_points(mittegr, top_checkpoint, bin_sw, bin_gr, img_rgb) > 0.6) {
		Point left_checkpoint = mittegr;
		Point right_checkpoint = mittegr;
		left_checkpoint.x -= point_distance;
		right_checkpoint.x += point_distance;

		int ratio_left = ratio_black_points(mittegr, left_checkpoint, bin_sw, bin_gr, img_rgb);
		int ratio_right = ratio_black_points(mittegr, right_checkpoint, bin_sw, bin_gr, img_rgb);

		if(ratio_left > ratio_right) {
			return GRUEN_RECHTS;
		} else if(ratio_right > ratio_left) {
			return GRUEN_LINKS;
		}
	}

	return GRUEN_NICHT;
}

void gruen_calc(Mat & img_rgb, Mat & img_hsv, Mat & bin_sw, Mat & bin_gr, int & grstate, Point & grcenter) {

	vector< vector<Point> > contg;

	//morphologyEx(img_bingr, img_bingr, MORPH_OPEN, getStructuringElement(MORPH_ELLIPSE, Size(10,10)));
	findContours(bin_gr, contg, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

	for(unsigned int i = 0; i < contg.size(); i++) {
		Moments m = moments(contg[i]);
		if(m.m00 < 800) {
			contg.erase(contg.begin() + i--);
		}
	}

#ifdef VISUAL_DEBUG
	drawContours(img_rgb, contg, -1, Scalar(50, 230, 50), 1);
#endif

	if(contg.size() > 1) {

		/*
		 * Mehr als 1 Grüner Punkt
		 */

		for(unsigned int i = 0; i < contg.size()-1; i++) {
			Moments m1 = moments(contg[i]), m2 = moments(contg[i+1]);
			//cout << "m00: " << m1.m00 << " / " << m2.m00 << endl;
			if(abs(m1.m00 - m2.m00) < 1500 || (m1.m00 > 3500 && m2.m00 > 3500)) {
				Point p1 = Point(m1.m10 / m1.m00, m1.m01 / m1.m00);
				Point p2 = Point(m2.m10 / m2.m00, m2.m01 / m2.m00);

#ifdef VISUAL_DEBUG
				line(img_rgb, p1, p2, Scalar(255,0,100), 2);
#endif

				if(abs(p1.y-p2.y < sqrt((m1.m00 + m2.m00) / 2))) {

					if(p1.x > p2.x) {
						Point p_tmp = p2;
						p2 = p1;
						p1 = p_tmp;
					}

					Point p1_new = rotate_point(p1, p2, -135, 0.8);
					Point p2_new = rotate_point(p2, p1, 135, 0.8);

#ifdef VISUAL_DEBUG
					circle(img_rgb, p1, 2, Scalar(0,255,0), 2);
					circle(img_rgb, p2, 2, Scalar(0,255,0), 2);
					circle(img_rgb, p1_new, 2, Scalar(0,0,255), 2);
					circle(img_rgb, p2_new, 2, Scalar(0,0,255), 2);
#endif

					float ratio_left_point = ratio_black_points(p1, p1_new, bin_sw, bin_gr, img_rgb);
					float ratio_right_point = ratio_black_points(p2, p2_new, bin_sw, bin_gr, img_rgb);

#ifdef DEBUG_GRUEN
					cout << "Ratio Blackpoints:	Links: " << ratio_left_point
							<< "		Rechts: " << ratio_right_point << endl;
#endif

					if(ratio_left_point > 0.4 && ratio_right_point > 0.4) {
						grstate = GRUEN_BEIDE;
						grcenter = gruen_center(p1_new, p2_new);
						return;
					}
				}
			}
		}

		int index_nearest = 0;
		double nearest_distance = -1;
		Point nearest_point;

		for(unsigned int i = 0; i < contg.size(); i++) {
			Point center = gruen_center(contg[i]);
			float distance_to_point = sqrt((center.x-img_rgb.cols/2)*(center.x-img_rgb.cols/2) + (center.y-img_rgb.rows)*(center.y-img_rgb.rows));

			if(distance_to_point < nearest_distance || nearest_distance == -1) {
				nearest_distance = distance_to_point;
				index_nearest = i;
				nearest_point = center;
			}
		}

		grstate = gruen_check_normal(img_rgb, bin_sw, bin_gr, contg[index_nearest]);
		grcenter = nearest_point;
		return;

	} else if(contg.size() == 1){
		/*
		 * Nur 1 Grüner Punkt
		 */
		grstate = gruen_check_normal(img_rgb, bin_sw, bin_gr, contg[0]);
		return;

	}

	grstate = GRUEN_NICHT;
	return;
}


/*
 * Alle bereiche, wo grün vorhanden ist in bin_gr schreiben.
 */

void separate_gruen(Mat & hsv, Mat & bin_gr) {
	inRange(hsv, LOWER_GREEN, UPPER_GREEN, bin_gr);
	morphologyEx(bin_gr, bin_gr, MORPH_OPEN, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
}



/* Checkt, wie viele schwarze Punkte sich zwischen origin und destination befinden.
 *
 * Maximaler Wert: Anzahl an checks, also NUM_ITERATIONS_BLACK_POINTS
 *
 * Rückgabewert: Verhältnis der Punkte, die Schwarz waren.
 * Wert zwischen 0 und 1
 */


float ratio_black_points(Point & origin, Point & destination, Mat & bin_sw, Mat & bin_gr, Mat & img_rgb) {
	int black_pixels = 0;
	int green_pixels = 0;
	Point cur_pixel;
	int color_sw;
	int color_gr;

	for(int i = 1; i < NUM_ITERATIONS_BLACK_POINTS; i++) {
		cur_pixel.x = origin.x + (float)i/30 * (destination.x - origin.x);
		cur_pixel.y = origin.y + (float)i/30 * (destination.y - origin.y);

		if(inMat(cur_pixel, bin_sw.cols, bin_sw.rows)) {

			color_sw = (int)bin_sw.at<uchar>(cur_pixel);
			color_gr = (int)bin_gr.at<uchar>(cur_pixel);

			//cout << "gr_punkt: " << id << "; Pixel: " << cur_pixel << " bin_colors:  sw:" << color_sw << " gr:" << color_gr << endl;

			if(color_sw == 255 && color_gr == 0) {
				black_pixels++;

	#ifdef VISUAL_DEBUG
				circle(img_rgb, cur_pixel, 1, Scalar(50, 50, 50), 2);
	#endif
			} else if(color_gr == 255) {
				green_pixels++;
	#ifdef VISUAL_DEBUG
				circle(img_rgb, cur_pixel, 1, Scalar(20, 150, 20), 2);
	#endif
			} else {
	#ifdef VISUAL_DEBUG
				circle(img_rgb, cur_pixel, 1, Scalar(200,200,200), 2);
	#endif
			}
		} else {
			i = NUM_ITERATIONS_BLACK_POINTS;
		}
	}

	return (float) black_pixels / (NUM_ITERATIONS_BLACK_POINTS - 1 - green_pixels);
}
