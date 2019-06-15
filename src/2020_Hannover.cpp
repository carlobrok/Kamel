#include <iostream>
#include "opencv2/opencv.hpp"
#include "math.h"

#include "config.h"
#include "gruen.h"
#include "line.h"


#ifdef ON_PI
#include "CameraCapture.h"
#include "VideoServer.h"
#include "KamelI2C.h"
#endif

#define VIDEO_NAME video_name[vid_name_idx]

using namespace std;
using namespace cv;

String video_name[4] = {
		"201905_line1.mp4",
		"201905_line2.mp4",
		"201905_line3.mp4",
		"201905_line4.mp4"
};

int vid_name_idx = 0;

// NÄCHSTES VIDEO AUS STRING ARRAY ÖFFNEN

int open_new_vid(VideoCapture & cap) {
	if(cap.isOpened()) {
		cap.release();
	}
	cap.open(VIDEO_NAME);

	if(!cap.isOpened()){
		cout << "Video not opened!" << endl;
		return -2;
	}

	if(vid_name_idx < 3) {
		vid_name_idx++;
	} else {
		vid_name_idx = 0;
	}
	return 1;
}


// MAIN METHODE

int main() {

	// CONTAINER FÜR GEÖFFNETES VIDEO
	VideoCapture cap;

	if(open_new_vid(cap) == -2) {		// wenn kein Video vorhanden abbruch
		return -2;
	}

	Mat img_rgb;			// Matrix mit dem input Bild


	int grstate;


	// Fenster für Output erstellen
	namedWindow("Mask Green", WINDOW_AUTOSIZE);
	namedWindow("Mask SW", WINDOW_AUTOSIZE);
	namedWindow("HSV", WINDOW_AUTOSIZE);
	namedWindow("Input", WINDOW_AUTOSIZE);


	while(1) {
		// Bild einlesen
		cap.read(img_rgb);
		if(img_rgb.empty()) {			// wenn Video zuende, neues öffnen
			cout << "No image!" << endl;

			open_new_vid(cap);

		} else {

			// Bild filter anwenden
			GaussianBlur(img_rgb, img_rgb, Size(15,15),2,2);

			Mat hsv;
			cvtColor(img_rgb, hsv, COLOR_BGR2HSV);

			Mat bin_sw;
			Mat bin_gr;
			vector<Point> line_points;

			separate_gruen(hsv, bin_gr);
			line_calc(img_rgb, hsv, bin_sw, bin_gr, line_points);

			/*float line_radiant_average = 0;
			for (unsigned int i = 0; i < line_points.size(); ++i) {
				float current_radiant = atan2(line_points[i].y - img_rgb.rows, line_points[i].x - img_rgb.cols/2)  * 180 / CV_PI + 90;
				cout << "radiant of point: " << current_radiant << endl;
				line_radiant_average += current_radiant;
			}
			if(line_points.size() > 0) {
				line_radiant_average /= line_points.size();
				cout << "Average: " << line_radiant_average << endl;
			}*/



			grstate = gruen_state(img_rgb, hsv, bin_sw, bin_gr);

			cout << "Gruenstate: " << grstate << " / ";

			switch (grstate) {
			case 0:
				cout << "KEINER";
				break;
			case 1:
				cout << "LINKS";
				break;
			case 2:
				cout << "RECHTS";
				break;
			case 3:
				cout << "BEIDE";
				break;
			}

			cout << endl;

#ifdef ON_PI
			srv.imshow("Input", img_rgb);
			srv.imshow("HSV", hsv);
			srv.imshow("Mask SW", bin_sw);
#else
			imshow("Input", img_rgb);
			imshow("HSV", hsv);
			imshow("Mask SW", bin_sw);

			int key = waitKey(0);
			if(key == 'q') {
				//cout << "Exit by key q" << endl;
				return -1;
			} else if(key == 'n') {
				open_new_vid(cap);
			}
#endif
		}
	}

	return -1;
}
