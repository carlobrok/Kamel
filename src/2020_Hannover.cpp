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



using namespace std;
using namespace cv;

#ifndef ON_PI
#define VIDEO_NAME video_name[vid_name_idx]

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

#endif


// MAIN METHODE

int main() {

	Mat img_rgb;			// input Bild
	int grstate;

/*
 * Umgebungen unterscheiden durch Precompiler:
 *
 * Pi:
 *  - CameraCapture als Videoquelle mit neuem Thread
 *  - VideoServer als Bildausgabe über IP
 *
 * PC:
 *  - VideoCapture für Videoinput
 *  - kein Videoserver, lokale Bildausgabe
 */

#ifdef ON_PI

	CameraCapture cam(0);
	VideoServer srv;

	cam.set(cv::CAP_PROP_FPS, 30);
	cam.set(cv::CAP_PROP_FRAME_WIDTH, 640);
	cam.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

	//Füge Fenster zum Server hinzu
	srv.namedWindow("Mask Green");
	srv.namedWindow("Mask SW");
	srv.namedWindow("HSV");
	srv.namedWindow("Input");


#else
	// CONTAINER FÜR GEÖFFNETES VIDEO
	VideoCapture cap;

	if(open_new_vid(cap) == -2) {		// wenn kein Video vorhanden abbruch
		return -2;
	}

	// Fenster für Output erstellen
	namedWindow("Mask Green", WINDOW_AUTOSIZE);
	namedWindow("Mask SW", WINDOW_AUTOSIZE);
	namedWindow("HSV", WINDOW_AUTOSIZE);
	namedWindow("Input", WINDOW_AUTOSIZE);
#endif


	int motor_fd = kamelI2Copen(0x08);

	writeMotor(motor_fd, MOTOR_BOTH, MOTOR_OFF, 0);


	while(1) {				// Dauerschleife
		/*
		 * Wenn das Programm auf dem Pi läuft aus der Kamera auslesen,
		 * wenn es auf dem PC läuft aus Video Array lesen
		 */

		int64 tloop = getTickCount();
		int64 tlast = getTickCount();

#ifdef ON_PI

		while(!cam.read(img_rgb)){}

#else
		// Bild einlesen
		cap.read(img_rgb);
		if(img_rgb.empty()) {			// wenn Video zuende, neues öffnen
			cout << "No image!" << endl;

			open_new_vid(cap);
			cap.read(img_rgb);
		}
#endif

		// Bild filter anwenden
		GaussianBlur(img_rgb, img_rgb, Size(5,5),2,2);

		Mat hsv;
		cvtColor(img_rgb, hsv, COLOR_BGR2HSV);

		std::cout << "Reading, color covert, gauss: " << (getTickCount() - tlast) / getTickFrequency() * 1000.0 << " ms" << endl;
		tlast = getTickCount();

		Mat bin_sw;
		Mat bin_gr;
		vector<Point> line_points;

		separate_gruen(hsv, bin_gr);
		line_calc(img_rgb, hsv, bin_sw, bin_gr, line_points);

		std::cout << "Line calculation: " << (getTickCount() - tlast) / getTickFrequency() * 1000.0 << " ms" << endl;
		tlast = getTickCount();

		if(line_points.size() == 1) {

			float rad = line_radiant(line_points[0], img_rgb.rows, img_rgb.cols);

			cout << "Line radiant: " << rad << endl;

			if(rad > 55) {
				writeMotor(motor_fd, MOTOR_LEFT, MOTOR_FORWARD, 150);
				this_thread::sleep_for(chrono::microseconds(500));
				writeMotor(motor_fd, MOTOR_RIGHT, MOTOR_BACKWARD, 150);
			} else if(rad < -55) {
				writeMotor(motor_fd, MOTOR_LEFT, MOTOR_BACKWARD, 150);
				this_thread::sleep_for(chrono::microseconds(500));
				writeMotor(motor_fd, MOTOR_RIGHT, MOTOR_FORWARD, 150);
			} else if(rad > 35) {
				writeMotor(motor_fd, MOTOR_LEFT, MOTOR_FORWARD, 120);
				this_thread::sleep_for(chrono::microseconds(500));
				writeMotor(motor_fd, MOTOR_RIGHT, MOTOR_BACKWARD, 80);
			} else if(rad < -35) {
				writeMotor(motor_fd, MOTOR_LEFT, MOTOR_BACKWARD, 80);
				this_thread::sleep_for(chrono::microseconds(500));
				writeMotor(motor_fd, MOTOR_RIGHT, MOTOR_FORWARD, 120);
			} else if(rad > 10) {
				writeMotor(motor_fd, MOTOR_LEFT, MOTOR_FORWARD, 120);
				this_thread::sleep_for(chrono::microseconds(500));
				writeMotor(motor_fd, MOTOR_RIGHT, MOTOR_FORWARD, 100);
			} else if(rad < -10) {
				writeMotor(motor_fd, MOTOR_LEFT, MOTOR_FORWARD, 100);
				this_thread::sleep_for(chrono::microseconds(500));
				writeMotor(motor_fd, MOTOR_RIGHT, MOTOR_FORWARD, 120);
			} else {
				writeMotor(motor_fd, MOTOR_BOTH, MOTOR_FORWARD, 100);
			}

		} else if(line_points.size() > 1) {
			writeMotor(motor_fd, MOTOR_BOTH, MOTOR_FORWARD, 100);
		}

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

		std::cout << "Green calc: " << (getTickCount() - tlast) / getTickFrequency() * 1000.0 << " ms" << endl;
		tlast = getTickCount();

#ifdef ON_PI
		srv.imshow("Input", img_rgb);
		srv.imshow("HSV", hsv);
		srv.imshow("Mask SW", bin_sw);
		srv.update();



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

		std::cout << "Sending images: " << (getTickCount() - tlast) / getTickFrequency() * 1000.0 << " ms" << endl;
		std::cout << "Processing took: " << (getTickCount() - tloop) / getTickFrequency() * 1000.0 << " ms; FPS: " <<  cv::getTickFrequency() / (cv::getTickCount() - tloop) << endl;
	}

	return -1;
}
