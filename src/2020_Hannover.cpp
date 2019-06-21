#include <iostream>
#include "opencv2/opencv.hpp"
#include "math.h"
#include <sstream>
#include <cstdio>
#include <fstream>

#include "config.h"
#include "gruen.h"
#include "line.h"
#include "util.h"

#ifdef ON_PI
#include "CameraCapture.h"
#include "VideoServer.h"
#include "KamelI2C.h"
#endif



using namespace std;
using namespace cv;
using this_thread::sleep_for;
using chrono::milliseconds;
using chrono::seconds;

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

/*

bool camera_ready = false;

void drive() {



	do {
		this_thread::sleep_for(chrono::milliseconds(50));
	} while(!camera_ready);
	this_thread::sleep_for(chrono::milliseconds(250));

	while(1) {

		if(grstate != GRUEN_NICHT) {

			// Ein Grünpunkt vorhanden

			// Offset, damit der Roboter sich auf die mitte der Kreuzung ausrichtet, nicht gerade auf den grünen Punkt
			int gr_offset = 0;
			if(grstate == GRUEN_LINKS) {
				gr_offset = 50;
			} else if(grstate == GRUEN_RECHTS) {
				gr_offset = -50;
			}

			// An Grünpunkt ausrichten
			if(grcenter.x + gr_offset < img_rgb.cols/2) {
				writeMotor(motor_fd, MOTOR_RIGHT, MOTOR_FORWARD, 110);
				this_thread::sleep_for(chrono::microseconds(500));
				writeMotor(motor_fd, MOTOR_LEFT, MOTOR_OFF, 0);
			} else if(grcenter.x + gr_offset > img_rgb.cols/2) {
				writeMotor(motor_fd, MOTOR_LEFT, MOTOR_FORWARD, 110);
				this_thread::sleep_for(chrono::microseconds(500));
				writeMotor(motor_fd, MOTOR_RIGHT, MOTOR_OFF, 0);
			} else {
				writeMotor(motor_fd, MOTOR_LEFT, MOTOR_FORWARD, 80);
				this_thread::sleep_for(chrono::microseconds(500));
				writeMotor(motor_fd, MOTOR_RIGHT, MOTOR_FORWARD, 80);
			}

			// Grünpunkt fahren wenn nah genug, ansonsten zurück fahren
			if(grcenter.y > img_rgb.rows / 3 /* && grcenter.x > img_rgb.cols/2 - 200 && grcenter.x < img_rgb.cols/2 + 200*/) {

				// Grünpunkt fahren

				if(grstate == GRUEN_LINKS) {
					writeMotor(motor_fd, MOTOR_RIGHT, MOTOR_FORWARD, 200);
					this_thread::sleep_for(chrono::microseconds(500));
					writeMotor(motor_fd, MOTOR_LEFT, MOTOR_OFF, 0);
					this_thread::sleep_for(chrono::microseconds(1500));

					cout << "GR_LINKS FAHREN!" << endl;

				} else if(grstate == GRUEN_RECHTS) {
					writeMotor(motor_fd, MOTOR_LEFT, MOTOR_FORWARD, 200);
					this_thread::sleep_for(chrono::microseconds(500));
					writeMotor(motor_fd, MOTOR_RIGHT, MOTOR_OFF, 0);
					this_thread::sleep_for(chrono::microseconds(1500));

					cout << "GR_RECHTS FAHREN!" << endl;
				}

			}/* else if(grcenter.y > img_rgb.rows / 2) {
				writeMotor(motor_fd, MOTOR_LEFT, MOTOR_BACKWARD, 200);
				this_thread::sleep_for(chrono::microseconds(500));
				writeMotor(motor_fd, MOTOR_RIGHT, MOTOR_BACKWARD, 200);
				this_thread::sleep_for(chrono::milliseconds(1500));
			}*/


		} else

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

			//		log_timing(tlast, "Motor write: ");

			this_thread::sleep_for(chrono::milliseconds(2));

	}
}

// MAIN METHODE

int main(int argc, char* argv[]) {

	/*
	 * Mats for image with
	 * colorspace rgb and hsv
	 * binary image for line and green points
	 */


	Mat img_rgb;			// input image

	Mat hsv;

	Mat bin_sw;
	Mat bin_gr;

	vector<Point> line_points;
	Point grcenter(0,0);
	int grstate = GRUEN_NICHT;


	int motor_fd = kamelI2Copen(0x08); 					// I2C Schnittstelle vom Motor-Arduino mit der Adresse 0x08 öffnen
	writeMotor(motor_fd, MOTOR_BOTH, MOTOR_OFF, 0); 		// Beide Motoren ausschalten


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

	thread fahren(drive);

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

	camera_ready = true;
	while(1) {
		/*
		 * Wenn das Programm auf dem Pi läuft aus der Kamera auslesen,
		 * wenn es auf dem PC läuft aus Video Array lesen
		 */


		int64 tloop = getTickCount();			// Tickcount for whole loop
		int64 tlast = getTickCount();			// Tickcount to next measurement

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

		// Filter image and convert to hsv
		GaussianBlur(img_rgb, img_rgb, Size(5,5),2,2);		// Gaussian blur to normalize image
		cvtColor(img_rgb, hsv, COLOR_BGR2HSV);		// Convert to HSV and save in Mat hsv

		log_timing(tlast, "Reading, color covert, gauss: ");

		separate_gruen(hsv, bin_gr);
		log_timing(tlast, "Green separation: ");

		line_points.clear();
		line_calc(img_rgb, hsv, bin_sw, bin_gr, line_points);
		log_timing(tlast, "Line calculation: ");

		gruen_calc(img_rgb, hsv, bin_sw, bin_gr, grstate, grcenter);

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

		if(grstate > 0) {
			cout << " " << grcenter;
		}

		cout << endl;

		log_timing(tlast, "Green calc: ");


		if(line_points.size() == 1) {

			if (line_points[1].x > 575) {
				writeMotor(motor_fd, MOTOR_LEFT, MOTOR_FORWARD, 180);
				this_thread::sleep_for(chrono::microseconds(500));
				writeMotor(motor_fd, MOTOR_RIGHT, MOTOR_BACKWARD, 180);
			} else if (line_points[1].x < 65) {
				writeMotor(motor_fd, MOTOR_LEFT, MOTOR_BACKWARD, 180);
				this_thread::sleep_for(chrono::microseconds(500));
				writeMotor(motor_fd, MOTOR_RIGHT, MOTOR_FORWARD, 180);
			} else if (line_points[1].x > 500) {
				writeMotor(motor_fd, MOTOR_LEFT, MOTOR_FORWARD, 160);
				this_thread::sleep_for(chrono::microseconds(500));
				writeMotor(motor_fd, MOTOR_RIGHT, MOTOR_BACKWARD, 80);
			} else if (line_points[1].x < 140) {
				writeMotor(motor_fd, MOTOR_RIGHT, MOTOR_FORWARD, 160);
				this_thread::sleep_for(chrono::microseconds(500));
				writeMotor(motor_fd, MOTOR_LEFT, MOTOR_BACKWARD, 80);
			} else if (line_points[1].x > 400) {
				writeMotor(motor_fd, MOTOR_LEFT, MOTOR_FORWARD, 160);
				this_thread::sleep_for(chrono::microseconds(500));
				writeMotor(motor_fd, MOTOR_RIGHT, MOTOR_BACKWARD, 10);
			} else if (line_points[1].x < 240) {
				writeMotor(motor_fd, MOTOR_RIGHT, MOTOR_FORWARD, 160);
				this_thread::sleep_for(chrono::microseconds(500));
				writeMotor(motor_fd, MOTOR_LEFT, MOTOR_BACKWARD, 10);
			} else if (line_points[1].x > 340) {
				writeMotor(motor_fd, MOTOR_LEFT, MOTOR_FORWARD, 160);
				this_thread::sleep_for(chrono::microseconds(500));
				writeMotor(motor_fd, MOTOR_RIGHT, MOTOR_OFF, 0);
			} else if (line_points[1].x < 300) {
				writeMotor(motor_fd, MOTOR_RIGHT, MOTOR_FORWARD, 160);
				this_thread::sleep_for(chrono::microseconds(500));
				writeMotor(motor_fd, MOTOR_LEFT, MOTOR_OFF, 0);
			} else {
				writeMotor(motor_fd, MOTOR_BOTH, MOTOR_FORWARD, 90);
			}

			/*float rad = line_radiant(line_points[0], img_rgb.rows, img_rgb.cols);

					//			ostringstream s;
					//			s << "Rad: " << rad;

					//			putText(img_rgb, s.str(), line_points[0],  FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255),1);

					//			cout << "Line radiant: " << rad << endl;

					if(rad > 50) {
						writeMotor(motor_fd, MOTOR_LEFT, MOTOR_FORWARD, 150);
						this_thread::sleep_for(chrono::microseconds(500));
						writeMotor(motor_fd, MOTOR_RIGHT, MOTOR_BACKWARD, 150);
					} else if(rad < -50) {
						writeMotor(motor_fd, MOTOR_LEFT, MOTOR_BACKWARD, 150);
						this_thread::sleep_for(chrono::microseconds(500));
						writeMotor(motor_fd, MOTOR_RIGHT, MOTOR_FORWARD, 150);
					} else if(rad > 20) {
						writeMotor(motor_fd, MOTOR_LEFT, MOTOR_FORWARD, 120);
						this_thread::sleep_for(chrono::microseconds(500));
						writeMotor(motor_fd, MOTOR_RIGHT, MOTOR_BACKWARD, 80);
					} else if(rad < -20) {
						writeMotor(motor_fd, MOTOR_LEFT, MOTOR_BACKWARD, 80);
						this_thread::sleep_for(chrono::microseconds(500));
						writeMotor(motor_fd, MOTOR_RIGHT, MOTOR_FORWARD, 120);
					} else if(rad > 3) {
						writeMotor(motor_fd, MOTOR_LEFT, MOTOR_FORWARD, 120);
						this_thread::sleep_for(chrono::microseconds(500));
						writeMotor(motor_fd, MOTOR_RIGHT, MOTOR_FORWARD, 100);
					} else if(rad < -3) {
						writeMotor(motor_fd, MOTOR_LEFT, MOTOR_FORWARD, 100);
						this_thread::sleep_for(chrono::microseconds(500));
						writeMotor(motor_fd, MOTOR_RIGHT, MOTOR_FORWARD, 120);
					} else {
						writeMotor(motor_fd, MOTOR_BOTH, MOTOR_FORWARD, 100);
					}*/

		} else {
			writeMotor(motor_fd, MOTOR_BOTH, MOTOR_FORWARD, 90);
		}


#ifdef ON_PI
		srv.imshow("Input", img_rgb);
		srv.imshow("HSV", hsv);
		srv.imshow("Mask SW", bin_sw);
		srv.imshow("Mask Green", bin_gr);
		srv.update();


#else
		imshow("Input", img_rgb);
		imshow("HSV", hsv);
		imshow("Mask SW", bin_sw);
		imshow("Mask Green", bin_gr);

		int key = waitKey(0);
		if(key == 'q') {
			//cout << "Exit by key q" << endl;
			return -1;
		} else if(key == 'n') {
			open_new_vid(cap);
		}
#endif

		std::cout << "Sending images: " << (getTickCount() - tlast) / getTickFrequency() * 1000.0 << " ms" << endl;
		std::cout << "Processing took: " << (getTickCount() - tloop) / getTickFrequency() * 1000.0 << " ms; FPS: " <<  cv::getTickFrequency() / (cv::getTickCount() - tloop) << endl << endl;

		//		log_sensordata(line_points, grstate, grcenter, img_rgb);
	}

	return -1;
}
