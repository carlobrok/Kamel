#include <iostream>
#include "opencv2/opencv.hpp"
#include "math.h"
#include <sstream>
#include <cstdio>
#include <fstream>
#include <mutex>

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

Mat img_rgb;			// input image

vector<Point> line_points;
Point grcenter(0,0);
int grstate = GRUEN_NICHT;

mutex mutex_linepoints;


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

void drive() {

	int motor_fd = kamelI2Copen(0x08); 					// I2C Schnittstelle vom Motor-Arduino mit der Adresse 0x08 öffnen
	writeMotor(motor_fd, MOTOR_BOTH, MOTOR_OFF, 0); 		// Beide Motoren ausschalten

	vector<Point> last_line_points = line_points;

	while(1) {

		//		cout << "In thread" << endl;

		mutex_linepoints.lock();

		if (grstate == GRUEN_BEIDE) {

			int64 last_gruen = getTickCount();
			mutex_linepoints.unlock();
			while (grstate != GRUEN_NICHT) {
				if (grcenter.y > 350) {
					vor(180);
					delay(500);

					/*motorLeft(false, 150);
		        motorRight(true, 150);
		        delay(3000);*/

					turn_angle(180, ypr[2]);
					aus();
					delay_data(500);

					break;
				} else if (grcenter.x < 310) {
					motorLeft();
					motorRight(true, 90);
				} else if (grcenter.x > 330) {
					motorLeft(true, 90);
					motorRight();
				} else {
					vor(50);
				}
			}
			mutex_linepoints.lock();
		} else if (grstate == GRUEN_LINKS && getGrPointY() > 480 - 150) {
			vor(150);
			delay(300);
			motorLeft(false, 190);
			motorRight(true, 150);
			delay_data(500);
		} else if (grstate == GRUEN_RECHTS && getGrPointY() > 480 - 150) {
			vor(150);
			delay(300);
			motorRight(false, 190);
			motorLeft(true, 150);
			delay_data(500);
		} else if(line_points.size() == 1) {
//			cout << "Different value -> check motor output for line" << endl;

			if (line_points[0].x > 575) {
				writeMotor(motor_fd, MOTOR_LEFT, MOTOR_FORWARD, 190);
				this_thread::sleep_for(chrono::microseconds(500));
				writeMotor(motor_fd, MOTOR_RIGHT, MOTOR_BACKWARD, 160);
			} else if (line_points[0].x < 65) {
				writeMotor(motor_fd, MOTOR_LEFT, MOTOR_BACKWARD, 160);
				this_thread::sleep_for(chrono::microseconds(500));
				writeMotor(motor_fd, MOTOR_RIGHT, MOTOR_FORWARD, 190);
			} else if (line_points[0].x > 500) {
				writeMotor(motor_fd, MOTOR_LEFT, MOTOR_FORWARD, 160);
				this_thread::sleep_for(chrono::microseconds(500));
				writeMotor(motor_fd, MOTOR_RIGHT, MOTOR_BACKWARD, 80);
			} else if (line_points[0].x < 140) {
				writeMotor(motor_fd, MOTOR_RIGHT, MOTOR_FORWARD, 160);
				this_thread::sleep_for(chrono::microseconds(500));
				writeMotor(motor_fd, MOTOR_LEFT, MOTOR_BACKWARD, 80);
			} else if (line_points[0].x > 400) {
				writeMotor(motor_fd, MOTOR_LEFT, MOTOR_FORWARD, 160);
				this_thread::sleep_for(chrono::microseconds(500));
				writeMotor(motor_fd, MOTOR_RIGHT, MOTOR_BACKWARD, 10);
			} else if (line_points[0].x < 240) {
				writeMotor(motor_fd, MOTOR_RIGHT, MOTOR_FORWARD, 160);
				this_thread::sleep_for(chrono::microseconds(500));
				writeMotor(motor_fd, MOTOR_LEFT, MOTOR_BACKWARD, 10);
			} else if (line_points[0].x > 340) {
				writeMotor(motor_fd, MOTOR_LEFT, MOTOR_FORWARD, 160);
				this_thread::sleep_for(chrono::microseconds(500));
				writeMotor(motor_fd, MOTOR_RIGHT, MOTOR_OFF, 0);
			} else if (line_points[0].x < 300) {
				writeMotor(motor_fd, MOTOR_RIGHT, MOTOR_FORWARD, 160);
				this_thread::sleep_for(chrono::microseconds(500));
				writeMotor(motor_fd, MOTOR_LEFT, MOTOR_OFF, 0);
			} else {
				writeMotor(motor_fd, MOTOR_BOTH, MOTOR_FORWARD, 90);
			}

		} else if(line_points.size() == 0 && last_line_points.size() == 1) {
			cout << "Lost line last:" << last_line_points << "  current:" << line_points;

			if (last_line_points[0].x > 575) {
				cout << "  correction right" << endl;
				writeMotor(motor_fd, MOTOR_LEFT, MOTOR_FORWARD, 160);
				this_thread::sleep_for(chrono::microseconds(500));
				writeMotor(motor_fd, MOTOR_RIGHT, MOTOR_BACKWARD, 120);

				while(line_points.size() == 0 || (line_points.size() == 1 && line_points[0].x > img_rgb.cols / 2 + 100)) {
					mutex_linepoints.unlock();
					this_thread::sleep_for(chrono::milliseconds(5));
					mutex_linepoints.lock();
				}

				mutex_linepoints.unlock();
				writeMotor(motor_fd, MOTOR_BOTH, MOTOR_BACKWARD, 100);
				this_thread::sleep_for(chrono::milliseconds(500));
				writeMotor(motor_fd, MOTOR_BOTH, MOTOR_OFF, 100);
				mutex_linepoints.lock();

			} else if (last_line_points[0].x < 65) {
				cout << "  correction left" << endl;
				writeMotor(motor_fd, MOTOR_LEFT, MOTOR_BACKWARD, 120);
				this_thread::sleep_for(chrono::microseconds(500));
				writeMotor(motor_fd, MOTOR_RIGHT, MOTOR_FORWARD, 160);
				while(line_points.size() == 0 || (line_points.size() == 1 && line_points[0].x < img_rgb.cols / 2 - 100)) {
					mutex_linepoints.unlock();
					this_thread::sleep_for(chrono::milliseconds(5));
					mutex_linepoints.lock();
				}

				mutex_linepoints.unlock();
				writeMotor(motor_fd, MOTOR_BOTH, MOTOR_BACKWARD, 100);
				this_thread::sleep_for(chrono::milliseconds(500));
				writeMotor(motor_fd, MOTOR_BOTH, MOTOR_OFF, 100);
				mutex_linepoints.lock();

			} else {
				writeMotor(motor_fd, MOTOR_BOTH, MOTOR_FORWARD, 90);
				cout << endl;
			}
		} else {
			writeMotor(motor_fd, MOTOR_BOTH, MOTOR_FORWARD, 90);
		}

		last_line_points = line_points;
		mutex_linepoints.unlock();

		this_thread::sleep_for(chrono::milliseconds(5));
	}
}



// MAIN METHODE

int main(int argc, char* argv[]) {

	/*
	 * Mats for image with
	 * colorspace rgb and hsv
	 * binary image for line and green points
	 */

	Mat hsv;

	Mat bin_sw;
	Mat bin_gr;

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


	thread drive_thread(drive);
	//	drive_thread.join();

	cout << "After thread start" << endl;

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

//		log_timing(tlast, "Reading, color covert, gauss: ");

		separate_gruen(hsv, bin_gr);
//		log_timing(tlast, "Green separation: ");

		vector<Point> m_line_points;

		line_calc(img_rgb, hsv, bin_sw, bin_gr, m_line_points);

		mutex_linepoints.lock();
		line_points = m_line_points;
		mutex_linepoints.unlock();


//		log_timing(tlast, "Line calculation: ");

		gruen_calc(img_rgb, hsv, bin_sw, bin_gr, grstate, grcenter);

		/*cout << "Gruenstate: " << grstate << " / ";
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

		cout << endl;*/

//		log_timing(tlast, "Green calc: ");

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

//		std::cout << "Sending images: " << (getTickCount() - tlast) / getTickFrequency() * 1000.0 << " ms" << endl;
//		std::cout << "Processing took: " << (getTickCount() - tloop) / getTickFrequency() * 1000.0 << " ms; FPS: " <<  cv::getTickFrequency() / (cv::getTickCount() - tloop) << endl << endl;

		//		log_sensordata(line_points, grstate, grcenter, img_rgb);
	}

	return -1;
}
