#include "opencv2/opencv.hpp"
#include "math.h"

#include <iostream>
#include <sstream>
#include <cstdio>
#include <fstream>

#include <vector>
#include <array>
#include <mutex>

#include <boost/circular_buffer.hpp>

#include "Logger.h"
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

namespace lvl = spdlog::level;
Logger debug_lg("debug");

Mat img_rgb;			// input image

vector<Point> line_points;
Point grcenter(0,0);
int grstate = GRUEN_NICHT;

mutex line_mutex;
mutex green_mutex;


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

void m_drive() {

	Logger behavior_lg("behavior");
	Logger sensor_lg("sensors");

	debug_lg << "init i2c devices" << lvl::debug;

	// init i2c devices

	int motor_fd = kamelI2Copen(0x08); 					// I2C Schnittstelle vom Motor-Arduino mit der Adresse 0x08 öffnen
	/*if(motor_fd == -1) {
//		BOOST_LOG(debug_lg.m_lg) << "[" << cur_sec() << "] error initializing motor arduino";
		return;
	}*/
	setMotorState(motor_fd, MOTOR_BOTH, MOTOR_OFF); 		// Beide Motoren ausschalten

	int sensor_fd = kamelI2Copen(0x09);
/*	if(sensor_fd == -1) {
//		BOOST_LOG(debug_lg.m_lg) << "[" << cur_sec() << "] error initializing sensor arduino";
		return;
	}*/

	debug_lg << "successfully initialized i2c devices" << lvl::debug;
	debug_lg << "init sensor / camera variables" << lvl::debug;

	// init line variables

	unique_lock<mutex> line_lock(line_mutex);
	vector<Point> m_line_points = line_points;
	boost::circular_buffer<vector<Point>> last_line_points(50);
	line_lock.unlock();


	// init green variables

	unique_lock<mutex> green_lock(green_mutex);
	Point m_grcenter = grcenter;
	int m_grstate = GRUEN_NICHT;
	boost::circular_buffer<Point> last_grcenter(50);
	green_lock.unlock();


	// init digital sensor variables

	bool digital_sensor_data[8];
	array<boost::circular_buffer<bool>, 8> last_digital_data;
//	array<boost::circular_buffer<double>, 8> last_digital_time;
	for (auto& cb : last_digital_data) {
		cb.resize(100);
	}
//	for (auto& cb : last_digital_time) {
//		cb.resize(100);
//	}


	// init analog sensor variables

	uint16_t analog_sensor_data[1];
	boost::circular_buffer<uint16_t> last_analog_data(100);
//	boost::circular_buffer<double> last_analog_time(100);


	// Init IMU variables

	/*float imu_data[3];
	array<boost::circular_buffer<float>, 3> last_imu_data;

//	array<boost::circular_buffer<double>, 3> last_imu_time;
	for (auto& cb : last_imu_data) {
		cb.resize(100);
	}*/
//	for (auto& cb : last_imu_time) {
//		cb.resize(100);
//	}

	debug_lg << "successfully initialized sensor / camera variables" << lvl::debug;

	while(1) {

		// sensor value updating ======================

		// lock variables
		line_lock.lock();
		green_lock.lock();

		// update values
		m_line_points = line_points;
		m_grstate = grstate;
		m_grcenter = grcenter;

		// push_front last values - recent value is item [0]
		last_line_points.push_front(m_line_points);
		last_grcenter.push_front(m_grcenter);

		// unlock variables
		line_lock.unlock();
		green_lock.unlock();

		// update arduino sensor data
//		getSensorData(sensor_fd, digital_sensor_data, analog_sensor_data);

		// push_front last values - recent value is item [0]
		/*last_analog_data.push_front(analog_sensor_data[0]);
		for(int i = 0; i < 8; i++) {
			last_digital_data[i].push_front(digital_sensor_data[i]);
		}*/


		// main part: drive decisions	=================================

		if (m_grstate == GRUEN_BEIDE) {

			debug_lg << "green point BOTH" << lvl::info;

			while (grstate != GRUEN_NICHT) {
				if (grcenter.y > 350) {
					setMotorDirPwm(motor_fd, MOTOR_BOTH, MOTOR_FORWARD, 180);
					thread_delay(500);

					setMotorDirPwmBoth(motor_fd, MOTOR_BACKWARD, 200, MOTOR_FORWARD, 200);
					thread_delay(3000);
					setMotorState(motor_fd, MOTOR_BOTH, MOTOR_OFF);
					thread_delay(500);

					break;
				} else if (grcenter.x < 310) {
					setMotorDirPwmBoth(motor_fd, MOTOR_FORWARD, 0, MOTOR_FORWARD, 90);
				} else if (grcenter.x > 330) {
					setMotorDirPwmBoth(motor_fd, MOTOR_FORWARD, 90, MOTOR_FORWARD, 0);
				} else {
					setMotorDirPwm(motor_fd, MOTOR_BOTH, MOTOR_FORWARD, 60);
				}
			}

		} else if (m_grstate == GRUEN_LINKS && grcenter.y > 480 - 150) {

			debug_lg << "green point LEFT" << lvl::info;

			setMotorDirPwm(motor_fd, MOTOR_BOTH, MOTOR_FORWARD, 150);
			thread_delay(300);
			setMotorDirPwmBoth(motor_fd, MOTOR_BACKWARD, 190, MOTOR_FORWARD, 150);
			thread_delay(500);

		} else if (m_grstate == GRUEN_RECHTS && grcenter.y > 480 - 150) {

			debug_lg << "green point RIGHT" << lvl::info;

			setMotorDirPwm(motor_fd, MOTOR_BOTH, MOTOR_FORWARD, 150);
			thread_delay(300);
			setMotorDirPwmBoth(motor_fd, MOTOR_FORWARD, 150, MOTOR_BACKWARD, 190);
			thread_delay(500);

		} else if(m_line_points.size() == 1) {

			debug_lg << "single Line point: " << m_line_points[0] << lvl::info;
			//			cout << "Different value -> check motor output for line" << endl;

			if (m_line_points[0].x > 575) {
				setMotorDirPwmBoth(motor_fd, MOTOR_FORWARD, 190, MOTOR_BACKWARD, 160);
			} else if (m_line_points[0].x < 65) {
				setMotorDirPwmBoth(motor_fd, MOTOR_BACKWARD, 160, MOTOR_FORWARD, 190);
			} else if (m_line_points[0].x > 500) {
				setMotorDirPwmBoth(motor_fd, MOTOR_FORWARD, 160, MOTOR_BACKWARD, 80);
			} else if (m_line_points[0].x < 140) {
				setMotorDirPwmBoth(motor_fd, MOTOR_BACKWARD, 80, MOTOR_FORWARD, 160);
			} else if (m_line_points[0].x > 400) {
				setMotorDirPwmBoth(motor_fd, MOTOR_FORWARD, 160, MOTOR_BACKWARD, 10);
			} else if (m_line_points[0].x < 240) {
				setMotorDirPwmBoth(motor_fd, MOTOR_BACKWARD, 10, MOTOR_FORWARD, 160);
			} else if (m_line_points[0].x > 340) {
				setMotorDirPwmBoth(motor_fd, MOTOR_FORWARD, 160, MOTOR_OFF, 0);
			} else if (m_line_points[0].x < 300) {
				setMotorDirPwmBoth(motor_fd, MOTOR_OFF, 0, MOTOR_FORWARD, 160);
			} else {
				setMotorState(motor_fd, MOTOR_BOTH, MOTOR_FORWARD_NORMAL);
			}

		} else if(m_line_points.size() == 0 && last_line_points[1].size() == 1) {
			debug_lg << "lost line, last line singe line point: " << last_line_points[1][0];

			if (last_line_points[1][0].x > 575) {
				debug_lg << " -  correction right" << lvl::warn;

				setMotorDirPwmBoth(motor_fd, MOTOR_FORWARD, 160, MOTOR_BACKWARD, 120);

				while(m_line_points.size() == 0 || (m_line_points.size() == 1 && m_line_points[0].x > img_rgb.cols / 2 + 100)) {
					thread_delay(5);
					line_lock.lock();
					m_line_points  = line_points;
					line_lock.unlock();
				}

				setMotorDirPwm(motor_fd, MOTOR_BOTH, MOTOR_BACKWARD, 100);
				thread_delay(500);
				setMotorState(motor_fd, MOTOR_BOTH, MOTOR_OFF);

			} else if (last_line_points[1][0].x < 65) {
				cout << " -   correction left" << lvl::warn;

				setMotorDirPwmBoth(motor_fd, MOTOR_BACKWARD, 120, MOTOR_FORWARD, 160);

				while(m_line_points.size() == 0 || (m_line_points.size() == 1 && m_line_points[0].x < img_rgb.cols / 2 - 100)) {
					thread_delay(5);
					line_lock.lock();
					m_line_points  = line_points;
					line_lock.unlock();
				}

				setMotorDirPwm(motor_fd, MOTOR_BOTH, MOTOR_BACKWARD, 100);
				thread_delay(500);
				setMotorState(motor_fd, MOTOR_BOTH, MOTOR_OFF);

			} else {
				setMotorState(motor_fd, MOTOR_BOTH, MOTOR_FORWARD_NORMAL);
				debug_lg << lvl::warn;
			}
		} else {
			setMotorState(motor_fd, MOTOR_BOTH, MOTOR_FORWARD_NORMAL);
			debug_lg << "driving forward, " << m_line_points.size() << " line points" << lvl::info;



		}

		thread_delay(1);
	}
}




void image_processing() {

	Logger camera_lg("camera");

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

	vector<Point> m_line_points;
	Point m_grcenter(0,0);
	int m_grstate = GRUEN_NICHT;

	while(1) {

//		cout << "Image processing thread running" << endl;

		int64 tloop = getTickCount();			// Tickcount for whole loop

		/*
		 * Wenn das Programm auf dem Pi läuft aus der Kamera auslesen,
		 * wenn es auf dem PC läuft aus Video Array lesen
		 */
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

		std::cout << "Green separation: " << (getTickCount() - tlast) / getTickFrequency() * 1000.0 << " ms" << endl;
		tlast = getTickCount();

		line_calc(img_rgb, hsv, bin_sw, bin_gr, line_points);

		std::cout << "Line calculation: " << (getTickCount() - tlast) / getTickFrequency() * 1000.0 << " ms" << endl;
		tlast = getTickCount();


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


		if(line_points.size() == 1) {

			float rad = line_radiant(line_points[0], img_rgb.rows, img_rgb.cols);

			ostringstream s;
			s << "Rad: " << rad;

			putText(img_rgb, s.str(), line_points[0],  FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255),1);

			cout << "Line radiant: " << rad << endl;

			if(rad > 50) {
				writeMotor(motor_fd, MOTOR_LEFT, MOTOR_FORWARD, 150);
				this_thread::sleep_for(chrono::microseconds(750));
				writeMotor(motor_fd, MOTOR_RIGHT, MOTOR_BACKWARD, 150);
			} else if(rad < -50) {
				writeMotor(motor_fd, MOTOR_LEFT, MOTOR_BACKWARD, 150);
				this_thread::sleep_for(chrono::microseconds(750));
				writeMotor(motor_fd, MOTOR_RIGHT, MOTOR_FORWARD, 150);
			} else if(rad > 35) {
				writeMotor(motor_fd, MOTOR_LEFT, MOTOR_FORWARD, 120);
				this_thread::sleep_for(chrono::microseconds(750));
				writeMotor(motor_fd, MOTOR_RIGHT, MOTOR_BACKWARD, 80);
			} else if(rad < -35) {
				writeMotor(motor_fd, MOTOR_LEFT, MOTOR_BACKWARD, 80);
				this_thread::sleep_for(chrono::microseconds(750));
				writeMotor(motor_fd, MOTOR_RIGHT, MOTOR_FORWARD, 120);
			} else if(rad > 10) {
				writeMotor(motor_fd, MOTOR_LEFT, MOTOR_FORWARD, 120);
				this_thread::sleep_for(chrono::microseconds(750));
				writeMotor(motor_fd, MOTOR_RIGHT, MOTOR_FORWARD, 100);
			} else if(rad < -10) {
				writeMotor(motor_fd, MOTOR_LEFT, MOTOR_FORWARD, 100);
				this_thread::sleep_for(chrono::microseconds(750));
				writeMotor(motor_fd, MOTOR_RIGHT, MOTOR_FORWARD, 120);
			} else {
				writeMotor(motor_fd, MOTOR_BOTH, MOTOR_FORWARD, 100);
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

		debug_lg << "Processing took: " << (getTickCount() - tloop) / getTickFrequency() * 1000.0 << " ms; FPS: " <<  cv::getTickFrequency() / (cv::getTickCount() - tloop) << lvl::info;

		//		log_sensordata(line_points, grstate, grcenter, img_rgb);
	}
}



// MAIN METHOD TO CALL THREADS

int main() {

	init_clock();

	thread drive_t (m_drive);
	image_processing();

	drive_t.detach();
	cout << "All threads closed" << endl;

	return -1;
}
