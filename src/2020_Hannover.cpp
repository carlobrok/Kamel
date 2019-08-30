#include <iostream>		// cout
#include <cstdio>			// uint.._t

#include <vector>
#include <array>			// letzte sensorwerte
#include <thread>			// multithreading

#include <boost/circular_buffer.hpp>			// speichern der letzten n werte
#include "opencv2/opencv.hpp"				// opencv für bildauswertung
#include "math.h"

#include "Logger.h"			// Logger class
#include "config.h"			// defines
#include "gruen.h"			// alles mit grünpunkt
#include "line.h"				// alles mit linie
#include "util.h"				// sonstige funktionen

#include "CameraCapture.h"		// thread zum kamera einlesen
#include "VideoServer.h"			// thread für Videoausgabe über IP
#include "KamelI2C.h"					// kommunikation mit Arduino und Servosteuerung

using namespace std;
using namespace cv;

namespace lvl = spdlog::level;
Logger debug_lg("debug");			// logger class for log file 'debug.log'

Mat img_rgb;				// input image


void m_drive() {

	Logger behavior_lg("behavior");				// logger class for log file 'behavior.log'
	Logger sensor_lg("sensors");					// logger class for log file 'sensors.log'

	debug_lg << "init i2c devices" << lvl::debug;

	// init i2c devices

	int motor_fd = kamelI2Copen(0x08); 					// I2C Schnittstelle vom Motor-Arduino mit der Adresse 0x08 öffnen
	/*if(motor_fd == -1) {
		return;
	}*/
	setMotorState(motor_fd, MOTOR_BOTH, MOTOR_OFF); 		// Beide Motoren ausschalten

	int sensor_fd = kamelI2Copen(0x09);
/*	if(sensor_fd == -1) {
		return;
	}*/

	debug_lg << "successfully initialized i2c devices" << lvl::debug;
	debug_lg << "init sensor / camera variables" << lvl::debug;


	// init line variables
	vector<Point> m_line_points;			// m_line_points enthält lokale line_points, gilt nur im drive thread
	get_line_data(m_line_points);
	boost::circular_buffer<vector<Point>> last_line_points(50);		// circular_buffer last_line_points initialisieren


	// init green variables
	Point m_grcenter;				// temporäre kopie für den drive thread
	int m_grstate;				// temporäre kopie für den drive thread
	get_gruen_data(m_grcenter, m_grstate);
	boost::circular_buffer<Point> last_grcenter(50);		// circular_buffer initialisieren

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

		// update values
		get_gruen_data(m_grcenter, m_grstate);
		get_line_data(m_line_points);

		// push_front last values - recent value is item [0]
		last_line_points.push_front(m_line_points);
		last_grcenter.push_front(m_grcenter);

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
					get_line_data(m_line_points);
				}

				setMotorDirPwm(motor_fd, MOTOR_BOTH, MOTOR_BACKWARD, 100);
				thread_delay(500);
				setMotorState(motor_fd, MOTOR_BOTH, MOTOR_OFF);

			} else if (last_line_points[1][0].x < 65) {
				cout << " -   correction left" << lvl::warn;

				setMotorDirPwmBoth(motor_fd, MOTOR_BACKWARD, 120, MOTOR_FORWARD, 160);

				while(m_line_points.size() == 0 || (m_line_points.size() == 1 && m_line_points[0].x < img_rgb.cols / 2 - 100)) {
					thread_delay(5);
					get_line_data(m_line_points);
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

	Logger camera_lg("camera");		// Logger für Dateiname des aktuellen Bilds, sowie andere Bildinformationen

	Mat hsv;			// input Bild im hsv Format
	Mat bin_sw;		// binäres bild schwarz / weiß erkennung; schwarze linie auf bild weiß, alles andere schwarz
	Mat bin_gr;		// binäres bild grünerkennung; grüner punkt weiß, alles andere schwarz


	CameraCapture cam(0);		// Video eingabe der Kamera '0'; 0, wenn nur eine Kamera angeschlossen ist
	VideoServer srv;				// Klasse für den VideoServer

	cam.set(cv::CAP_PROP_FPS, 30);			// Kamera Framerate auf 30 fps
	cam.set(cv::CAP_PROP_FRAME_WIDTH, 640);			// Bildauflösung auf 640 x 480px
	cam.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

	//Füge Fenster zum VideoServer hinzu
	srv.namedWindow("Mask Green");
	srv.namedWindow("Mask SW");
	srv.namedWindow("HSV");
	srv.namedWindow("Input");

	vector<Point> m_line_points;		// lokaler vector mit allen Punkten der Linie
	Point m_grcenter(0,0);				// lokaler Point des Zentrums vom Grünen Punkt
	int m_grstate = GRUEN_NICHT;			// Zustand des grünen Punktes

	while(1) {

		int64 tloop = getTickCount();			// Tickcount for whole loop

		while(!cam.read(img_rgb)){}		// warten bis das aktuelle bild einlesbar ist, dann in img_rbg einlesen

		// Filter image and convert to hsv
		GaussianBlur(img_rgb, img_rgb, Size(5,5),2,2);		// Gaussian blur to normalize image
		cvtColor(img_rgb, hsv, COLOR_BGR2HSV);			// Convert to HSV and save in Mat hsv

		//		log_timing(tlast, "Reading, color covert, gauss: ");

		// grün auf dem Bild erkennen und in bin_gr schreiben,
		// muss vor line_calc stehen, wenn der grünpunkt nicht als Linie erkannt werden soll
		separate_gruen(hsv, bin_gr);

		//		log_timing(tlast, "Green separation: ");

		line_calc(img_rgb, hsv, bin_sw, bin_gr, m_line_points);		// linienpunkte berechnen, in m_line_points schreiben

		set_line_data(m_line_points);

		//		log_timing(tlast, "Line calculation: ");

		gruen_calc(img_rgb, hsv, bin_sw, bin_gr, m_grstate, m_grcenter);

		set_gruen_data(m_grcenter, m_grstate);

		//		log_timing(tlast, "Green calc: ");

		srv.imshow("Input", img_rgb);
		srv.imshow("HSV", hsv);
		srv.imshow("Mask SW", bin_sw);
		srv.imshow("Mask Green", bin_gr);
		srv.update();


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
