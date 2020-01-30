#include <iostream>		// cout
#include <fstream>    // std::ifstream std::ofstream
#include <cstdio>			// uint.._t

#include <vector>
#include <array>			// letzte sensorwerte
#include <thread>			// multithreading

#include <boost/circular_buffer.hpp>			// speichern der letzten n werte
#include <opencv2/opencv.hpp>				// opencv für bildauswertung
#include <math.h>					// abs, cos, sin, ...

#include "Logger.h"			// Logger class
#include "gruen.h"			// alles mit grünpunkt
#include "line.h"				// alles mit linie
#include "util.h"				// sonstige funktionen
#include "config.h"			// defines
#include "config_file.h"	// config datei

#include "CameraCapture.h"		// thread zum kamera einlesen
#include "VideoServer.h"			// thread für Videoausgabe über IP
#include "KamelDevices.h"					// kommunikation mit Arduino und Servosteuerung



using namespace std;
using namespace cv;

namespace lvl = spdlog::level;
Logger debug_lg("debug");			// logger class for log file 'debug.log'

configuration::data configdata;

Mat img_rgb;				// input image


void m_drive() {

	Logger behavior_lg("behavior");				// logger class for log file 'behavior.log'
	Logger sensor_lg("sensors");					// logger class for log file 'sensors.log'

	debug_lg << "init i2c devices" << lvl::debug;

	// init i2c devices

	int motor_fd = kamelI2Copen(0x08); 					// I2C Schnittstelle vom Motor-Arduino mit der Adresse 0x08 öffnen
	if(motor_fd == -1) {
		debug_lg << "error opening motor_arduino" << lvl::off;
		cout << "error opening motor_arduino" << endl;
		return;
	}

//	setMotorState(motor_fd, MOTOR_BOTH, MOTOR_OFF); 		// Beide Motoren ausschalten

	int sensor_fd = kamelI2Copen(0x09);
	if(sensor_fd == -1) {
		debug_lg << "error opening sensor_arduino" << lvl::off;
		cout << "error opening sensor_arduino" << endl;
		return;
	}

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


	bool rampe_hoch = false;
	bool rampe_runter = false;

	float m_imu_data[3];
	/*array<boost::circular_buffer<float>, 3> last_imu_data;

//	array<boost::circular_buffer<double>, 3> last_imu_time;
	for (auto& cb : last_imu_data) {
		cb.resize(100);
	}*/
//	for (auto& cb : last_imu_time) {
//		cb.resize(100);
//	}

	debug_lg << "successfully initialized sensor / camera variables" << lvl::debug;

	while(1) {

		//setMotorDirPwmBoth(sensor_fd, MOTOR_FORWARD, 160, MOTOR_OFF, 0);

		// sensor value updating ======================

		// update values

		get_gruen_data(m_grcenter, m_grstate);		// grün data updaten
		get_line_data(m_line_points);					 		// line data updaten
		get_imu_data(m_imu_data);									// imu data updaten

		//last_line_points.push_front(m_line_points);	// push_front recent values - recent value is item [0]
		//last_grcenter.push_front(m_grcenter);		// push_front recent values - recent value is item [0]

		// update arduino sensor data
		//cout << "Read data: " << getSensorData(sensor_fd, digital_sensor_data, analog_sensor_data) << endl;

		//getDigitalSensorData(sensor_fd, digital_sensor_data);

		cout << "IMU data: [" << m_imu_data[PITCH] << " | " << m_imu_data[ROLL] << " | " << m_imu_data[YAW] << "]" << endl;

		// push_front last values - recent value is item [0]
		/*last_analog_data.push_front(analog_sensor_data[0]);
		for(int i = 0; i < 8; i++) {
			last_digital_data[i].push_front(digital_sensor_data[i]);
		}*/

		// cout << "drive routine; line_points: " << m_line_points << endl;

		// main part: drive decisions	=================================


		// Rampe hoch
		if (rampe_hoch) {
			// Sichergehen, ob der Robo noch auf der Rampe ist.
			if (m_imu_data[PITCH] < 5.0) {
				rampe_hoch = false;
				continue;
			}

			// Wenn der Roboter seitlich geneigt ist die wieder gerade ausrichten
			if (m_imu_data[ROLL] < -2.0) {	// Fährt nach links - linke Seite unten
				setMotorDirPwmBoth(motor_fd, MOTOR_FORWARD, 180, MOTOR_FORWARD, 30);
			}
			else if (m_imu_data[ROLL] > 2.0) {	// Fährt nach rechts - rechte Seite unten
				setMotorDirPwmBoth(motor_fd, MOTOR_FORWARD, 30, MOTOR_FORWARD, 180);
			}

			// wenn nur 1 linepoint vorhanden ist
			else if(m_line_points.size() == 1) {
				// Wenn der Roboter nicht seitlich geneigt ist, aber die Linie nicht mehr in der Mitte ist
				if (m_line_points[0].x > 360) {										// linie auf der Rampe zu weit rechts
					setMotorDirPwmBoth(motor_fd, MOTOR_FORWARD, 180, MOTOR_FORWARD, 90);
				} else if (m_line_points[0].x < 280) {								// linie auf der Rampe zu weit links
					setMotorDirPwmBoth(motor_fd, MOTOR_FORWARD, 90, MOTOR_FORWARD, 180);
				}
				// Ansonsten gerade fahren
				else {
					setMotorDirPwm(motor_fd, MOTOR_BOTH, MOTOR_FORWARD, 180);
				}
			}
			// Ansonsten gerade fahren
			else {
				setMotorDirPwm(motor_fd, MOTOR_BOTH, MOTOR_FORWARD, 180);
			}
		}


		// Rampe runter
		else if (rampe_runter) {
			if (m_imu_data[PITCH] > -5.0 && m_imu_data[PITCH] < 5.0) {
				rampe_runter = false;
				continue;
			}

			if(m_line_points.size() == 1) {
				// Wenn die Linie nicht mehr in der Mitte ist
				if (m_line_points[0].x > 360) {										// linie auf der Rampe zu weit rechts
					setMotorDirPwmBoth(motor_fd, MOTOR_FORWARD, 60, MOTOR_BACKWARD, 10);
				} else if (m_line_points[0].x < 280) {								// linie auf der Rampe zu weit links
					setMotorDirPwmBoth(motor_fd, MOTOR_BACKWARD, 10, MOTOR_FORWARD, 60);
				} else {
					setMotorDirPwm(motor_fd, MOTOR_BOTH, MOTOR_FORWARD, 60);
				}
			} else {
				setMotorDirPwm(motor_fd, MOTOR_BOTH, MOTOR_FORWARD, 60);
			}
		}


		// Roboter gerade
		else {
			if (m_imu_data[PITCH] > 12.0) {
				rampe_hoch = true;
				continue;				// Rest der while schleife skippen
			}
			if (m_imu_data[PITCH] < -12.0) {
				rampe_runter = true;
				continue;				// Rest der while schleife skippen
			}


			// ROBOTER IST WAAGERECHT

			if (m_grstate == GRUEN_BEIDE) {

				debug_lg << "green point BOTH" << lvl::info;

				while (m_grstate != GRUEN_NICHT) {		// Solange grstate nicht GRUEN_NICHT ist
					get_gruen_data(m_grcenter, m_grstate);											// gruen werte aktualisieren
					if (m_grcenter.y > 350) {						// nah am grünpunkt; m_grcenter ist im unteren bildbereich
						setMotorDirPwm(motor_fd, MOTOR_BOTH, MOTOR_FORWARD, 180);		// beide Motoren vorwärts, pwm: 180
						thread_delay(500);																					// delay 500ms

						setMotorDirPwmBoth(motor_fd, MOTOR_BACKWARD, 200, MOTOR_FORWARD, 200);	// beide Motoren auf unterschiedliche Werte setzen
						thread_delay(3000);																	// delay 3s
						setMotorState(motor_fd, MOTOR_BOTH, MOTOR_OFF);			// Motoren ausschalten
						thread_delay(500);																	// delay 500ms

						break;
					} else if (m_grcenter.x < 310) {						// m_grcenter im linken Bildbereich
						setMotorDirPwmBoth(motor_fd, MOTOR_FORWARD, 0, MOTOR_FORWARD, 90);			// links Kurve
					} else if (m_grcenter.x > 330) {						// m_grcenter im rechten Bildbereich
						setMotorDirPwmBoth(motor_fd, MOTOR_FORWARD, 90, MOTOR_FORWARD, 0);			// rechts Kurve
					} else {																	// m_grcenter im mittleren, oberen Bildbereich
						setMotorDirPwm(motor_fd, MOTOR_BOTH, MOTOR_FORWARD, 60);								// langsam vorwärts
					}
				}

			} else if (m_grstate == GRUEN_LINKS && m_grcenter.y > 480 - 150) {			// m_grcenter im unteren Bildbereich + m_grstate = GRUEN_LINKS

				debug_lg << "green point LEFT" << lvl::info;					// in debug.log loggen

				setMotorDirPwm(motor_fd, MOTOR_BOTH, MOTOR_FORWARD, 150);				// beide Motoren vorwärts, pwm: 150
				thread_delay(300);						// delay 300ms
				setMotorDirPwmBoth(motor_fd, MOTOR_BACKWARD, 190, MOTOR_FORWARD, 150);		// linkskurve, links schneller
				thread_delay(500);						// delay 500ms

			} else if (m_grstate == GRUEN_RECHTS && m_grcenter.y > 480 - 150) {				// m_grcenter im unteren Bildbereich + m_grstate = GRUEN_RECHTS

				debug_lg << "green point RIGHT" << lvl::info;			// in debug.log loggen

				setMotorDirPwm(motor_fd, MOTOR_BOTH, MOTOR_FORWARD, 150);			// beide Motoren vorwärts, pwm: 150
				thread_delay(300);					// delay 300ms
				setMotorDirPwmBoth(motor_fd, MOTOR_FORWARD, 150, MOTOR_BACKWARD, 190);		// rechtskurve, rechts schneller
				thread_delay(500);					// delay 500ms

			} else if(m_line_points.size() == 1) {							// wenn nur 1 linepoint vorhanden ist

				debug_lg << "single Line point: " << m_line_points[0] << lvl::info;
				//			cout << "Different value -> check motor output for line" << endl;

				if (m_line_points[0].x > 575) {										// line_points[0] rechts außen
					setMotorDirPwmBoth(motor_fd, MOTOR_FORWARD, 190, MOTOR_BACKWARD, 160);
				} else if (m_line_points[0].x < 65) {								// line_points[0] links außen
					setMotorDirPwmBoth(motor_fd, MOTOR_BACKWARD, 160, MOTOR_FORWARD, 190);
				} else if (m_line_points[0].x > 500) {							// line_points[0] rechts
					setMotorDirPwmBoth(motor_fd, MOTOR_FORWARD, 160, MOTOR_BACKWARD, 80);
				} else if (m_line_points[0].x < 140) {							// line_points[0] links
					setMotorDirPwmBoth(motor_fd, MOTOR_BACKWARD, 80, MOTOR_FORWARD, 160);
				} else if (m_line_points[0].x > 400) {							// line_points[0] halb rechts
					setMotorDirPwmBoth(motor_fd, MOTOR_FORWARD, 160, MOTOR_BACKWARD, 10);
				} else if (m_line_points[0].x < 240) {							// line_points[0] halb links
					setMotorDirPwmBoth(motor_fd, MOTOR_BACKWARD, 10, MOTOR_FORWARD, 160);
				} else if (m_line_points[0].x > 340) {							// line_points[0] mitte rechts
					setMotorDirPwmBoth(motor_fd, MOTOR_FORWARD, 160, MOTOR_OFF, 0);
				} else if (m_line_points[0].x < 300) {						 // line_points[0] mitte links
					setMotorDirPwmBoth(motor_fd, MOTOR_OFF, 0, MOTOR_FORWARD, 160);
				} else {																					 // line_points[0] mitte
					setMotorState(motor_fd, MOTOR_BOTH, MOTOR_FORWARD_NORMAL);
				}

			} else if(m_line_points.size() == 0 && last_line_points[1].size() == 1) {						// linie verloren, vorher nur 1 linepoint
				debug_lg << "lost line, last line singe line point: " << last_line_points[1][0];

				if (last_line_points[1][0].x > 575) {				// letzter linepoint rechts außen: nach rechts fahren, bis linie wieder gefunden
					debug_lg << " -  correction right" << lvl::warn;

					setMotorDirPwmBoth(motor_fd, MOTOR_FORWARD, 160, MOTOR_BACKWARD, 120);

					while(m_line_points.size() == 0 || (m_line_points.size() == 1 && m_line_points[0].x > img_rgb.cols / 2 + 100)) {
						thread_delay(5);
						get_line_data(m_line_points);
					}

					setMotorDirPwm(motor_fd, MOTOR_BOTH, MOTOR_BACKWARD, 100);
					thread_delay(500);
					setMotorState(motor_fd, MOTOR_BOTH, MOTOR_OFF);

				} else if (last_line_points[1][0].x < 65) {				// letzter linepoint links außen: nach links fahren, bis linie wieder gefunden
					cout << " -   correction left" << lvl::warn;

					setMotorDirPwmBoth(motor_fd, MOTOR_BACKWARD, 120, MOTOR_FORWARD, 160);

					while(m_line_points.size() == 0 || (m_line_points.size() == 1 && m_line_points[0].x < img_rgb.cols / 2 - 100)) {
						thread_delay(5);
						get_line_data(m_line_points);
					}

					setMotorDirPwm(motor_fd, MOTOR_BOTH, MOTOR_BACKWARD, 100);
					thread_delay(500);
					setMotorState(motor_fd, MOTOR_BOTH, MOTOR_OFF);

				} else {							// wenn der letzte linepoint mittig war (Lücke) weiter gerade fahren
					setMotorState(motor_fd, MOTOR_BOTH, MOTOR_FORWARD_NORMAL);			// vorwärts mit festgelegtem Standardtempo
					debug_lg << lvl::warn;
				}
			} else {			// wenn linepoints.size() > 1 und kein grünpunkt, grade fahren
				setMotorState(motor_fd, MOTOR_BOTH, MOTOR_FORWARD_NORMAL);				// vorwärts mit festgelegtem Standardtempo
				//debug_lg << "driving forward, " << m_line_points.size() << " line points" << lvl::info;
			}
		}

		thread_delay(7);			// delay, da sonst der i2c buffer überschrieben wird und der Motorarduino falsche werte bekommt
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
	srv.namedWindow("H-S Histogram");



	int64 tloop;

	while(1) {

		tloop = getTickCount();			// Tickcount for whole loop

		while(!cam.read(img_rgb)){}		// warten bis das aktuelle bild einlesbar ist, dann in img_rbg einlesen

		// Filter image and convert to hsv
		GaussianBlur(img_rgb, img_rgb, Size(5,5),2,2);		// Gaussian blur to normalize image
		cvtColor(img_rgb, hsv, COLOR_BGR2HSV);			// Convert to HSV and save in Mat hsv

//#ifdef PLOT_HSV_HISTOGRAM

		// Histogramm der HSV Matrix ausgeben (wenn PLOT_HSV_HISTOGRAM definiert ist)

		vector<Mat> bgr_planes;
		split( hsv, bgr_planes );
		int histSize = 256;
		float range[] = { 0, 256 }; //the upper boundary is exclusive
		const float* histRange = { range };
		bool uniform = true, accumulate = false;
		Mat b_hist, g_hist, r_hist;
		calcHist( &bgr_planes[0], 1, 0, Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate );
		calcHist( &bgr_planes[1], 1, 0, Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate );
		calcHist( &bgr_planes[2], 1, 0, Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate );
		int hist_w = 512, hist_h = 400;
		int bin_w = cvRound( (double) hist_w/histSize );
		Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 255,255,255) );
		normalize(b_hist, b_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
		normalize(g_hist, g_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
		normalize(r_hist, r_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
		for( int i = 1; i < histSize; i++ )
		{
				line( histImage, Point( bin_w*(i-1), hist_h - cvRound(b_hist.at<float>(i-1)) ),
							Point( bin_w*(i), hist_h - cvRound(b_hist.at<float>(i)) ),
							Scalar( 255, 0, 0), 2, 8, 0  );
				line( histImage, Point( bin_w*(i-1), hist_h - cvRound(g_hist.at<float>(i-1)) ),
							Point( bin_w*(i), hist_h - cvRound(g_hist.at<float>(i)) ),
							Scalar( 0, 255, 0), 2, 8, 0  );
				line( histImage, Point( bin_w*(i-1), hist_h - cvRound(r_hist.at<float>(i-1)) ),
							Point( bin_w*(i), hist_h - cvRound(r_hist.at<float>(i)) ),
							Scalar( 0, 0, 255), 2, 8, 0  );
		}


//#endif


		//		log_timing(tlast, "Reading, color covert, gauss: ");

		separate_gruen(hsv, bin_gr);  // grün auf dem Bild erkennen und in bin_gr als weiß schreiben, muss vor line_calc stehen, wenn der grünpunkt nicht als Linie erkannt werden soll

		//		log_timing(tlast, "Green separation: ");

		line_calc(img_rgb, hsv, bin_sw, bin_gr);		// linienpunkte berechnen, in m_line_points schreiben
		//		log_timing(tlast, "Line calculation: ");

		gruen_calc(img_rgb, hsv, bin_sw, bin_gr);		// grstate und grcenter berechnen


		//		log_timing(tlast, "Green calc: ");

		// Alle Bilder an VideoServer übergeben

		srv.imshow("H-S Histogram", histImage );
		srv.imshow("Input", img_rgb);
		srv.imshow("HSV", hsv);
		srv.imshow("Mask SW", bin_sw);
		srv.imshow("Mask Green", bin_gr);
		srv.update();			// VideoServer updaten

		camera_lg << "Processing took: " << (getTickCount() - tloop) / getTickFrequency() * 1000.0 << " ms; FPS: " <<  cv::getTickFrequency() / (cv::getTickCount() - tloop) << lvl::debug;

	}
}



// MAIN METHOD TO CALL THREADS

extern uint8_t THRESH_BLACK;
extern cv::Scalar LOWER_GREEN;
extern cv::Scalar UPPER_GREEN;


int main() {

	init_clock();			// set start_clock to current ms

	std::ifstream ifs("../src/config.info", std::ifstream::in);
  	ifs >> configdata;
  	ifs.close();

	set_thresh_black(configdata.getintvalue("THRESH_BLACK"));
	set_gruen_range(configdata.getscalarvalue("LOWER_GREEN"),configdata.getscalarvalue("UPPER_GREEN"));

	thread drive_t (m_drive);			// thread starten; ruft void m_drive auf
	thread imu_t (m_imu);					// thread startet; void m_imu in neuem thread
	image_processing();						// start void image_processing

	drive_t.detach();							// drive_t anhalten
	imu_t.detach();							// imu_t anhalten
	cout << "All threads closed" << endl;

	return -1;						// fehler
}
