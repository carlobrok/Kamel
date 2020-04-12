#include <iostream>		// cout
#include <fstream>    // std::ifstream std::ofstream
#include <stdint.h>       	// int8_t, uint8_t, uint16_t, ...

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
#include "drive.h"

#include "CameraCapture.h"		// thread zum kamera einlesen
#include "VideoServer.h"			// thread für Videoausgabe über IP
#include "KamelDevices.h"					// kommunikation mit Arduino und Servosteuerung

namespace lvl = spdlog::level;

using namespace std;
using namespace cv;


Logger debug_lg("debug");			// logger class for log file 'debug.log'

configuration::data configdata;

void image_processing() {

	Mat img_rgb;	// input image
	Mat hsv;			// input Bild im hsv Format
	Mat bin_sw;		// binäres bild schwarz / weiß erkennung; schwarze linie auf bild weiß, alles andere schwarz
	Mat bin_gr;		// binäres bild grünerkennung; grüner punkt weiß, alles andere schwarz

	debug_lg << " > init camera" << lvl::debug;
	CameraCapture cam(0);		// Video eingabe der Kamera '0'; 0, wenn nur eine Kamera angeschlossen ist
	VideoServer srv;				// Klasse für den VideoServer


	cam.set(cv::CAP_PROP_FPS, 30);			// Kamera Framerate auf 30 fps

	// Does not work with raspbian buster on the Raspberry 4B
	//cam.set(cv::CAP_PROP_FRAME_WIDTH, 640);			// Bildauflösung auf 640 x 480px
	//cam.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

	//Füge Fenster zum VideoServer hinzu
	srv.namedWindow("Mask Green");
	srv.namedWindow("Mask SW");
	srv.namedWindow("HSV");
	srv.namedWindow("Input");
#ifdef PLOT_HSV_HISTOGRAM
	srv.namedWindow("H-S Histogram");
#endif



	int64 tloop;

	while(1) {

		tloop = getTickCount();			// Tickcount for whole loop

		while(!cam.read(img_rgb)){}		// warten bis das aktuelle bild einlesbar ist, dann in img_rbg einlesen

		// Filter image and convert to hsv
		GaussianBlur(img_rgb, img_rgb, Size(5,5),2,2);		// Gaussian blur to normalize image
		cvtColor(img_rgb, hsv, COLOR_BGR2HSV);			// Convert to HSV and save in Mat hsv

#ifdef PLOT_HSV_HISTOGRAM

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
		srv.imshow("H-S Histogram", histImage );
#endif


		//		log_timing(tlast, "Reading, color covert, gauss: ");

		separate_gruen(hsv, bin_gr);  // grün auf dem Bild erkennen und in bin_gr als weiß schreiben, muss vor line_calc stehen, wenn der grünpunkt nicht als Linie erkannt werden soll

		//		log_timing(tlast, "Green separation: ");

		line_calc(img_rgb, hsv, bin_sw, bin_gr);		// linienpunkte berechnen, in m_line_points schreiben
		//		log_timing(tlast, "Line calculation: ");

		gruen_calc(img_rgb, hsv, bin_sw, bin_gr);		// grstate und grcenter berechnen


		//		log_timing(tlast, "Green calc: ");

		// Alle Bilder an VideoServer übergeben


		srv.imshow("Input", img_rgb);
		srv.imshow("HSV", hsv);
		srv.imshow("Mask SW", bin_sw);
		srv.imshow("Mask Green", bin_gr);
		srv.update();			// VideoServer updaten

		std::cout << "Processing took: " << (getTickCount() - tloop) / getTickFrequency() * 1000.0 << " ms; FPS: " <<  cv::getTickFrequency() / (cv::getTickCount() - tloop) << std::endl;

	}
}



// MAIN FUNCTION TO CALL THREADS

int main() {

	debug_lg.set_level(lvl::debug);

	debug_lg << "read config file" << lvl::debug;
	std::ifstream ifs("/home/pi/projects/KamelPi/src/config.info", std::ifstream::in);			// Config datei einlesen
	ifs >> configdata;			// Config laden
	ifs.close();						// Datei wieder schließen
	debug_lg << "> COMPLETE" << lvl::debug;

	debug_lg << "set config variables" << lvl::debug;
	debug_lg << "THRESH_BLACK" << lvl::debug;
	set_thresh_black(configdata.getintvalue("THRESH_BLACK"));				// Schwarz threshold setzen
	debug_lg << "LOWER_GREEN, UPPER_GREEN" << lvl::debug;
	set_gruen_range(configdata.getscalarvalue("LOWER_GREEN"),configdata.getscalarvalue("UPPER_GREEN"));		// Gruen Threshold setzen

	debug_lg << "FLASCHE_FAHREN" << lvl::debug;
	set_flasche_fahren(configdata.getintvalue("FLASCHE_FAHREN"));
	debug_lg << "FLASCHE_LINKS" << lvl::debug;
	set_flasche_links(configdata.getboolvalue("FLASCHE_LINKS"));
	debug_lg << "KAMERA_WINKEL" << lvl::debug;
	set_kamera_winkel(configdata.getintvalue("KAMERA_WINKEL"));
	debug_lg << "> COMPLETE" << lvl::debug;

	debug_lg << "init sensoren arduino" << lvl::debug;
	if(init_sensoren(0x09) == -1) {
		debug_lg << "> FAILED - error opening sensor_arduino" << lvl::off;
		return -1;
	}
	debug_lg << "> COMPLETE" << lvl::debug;

	debug_lg << "starting threads" << lvl::debug;
	thread drive_t (m_drive);			// thread starten; ruft void m_drive auf
	thread imu_t (m_imu);					// thread startet; void m_imu in neuem thread
	image_processing();						// start void image_processing

	debug_lg << "stopping all threads" << lvl::warn;
	drive_t.detach();							// drive_t anhalten
	imu_t.detach();							// imu_t anhalten
	debug_lg << "> COMPLETE - all threads closed" << lvl::warn;

	return -1;						// fehler
}
