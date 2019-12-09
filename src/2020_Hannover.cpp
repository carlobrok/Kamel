#include <iostream>		// cout
#include <cstdio>			// uint.._t

#include <vector>
#include <array>			// letzte sensorwerte
#include <thread>			// multithreading

#include <boost/circular_buffer.hpp>			// speichern der letzten n werte
#include "opencv2/opencv.hpp"				// opencv für bildauswertung
#include "math.h"					// abs, cos, sin, ...

#include "Logger.h"			// Logger class
#include "config.h"			// defines
#include "gruen.h"			// alles mit grünpunkt
#include "line.h"				// alles mit linie
#include "util.h"				// sonstige funktionen


#define VIDEO_NAME video_name[vid_name_idx]
#define VIDEOS_AMOUNT 4

using namespace std;
using namespace cv;


String video_name[VIDEOS_AMOUNT] = {
	//"20191207_line1.mp4",
	//"20191207_line2.mp4",
	"201905_line1.mp4",
	"201905_line2.mp4",
	"201905_line3.mp4",
	"201905_line4.mp4"
};

int vid_name_idx = 0;


int open_new_vid(VideoCapture & cap) {
	if(cap.isOpened()) {
		cap.release();
	}
	cap.open(VIDEO_NAME);

	if(!cap.isOpened()){
		cout << "Video not opened!" << endl;
		return -2;
	}

	if(vid_name_idx < VIDEOS_AMOUNT-1) {
		vid_name_idx++;
	} else {
		vid_name_idx = 0;
	}
	return 1;
}


namespace lvl = spdlog::level;
Logger debug_lg("debug");			// logger class for log file 'debug.log'

Mat img_rgb;				// input image





// MAIN METHOD TO CALL THREADS

int main() {

	init_clock();			// set start_clock to current ms
	//init_line_ellipse();					//  Ellipsen initialisieren

	// IMAGE PROCESSING ====================


	Logger camera_lg("camera");		// Logger für Dateiname des aktuellen Bilds, sowie andere Bildinformationen

	Mat hsv;			// input Bild im hsv Format
	Mat bin_sw(IMG_HEIGHT, IMG_WIDTH, CV_8U, cv::Scalar(0));		// binäres bild schwarz / weiß erkennung; schwarze linie auf bild weiß, alles andere schwarz
	Mat bin_gr(IMG_HEIGHT, IMG_WIDTH, CV_8U);		// binäres bild grünerkennung; grüner punkt weiß, alles andere schwarz

	VideoCapture cap;		// Video eingabe der Kamera '0'; 0, wenn nur eine Kamera angeschlossen ist

	if(open_new_vid(cap) == -2) {		// wenn kein Video vorhanden abbruch
		return -2;
	}

	//Füge Fenster zum VideoServer hinzu
	namedWindow("Mask Green");
	namedWindow("Mask SW");
	namedWindow("HSV");
	namedWindow("Input");

	vector<Point> m_line_points;		// lokaler vector mit allen Punkten der Linie
	Point m_grcenter(0,0);				// lokaler Point des Zentrums vom Grünen Punkt
	int m_grstate = GRUEN_NICHT;			// Zustand des grünen Punktes

	init_line_ellipse(); //  Ellipse initialisieren

	int64 tloop;

	while (1) {

		cap.read(img_rgb);
		if(img_rgb.empty()) {			// wenn Video zuende, neues öffnen
			cout << "No image!" << endl;

			open_new_vid(cap);

		} else {

			tloop = getTickCount();			// Tickcount for whole loop

			// Filter image and convert to hsv
			GaussianBlur(img_rgb, img_rgb, Size(5,5),2,2);
			cvtColor(img_rgb, hsv, COLOR_BGR2HSV);			// Convert to HSV and save in Mat hsv

			separate_gruen(hsv, bin_gr);  // grün auf dem Bild erkennen und in bin_gr als weiß schreiben, muss vor line_calc stehen, wenn der grünpunkt nicht als Linie erkannt werden soll
			line_calc(img_rgb, hsv, bin_sw, bin_gr);		// linienpunkte berechnen, in m_line_points schreiben

			gruen_calc(img_rgb, hsv, bin_sw, bin_gr);		// grstate und grcenter berechnen
			get_gruen_data(m_grcenter, m_grstate);
			//cout << "GRSTATE: " << m_grstate << endl;

			// Alle Bilder anzeigen
			imshow("Input", img_rgb);
			imshow("HSV", hsv);
			imshow("Mask SW", bin_sw);
			imshow("Mask Green", bin_gr);


			camera_lg << "Processing took: " << (getTickCount() - tloop) / getTickFrequency() * 1000.0 << " ms; FPS: " <<  cv::getTickFrequency() / (cv::getTickCount() - tloop) << lvl::info;

			int key = waitKey(0);
			if(key == 'q') {
				//cout << "Exit by key q" << endl;
				return -1;
			} else if(key == 'n') {
				open_new_vid(cap);
			}
		}
	}

	return -1;						// fehler
}
