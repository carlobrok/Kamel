/*
 * CameraCapture.h
 *
 *  Created on: Feb 24, 2018
 *      Author: joern
 */

#ifndef CAMERACAPTURE_H_
#define CAMERACAPTURE_H_

#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>

class CameraCapture {
public:
	CameraCapture(int cam);
	virtual ~CameraCapture();

	bool read(cv::OutputArray image);
	bool isOpened();
	bool set(int propId, double value);

private:
	void run();

	cv::VideoCapture m_cap;

	bool m_readRetVal;

	cv::Mat m_image;

	std::thread m_runThread;
	std::mutex m_readMutex;

};


#endif /* CAMERACAPTURE_H_ */
