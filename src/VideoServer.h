/*
 * VideoServer.h
 *
 *  Created on: Sep 2, 2017
 *      Author: joern
 */

#ifndef VIDEOSERVER_H
#define VIDEOSERVER_H

#include <iostream>
#include <string>
#include <unordered_map>
#include <queue>
#include <thread>
#include <mutex>
#include "boost/asio.hpp"
#include "opencv2/opencv.hpp"

using boost::asio::ip::tcp;

class VideoServer {
public:
	VideoServer(bool l_asyncSend = false);
	virtual ~VideoServer();
	void namedWindow(const std::string & window);
	void imshow(const std::string & window, const cv::Mat & image);
	void update();

private:
	void run();
	void waitForConnection();

	struct window_t{
		cv::Mat image;
		std::string windowName;
		bool readyForData;
	};

	bool m_asyncSend;
	bool m_readyForData;
	std::vector<int> m_jpegParams;

	typedef std::unordered_map<std::string, window_t> window_map;

	boost::asio::io_service m_ioService;
	tcp::acceptor m_acceptor;
	tcp::socket m_socket;

	window_map m_windowMap;
	std::queue<window_t> m_work;

	std::thread m_runThread;
	std::mutex m_workQueueMutex;

};

#endif /* VIDEOSERVER_H_ */
