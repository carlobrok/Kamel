#include "VideoServer.h"

VideoServer::VideoServer(bool l_asyncSend) : m_acceptor(m_ioService, tcp::endpoint(tcp::v4(), 1300)), m_socket(m_ioService) {

	m_asyncSend = l_asyncSend;
	m_readyForData = true;

	m_jpegParams.push_back(cv::ImwriteFlags::IMWRITE_JPEG_QUALITY);
	m_jpegParams.push_back(80);

	m_runThread = std::thread(&VideoServer::run, this);
	m_runThread.detach();
}

VideoServer::~VideoServer() {
	// TODO Auto-generated destructor stub
}

void VideoServer::waitForConnection(){
	std::cout << "Waiting for new connection..." << std::endl;
	try{
		m_acceptor.accept(m_socket);
		std::cout << "Connection established!" << std::endl;
	}
	catch (std::exception& e)
	{
		std::cerr << e.what() << std::endl;
	}

}

void VideoServer::namedWindow(const std::string & f_windowName){
	window_map::const_iterator l_element = m_windowMap.find(f_windowName);

	if(l_element == m_windowMap.end()){
		window_t l_tmpWindow;
		l_tmpWindow.readyForData = true;
		l_tmpWindow.windowName = f_windowName;
		l_tmpWindow.windowName.reserve(20);
		m_windowMap.insert(std::make_pair(f_windowName, l_tmpWindow));
	}
	else{
		std::cout << "Window already exists!" << std::endl;
	}
}

void VideoServer::imshow(const std::string & f_windowName, const cv::Mat & f_image){
	window_map::iterator l_element = m_windowMap.find(f_windowName);

	if(l_element != m_windowMap.end()){
		if(l_element->second.readyForData && (m_asyncSend || (!m_asyncSend && m_readyForData))){
			f_image.copyTo(l_element->second.image);
			std::lock_guard<std::mutex> lock(m_workQueueMutex);
			m_work.push(l_element->second);
			l_element->second.readyForData = false;
		}
		else{
			//std::cout << "Previous image(s) not processed. Skipping..." << std::endl;
		}
	}
	else{
		std::cout << "ERROR: Window not found!" << std::endl;
	}
}

void VideoServer::update(){
	if(m_readyForData && !m_work.empty()){
		m_readyForData = false;
	}
}

void VideoServer::run(){
	waitForConnection();
	while(1){

		if(!m_work.empty() && (m_asyncSend || (!m_asyncSend && !m_readyForData))){
			std::unique_lock<std::mutex> lock(m_workQueueMutex);
			window_t l_tmpWindow = m_work.front();
			m_work.pop();
			lock.unlock();

			std::vector<uchar> img_buf;
			cv::imencode(".jpg", l_tmpWindow.image, img_buf, m_jpegParams);
			m_windowMap[l_tmpWindow.windowName].readyForData = true;

			uint32_t sz = img_buf.size();
			try{
				boost::asio::write(m_socket, boost::asio::buffer(&sz, sizeof(sz)));
				boost::asio::write(m_socket, boost::asio::buffer(l_tmpWindow.windowName.c_str(), 20));
				boost::asio::write(m_socket, boost::asio::buffer(img_buf));
			}
			catch (std::exception& e){
				std::cerr << e.what() << std::endl;
				m_socket.close();
				waitForConnection();
			}

			//std::cout << "Send took " << (t2-t1) / cv::getTickFrequency() * 1000.0 << std::endl;

			if(m_work.empty()){
				m_readyForData = true;
			}
		}
		else{
			std::this_thread::sleep_for (std::chrono::milliseconds(1));
		}
	}
}
