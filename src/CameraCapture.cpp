#include "CameraCapture.h"

CameraCapture::CameraCapture(int cam) : m_cap(cam){
	m_readRetVal = false;
	m_runThread = std::thread(&CameraCapture::run, this);
	m_runThread.detach();
}

CameraCapture::~CameraCapture() {
	// TODO Auto-generated destructor stub
}

bool CameraCapture::read(cv::OutputArray image){
	//Sperren des readMutex, ggf. warten wenn nicht verfügbar. Freigbae wenn Methode verlassen
	std::lock_guard<std::mutex> lock(m_readMutex);

	//Kopiere letztes ausgelesenes Bild nach "image"
	m_image.copyTo(image);
	return m_readRetVal;
}

bool CameraCapture::isOpened(){
	return m_cap.isOpened();
}

bool CameraCapture::set(int propId, double value){
	return m_cap.set(propId, value);
}

void CameraCapture::run(){

	if(!m_cap.isOpened()){
		std::cout << "CameraCapture: Konnte Kamera nicht öffnen!" << std::endl;
		return;
	}

	while(1){
		//int64_t t1 = cv::getTickCount();
		//Anfordern eines neuen Bildes. Funktion blockiert bis neus Bild aufgenommen
		if(!m_cap.grab()){
			m_readRetVal = false;
			std::this_thread::sleep_for (std::chrono::milliseconds(100));
			std::cout << "Fehler beim Holen des Bildes!" << std::endl;
			continue;
		}

		//Sperren des read mutex. ggf warten bis verfügbar
		std::unique_lock<std::mutex> lock(m_readMutex);

		//Dekodieren und speichern des letzten Bildes
		m_readRetVal = m_cap.retrieve(m_image);

		//read mutex freigeben: Erlaube holen des Bildes über "read"-Methode
		lock.unlock();

		//int64_t t2 = cv::getTickCount();
		//std::cout << "Bild aufnehmen dauerte " << (t2-t1) / cv::getTickFrequency() * 1000.0 << std::endl;
	}
}
