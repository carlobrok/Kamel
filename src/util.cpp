#include "util.h"
#include <fstream>

bool log_to_file = false;
std::ofstream sensorlog;

void init_sensorlog(int & argc, char* & argv[]) {
	if(argc > 1) {
		sensorlog.open(argv[1]);
		if(sensorlog.is_open()) {
			std::cerr << "sensordata will be logged to: " << argv[1] << std::endl;
			log_to_file = true;
		} else {
			std::cerr << "Error opening " << argv[1] << ". Sensordata will be written to cout" << std::endl;
			log_to_file = false;
		}
	} else {
		std::cerr << "No filename given. Output sensordata to cout" << std::endl;
		log_to_file = false;
	}
}


void log_sensordata(std::vector<cv::Point> & line_points, int & grstate, int & gr_center) {
	if(log_to_file)  {

	} else {
		std::cout << "sensordata here" << std::endl;
	}
}

bool inMat(cv::Point p, int cols, int rows) {
	if (p.x >= 0 && p.x < cols && p.y >= 0 && p.y < rows)
	{
		return true;
	} else {
		return false;
	}
}

