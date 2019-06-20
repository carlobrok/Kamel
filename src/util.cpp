#include "util.h"
#include "config.h"
#include <fstream>
#include <string>

bool log_to_file = false;
std::ofstream sensorlog;
int log_iteration = 0;
char* log_filename;
std::string log_imgname;

/*
void init_sensorlog(int & argc, char* & argv[]) {
	if(argc > 1) {
		sensorlog.open(argv[1]);
		if(sensorlog.is_open()) {
			std::cerr << "sensordata will be logged to: " << argv[1] << std::endl;
			log_to_file = true;
			log_filename = argv[1];
			log_imgname = std::string(argv[1]);				// img_name = logfile_name.log
			log_imgname.replace(log_imgname.end()-4, log_imgname.end(), "");	// delete ".log"
			log_imgname += "_%n.jpg";				// add %n to insert log_iteration
		} else {
			std::cerr << "Error opening " << argv[1] << ". Sensordata will be written to cout" << std::endl;
			log_to_file = false;
		}
	} else {
		std::cerr << "No filename given. Output sensordata to cout" << std::endl;
		log_to_file = false;
	}
}*/


void log_sensordata(std::vector<cv::Point> & line_points, int & grstate, cv::Point & grcenter, cv::Mat & img_output) {
#ifdef LOG_FILE
	if(log_to_file)  {
		sensorlog << log_iteration << " " << grstate << " " << grcenter.x << "/" << grcenter.y << " ";
		for (unsigned int i  = 0; i  < line_points.size(); i++) {
			sensorlog << line_points[i].x << "/" << line_points[i].y;
			if(i < line_points.size() - 1) {
				sensorlog << ":";
			}
		}

		std::string img_name = log_imgname;
		std::size_t found = img_name.find("%n");
		img_name.replace(found, found+1, std::string(log_iteration));

		sensorlog << " " << img_name << std::endl;
		cv::imwrite(img_name, img_output);
	} else {
#endif
		std::cout << log_iteration << " " << grstate << " " << grcenter.x << "/" << grcenter.y << " ";
		for (unsigned int i  = 0; i  < line_points.size(); i++) {
			std::cout << line_points[i].x << "/" << line_points[i].y;
			if(i < line_points.size() - 1) {
				std::cout << ":";
			}
		}

		std::cout << std::endl;
#ifdef LOG_FILE
	}
#endif
}

bool inMat(cv::Point p, int cols, int rows) {
	if (p.x >= 0 && p.x < cols && p.y >= 0 && p.y < rows)
	{
		return true;
	} else {
		return false;
	}
}

