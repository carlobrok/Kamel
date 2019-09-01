#ifndef LOGGER_H
#define LOGGER_H

#include <string>

#include "spdlog/async.h" //support for async logging.
#include "spdlog/sinks/basic_file_sink.h"

#include "opencv2/opencv.hpp"
#include "util.h"

class Logger {
private:

	std::shared_ptr<spdlog::logger> _logger;
	std::string _buffer = "";

public:
	Logger(std::string log_name);

	void log(std::string msg, spdlog::level::level_enum lvl);
	void log_buffer(spdlog::level::level_enum lvl);

	/*
		'<<' nutzen um in den buffer zu schreiben.
		Als letztes Element spdlog::level::level_enum hinzufügen, um den buffer in die Datei zu schreiben.
		Alternativ kann log_buffer genutzt werden.

		Bsp.:
		Logger example_log("example");
		example_log << "This is only for testing purposes!" << spdlog::level::info;
	*/

	template< typename T >
	Logger& operator<<(const T& val) {
		_buffer += val;
		return *this;
	}

	Logger& operator<<(const int& i) {
		_buffer += std::to_string(i);
		return *this;
	}

	Logger& operator<<(const long& l) {
		_buffer += std::to_string(l);
		return *this;
	}

	Logger& operator<<(const double& d) {
		_buffer += std::to_string(d);
		return *this;
	}

	Logger& operator<<(const float& f) {
		_buffer += std::to_string(f);
		return *this;
	}

	Logger& operator<<(const size_t& s) {
		_buffer += std::to_string((int)s);
		return *this;
	}

	Logger& operator<<(const cv::Point& p) {
		_buffer += "("  + std::to_string(p.x) + "," + std::to_string(p.y) + ")";
		return *this;
	}

	Logger& operator<<(const spdlog::level::level_enum& lvl) {
		if(!_buffer.empty()) {
			_buffer = std::to_string(cur_sec()) + " " + _buffer;
			_logger->log(lvl, _buffer);
			_logger->flush();
			_buffer.clear();
		}
		return *this;
	}

};


#endif
