#include "Logger.h"
#include "boost/filesystem.hpp"

namespace fs = boost::filesystem;

Logger::Logger(std::string log_name) {
	fs::path cur_path = fs::current_path().append("logs");
	if(!fs::exists(cur_path)) {
		fs::create_directory(cur_path);
	}

	_logger = spdlog::basic_logger_mt<spdlog::async_factory>(channel_name, "logs/" + channel_name + ".log");
	_logger->set_pattern("[%^%l%$] %T %v");
}

void Logger::log(std::string msg, spdlog::level::level_enum lvl) {
	msg = std::to_string(cur_ms()) + " " + msg;
	_logger->log(lvl, msg);
	_logger->flush();
}

// Schreibt _buffer in die Datei
void Logger::log_buffer(spdlog::level::level_enum lvl) {
	if(!_buffer.empty()) {
		_buffer = std::to_string(cur_ms()) + " " + _buffer;
		_logger->log(lvl, _buffer);
		_logger->flush();
		_buffer.clear();
	}
}
