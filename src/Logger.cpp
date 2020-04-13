#include <boost/filesystem.hpp>

#include "Logger.h"

namespace fs = boost::filesystem;

Logger::Logger(std::string log_name) {
	fs::path cur_path = fs::current_path().append("logs");
	if(!fs::exists(cur_path)) {
		fs::create_directory(cur_path);
	}

	_logger = spdlog::basic_logger_mt<spdlog::async_factory>(log_name, "logs/" + log_name + ".log", true);
	_logger->set_pattern("[%T] [%=3!l] [%i] %v");
}

void Logger::log(std::string msg, spdlog::level::level_enum lvl) {
	_logger->log(lvl, msg);
	_logger->flush();
}

// Schreibt _buffer in die Datei
void Logger::log_buffer(spdlog::level::level_enum lvl) {
	if(!_buffer.empty()) {
		_logger->log(lvl, _buffer);
		_logger->flush();
		_buffer.clear();
	}
}

void Logger::set_level(spdlog::level::level_enum lvl) {
	_logger->set_level(lvl);
}
