// snes emulator
// core/src/system/Logging.cpp
// Thread-safe logging and diagnostic output.

#include "snes/core/Logging.hpp"

#include <iostream>
#include <utility>

namespace snes::core {

namespace {

const char* ToString(LogLevel level) {
    switch (level) {
    case LogLevel::Debug: return "DEBUG";
    case LogLevel::Info: return "INFO";
    case LogLevel::Warning: return "WARN";
    case LogLevel::Error: return "ERROR";
    default: return "UNKNOWN";
    }
}

} // namespace

Logger& Logger::Instance() {
    static Logger instance;
    return instance;
}

void Logger::SetSink(LogSink sink) {
    std::scoped_lock lock(mutex_);
    sink_ = std::move(sink);
}

void Logger::Write(LogLevel level, std::string message) {
    std::scoped_lock lock(mutex_);

    if (sink_) {
        sink_(level, message);
        return;
    }

    std::cout << '[' << ToString(level) << "] " << message << '\n';
}

} // namespace snes::core
