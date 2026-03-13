#pragma once

#include <functional>
#include <mutex>
#include <string>

namespace snes::core {

enum class LogLevel {
    Debug,
    Info,
    Warning,
    Error
};

using LogSink = std::function<void(LogLevel level, const std::string& message)>;

class Logger {
public:
    static Logger& Instance();

    void SetSink(LogSink sink);
    void Write(LogLevel level, std::string message);

private:
    Logger() = default;

    std::mutex mutex_;
    LogSink sink_;
};

} // namespace snes::core
