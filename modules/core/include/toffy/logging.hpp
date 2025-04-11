#pragma once

#include <string>
#include <sstream>

/**
 * logging: abstract from boost log 
 */

namespace toffy {
namespace log {

enum logLevel
{
    trace,
    debug,
    info,
    warning,
    error,
    fatal
};

const char endl = '\n';  // NO flushing

// mix-in class with log methods
class Log
{
    std::string tag;
    logLevel level;

   public:
    /** helper struct for finding the end of the << term to emit the log line.
     *  cf. https://stackoverflow.com/questions/3497181/operator-how-to-detect-last-argument
     * the destructor is called on the last element...
     */
    class Logger;

    struct LogMsg
    {
        std::stringstream m;
        const Logger& log;

        template <typename T>
        LogMsg(const Logger& log, T const& data) : log(log)
        {
            this->operator<<(data);
        }
        // destruct and log to output stream:
        ~LogMsg() { log.sink.log(log.level, m.str()); }

        template <typename T>
        LogMsg& operator<<(T const& data)
        {
            m << data;

            return *this;
        }
        // some special data types:

        /// output void pointer
        LogMsg& operator<<(void* data)
        {
            m << std::hex << "[ptr]0x" << (unsigned long)data << std::dec;

            return *this;
        }
    };

    struct Logger
    {
        Log& sink;
        // std::stringstream strm;

        logLevel level;

        Logger(Log& sink, logLevel level) : sink(sink), level(level) {}

        // return a LogMsg for tracking the concatenated << term
        template <typename T>
        LogMsg operator<<(T const& val)
        {
            return LogMsg(*this, val);
        }
    };

   public:
    Log(const std::string& tag = "XXX") : tag(tag) {}
    virtual ~Log() {}

    void setLevel(logLevel level) { this->level = level; };
    logLevel getLevel() const { return level; }

    Logger debug = Logger(*this, logLevel::debug);

    Logger info = Logger(*this, logLevel::info);

    Logger warning = Logger(*this, logLevel::warning);

    Logger error = Logger(*this, logLevel::error);

    Logger& getLogger(logLevel l);

    protected:
    void log(Logger& l);
    void log(logLevel l, const std::string& m);
};

extern Log* theLogger;

#define LOG_D(logPtr) ((logPtr)->debug)
#define LOG_I(logPtr) ((logPtr)->info)
#define LOG_W(logPtr) ((logPtr)->warning)
#define LOG_E(logPtr) ((logPtr)->error)

#define LOGD LOG_D(toffy::log::theLogger)
#define LOGI LOG_I(toffy::log::theLogger)
#define LOGW LOG_W(toffy::log::theLogger)
#define LOGE LOG_E(toffy::log::theLogger)


}  // namespace log
}  // namespace toffy