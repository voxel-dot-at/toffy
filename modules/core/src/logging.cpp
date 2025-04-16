#include <stdarg.h>
#include <iostream>

#include <toffy/logging.hpp>

using namespace toffy::log;

Log* toffy::log::theLogger = new Log("TOFFY");

void Log::log(logLevel l, const std::string& m)
{
    if (l < level) {
        // l.clear();
        return;
    }
    std::cout << "[" << l << "] ";
    std::cout << "[" << tag << "]\t" << m << std::endl;
}

Log::Logger& Log::getLogger(logLevel l)
{
    switch (l) {
        case toffy::log::debug:
            return this->debug;
        case toffy::log::info:
            return this->info;
        case toffy::log::warning:
            return this->error;
        case toffy::log::error:
            return this->error;
        default:
            throw std::runtime_error("unhandled log level in Log::getLogger");
    }
}