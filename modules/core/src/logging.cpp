/*
   Copyright 2026 Simon Vogl <svogl@voxel.at>

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/
/**
 * @file
 * @brief The logging backend. This is the only translation unit in toffy that
 *        names a logging library (P2-17).
 *
 * log4cplus. The rules that have to hold whatever is behind this file are in
 * cleanup/plan/logging.md, and they are pinned by tests/test_logging_abstraction.cpp,
 * which names no backend and has to pass against anything.
 *
 * Most of what the abstraction does is things log4cplus already does — the
 * dotted hierarchy, inheritance through names that were never requested,
 * per-logger levels — so this file is mostly mapping. The two places where it
 * is not a mapping are both about defaults:
 *
 *   - An unconfigured log4cplus root logger has no appender and drops every
 *     record without saying so. toffy's contract is that a program which
 *     configures nothing still gets its records, on stderr, unfiltered, so a
 *     default appender is installed on first use.
 *   - `PropertyConfigurator::doConfigure()` does not reset the hierarchy first
 *     (its own documentation says to do it), so `init()` does. Without that, a
 *     second `init()` would merge with the first instead of replacing it and
 *     "the level follows the last call" would be false.
 */
#include <toffy/logging.hpp>

#include <cctype>
#include <map>
#include <mutex>
#include <utility>

#include <log4cplus/configurator.h>
#include <log4cplus/fileappender.h>
#include <log4cplus/hierarchy.h>
#include <log4cplus/consoleappender.h>
#include <log4cplus/logger.h>
#include <log4cplus/loglevel.h>
#include <log4cplus/tstring.h>

namespace toffy {
namespace log {

namespace {

log4cplus::LogLevel toBackend(level lvl) {
    switch (lvl) {
        case level::trace: return log4cplus::TRACE_LOG_LEVEL;
        case level::debug: return log4cplus::DEBUG_LOG_LEVEL;
        case level::info: return log4cplus::INFO_LOG_LEVEL;
        case level::warning: return log4cplus::WARN_LOG_LEVEL;
        case level::error: return log4cplus::ERROR_LOG_LEVEL;
        case level::fatal: return log4cplus::FATAL_LOG_LEVEL;
        case level::off: return log4cplus::OFF_LOG_LEVEL;
    }
    return log4cplus::INFO_LOG_LEVEL;
}

level fromBackend(log4cplus::LogLevel ll) {
    // `getChainedLogLevel()` resolves inheritance, so it never returns
    // NOT_SET_LOG_LEVEL; the comparisons below are against the resolved value.
    if (ll >= log4cplus::OFF_LOG_LEVEL) return level::off;
    if (ll >= log4cplus::FATAL_LOG_LEVEL) return level::fatal;
    if (ll >= log4cplus::ERROR_LOG_LEVEL) return level::error;
    if (ll >= log4cplus::WARN_LOG_LEVEL) return level::warning;
    if (ll >= log4cplus::INFO_LOG_LEVEL) return level::info;
    if (ll >= log4cplus::DEBUG_LOG_LEVEL) return level::debug;
    return level::trace;
}

log4cplus::Logger backend_logger(const std::string& name) {
    if (name.empty()) {
        return log4cplus::Logger::getRoot();
    }
    return log4cplus::Logger::getInstance(log4cplus::tstring(name));
}

/**
 * The handle table.
 *
 * Only the `logger` objects live here, so that `getLogger()` can hand back a
 * reference that stays valid — callers cache them. The levels are log4cplus's.
 *
 * Leaked on purpose (`new`, never deleted): toffy logs from destructors, and a
 * `Player` or `Filter` torn down during static destruction would otherwise be
 * calling into a table that has already run its own destructor. A few hundred
 * bytes of map at exit is the cheaper failure.
 */
struct handle_table {
    std::recursive_mutex m;
    std::map<std::string, logger> handles;
    bool defaultInstalled = false;
    bool configured = false;
};

handle_table& table() {
    static handle_table* t = new handle_table();
    return *t;
}

/**
 * Give the root logger somewhere to write, once.
 *
 * This is the silent-regression trap of the whole item. Boost.Log's trivial
 * logger creates a stderr sink on first use, so "the application configured
 * nothing" produced output. An unconfigured log4cplus root has no appender at
 * all and discards every record without a word. A toffy that built clean,
 * passed every test and logged nothing in the field is the failure mode this
 * prevents.
 */
void install_default_appender() {
    handle_table& t = table();
    std::lock_guard<std::recursive_mutex> lock(t.m);
    if (t.defaultInstalled) {
        return;
    }
    t.defaultInstalled = true;

    log4cplus::SharedAppenderPtr console(
        new log4cplus::ConsoleAppender(/*logToStdErr=*/true,
                                       /*immediateFlush=*/true));
    log4cplus::Logger::getRoot().addAppender(console);
    // Unconfigured means "filter nothing", which is what the previous backend
    // did with no filter installed. Anything else would quietly quiet every
    // program that never calls init().
    log4cplus::Logger::getRoot().setLogLevel(log4cplus::TRACE_LOG_LEVEL);
}

}  // namespace

logger::logger(std::string name) : name_(std::move(name)) {}

const logger& logger::root() {
    handle_table& t = table();
    std::lock_guard<std::recursive_mutex> lock(t.m);
    auto it = t.handles.find(std::string());
    if (it == t.handles.end()) {
        it = t.handles.emplace(std::string(), logger(std::string())).first;
    }
    return it->second;
}

const logger& logger::get(const std::string& name) {
    if (name.empty()) {
        return root();
    }
    handle_table& t = table();
    std::lock_guard<std::recursive_mutex> lock(t.m);
    auto it = t.handles.find(name);
    if (it == t.handles.end()) {
        it = t.handles.emplace(name, logger(name)).first;
    }
    return it->second;
}

const std::string& logger::name() const { return name_; }

level logger::effectiveLevel() const {
    install_default_appender();
    return fromBackend(backend_logger(name_).getChainedLogLevel());
}

bool logger::enabled(level lvl) const {
    install_default_appender();
    return backend_logger(name_).isEnabledFor(toBackend(lvl));
}

void logger::setLevel(level lvl) const {
    install_default_appender();
    // const because a `const logger&` from get() is the only way anyone holds a
    // channel; the level is process configuration, not object state.
    backend_logger(name_).setLogLevel(toBackend(lvl));
}

void logger::inheritLevel() const {
    install_default_appender();
    backend_logger(name_).setLogLevel(log4cplus::NOT_SET_LOG_LEVEL);
}

void logger::emit(level lvl, const std::string& text) const {
    if (!enabled(lvl)) {
        return;
    }
    backend_logger(name_).log(toBackend(lvl), log4cplus::tstring(text));
}

record::record(const logger& owner, level lvl)
    : owner_(&owner), lvl_(lvl), on_(owner.enabled(lvl)), buf_() {}

record::~record() noexcept {
    try {
        if (on_ && buf_ && !buf_->str().empty()) {
            owner_->emit(lvl_, buf_->str());
        }
    } catch (...) {
        // A log line must not be the reason a pipeline died.
    }
}

record::record(record&& other) noexcept
    : owner_(other.owner_), lvl_(other.lvl_), on_(other.on_),
      buf_(std::move(other.buf_)) {
    other.on_ = false;  // the moved-from record emits nothing
}

level parseLevel(const std::string& text, level fallback) {
    std::string lowered;
    lowered.reserve(text.size());
    for (char c : text) {
        lowered.push_back(
            static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    }
    if (lowered == "trace") return level::trace;
    if (lowered == "debug") return level::debug;
    if (lowered == "info") return level::info;
    // "warn" is an alias pipeline files use; dropping it would silently
    // downgrade every one of them to the fallback.
    if (lowered == "warning" || lowered == "warn") return level::warning;
    if (lowered == "error") return level::error;
    if (lowered == "fatal") return level::fatal;
    if (lowered == "off") return level::off;
    return fallback;
}

const char* name(level lvl) {
    switch (lvl) {
        case level::trace: return "trace";
        case level::debug: return "debug";
        case level::info: return "info";
        case level::warning: return "warning";
        case level::error: return "error";
        case level::fatal: return "fatal";
        case level::off: return "off";
    }
    return "info";
}

void init(const config& cfg) {
    handle_table& t = table();

    if (!cfg.propertiesFile.empty()) {
        // doConfigure() does not reset the hierarchy first — its own
        // documentation says the caller should. Doing it here is what makes
        // "repeatable: the level follows the last call" true rather than
        // merged, and it means a properties file configures the logging, not
        // augments whatever was already there.
        log4cplus::Logger::getDefaultHierarchy().resetConfiguration();
        log4cplus::PropertyConfigurator::doConfigure(
            log4cplus::tstring(cfg.propertiesFile));
        {
            std::lock_guard<std::recursive_mutex> lock(t.m);
            t.defaultInstalled = true;  // the file owns the appenders now
            t.configured = true;
        }
        // minLevel is deliberately not applied: a carefully written file owns
        // the levels, including per-channel ones.
        return;
    }

    // Replace the sink rather than adding to it: `init()` means "configure",
    // not "augment", and a second call that left two appenders on the root
    // would duplicate every record. `fileSink` has always meant "to a file
    // instead of to the console", not "to both".
    //
    // The file name and rotation differ from the previous backend: it wrote
    // `toffy_%N.log` with both a 10 MiB and a midnight rotation configured, and
    // log4cplus does one or the other per appender. Size rotation at 10 MiB
    // with five backups is the closer of the two, and it is the one that bounds
    // disk use, which is what the setting was for.
    log4cplus::SharedAppenderPtr sink;
    if (cfg.fileSink) {
        sink = log4cplus::SharedAppenderPtr(new log4cplus::RollingFileAppender(
            log4cplus::tstring("toffy.log"), 10 * 1024 * 1024, 5,
            /*immediateFlush=*/true));
    } else {
        sink = log4cplus::SharedAppenderPtr(new log4cplus::ConsoleAppender(/*logToStdErr=*/true,
                                                 /*immediateFlush=*/true));
    }
    log4cplus::Logger::getRoot().removeAllAppenders();
    log4cplus::Logger::getRoot().addAppender(sink);

    {
        std::lock_guard<std::recursive_mutex> lock(t.m);
        t.defaultInstalled = true;  // this call configured the sink
        t.configured = true;
    }

    root().setLevel(cfg.minLevel);
}

void initOnce(const config& cfg) {
    handle_table& t = table();
    std::lock_guard<std::recursive_mutex> lock(t.m);
    if (t.configured) {
        return;
    }
    init(cfg);
}

bool isConfigured() {
    handle_table& t = table();
    std::lock_guard<std::recursive_mutex> lock(t.m);
    return t.configured;
}

}  // namespace log
}  // namespace toffy
