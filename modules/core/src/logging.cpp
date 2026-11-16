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
 *        knows which logging library it is built against (P2-17).
 *
 * Currently Boost.Log. The mapping rules that have to survive a swap are in
 * cleanup/plan/logging.md; the one that bites is that an unconfigured Boost.Log
 * writes everything to stderr while an unconfigured log4cplus has no appender
 * on the root logger and drops every record silently.
 */
#include <toffy/logging.hpp>

#include <cctype>
#include <map>
#include <mutex>
#include <utility>

#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/utility/setup.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/file.hpp>

namespace toffy {
namespace log {

namespace {

constexpr int kInherit = -1;

/**
 * The channel table.
 *
 * Leaked on purpose (`new`, never deleted): toffy logs from destructors, and a
 * `Player` or `Filter` torn down during static destruction would otherwise be
 * calling into a table that has already run its own destructor. A few kilobytes
 * of map at exit is the cheaper failure.
 */
struct channel_table {
    /**
     * Recursive, not because the lock is taken twice on purpose but because
     * `effectiveLevel()` falls back to the root channel, and the root channel
     * is created on demand by that same lookup. Depth is bounded at two. The
     * alternative -- a non-locking twin of every accessor, kept in step by hand
     * -- is the kind of thing that deadlocks six months later.
     */
    std::recursive_mutex m;
    std::map<std::string, logger> channels;
    bool configured = false;
    bool fileSinkInstalled = false;
};

channel_table& table() {
    static channel_table* t = new channel_table();
    return *t;
}

const logger& root_channel() { return logger::root(); }

}  // namespace

logger::logger(std::string name, int ownLevel)
    : name_(std::move(name)), ownLevel_(ownLevel) {}

const logger& logger::root() {
    channel_table& t = table();
    std::lock_guard<std::recursive_mutex> lock(t.m);
    auto it = t.channels.find(std::string());
    if (it == t.channels.end()) {
        // Unconfigured means "filter nothing", which is what BOOST_LOG_TRIVIAL
        // does with no filter installed. Anything else would silently quiet
        // every program that never calls init().
        it = t.channels
                 .emplace(std::string(),
                          logger(std::string(), static_cast<int>(level::trace)))
                 .first;
    }
    return it->second;
}

const logger& logger::get(const std::string& name) {
    if (name.empty()) {
        return root();
    }
    channel_table& t = table();
    std::lock_guard<std::recursive_mutex> lock(t.m);
    auto it = t.channels.find(name);
    if (it == t.channels.end()) {
        it = t.channels.emplace(name, logger(name, kInherit)).first;
    }
    return it->second;
}

const std::string& logger::name() const { return name_; }

level logger::effectiveLevel() const {
    channel_table& t = table();
    std::lock_guard<std::recursive_mutex> lock(t.m);
    // Walk the dotted name upwards to the nearest ancestor that exists, and
    // stop at the first one with a pinned level. Skipping over missing ones is
    // not a shortcut: log4cplus creates the intermediate loggers of a name
    // implicitly, so setting "toffy" does apply to "toffy.filters.rectify"
    // even when nothing ever asked for "toffy.filters". Stopping at the gap
    // here would make that work under log4cplus and not under this backend,
    // which is the kind of difference that only shows up in production.
    const logger* cur = this;
    for (int depth = 0; cur != nullptr && depth < 64; ++depth) {
        if (cur->ownLevel_ != kInherit) {
            return static_cast<level>(cur->ownLevel_);
        }
        std::string ancestor = cur->name_;
        const logger* parent = nullptr;
        while (!ancestor.empty() && parent == nullptr) {
            const std::string::size_type dot = ancestor.rfind('.');
            if (dot == std::string::npos) {
                ancestor.clear();  // top level: the root decides
                break;
            }
            ancestor = ancestor.substr(0, dot);
            auto it = t.channels.find(ancestor);
            if (it != t.channels.end()) {
                parent = &it->second;
            }
        }
        cur = parent;
    }
    return static_cast<level>(root_channel().ownLevel_);
}

bool logger::enabled(level lvl) const {
    return static_cast<int>(lvl) >= static_cast<int>(effectiveLevel());
}

void logger::setLevel(level lvl) const {
    // const because a `const logger&` from get() is the only way anyone holds
    // a channel; the state is process configuration, not object state.
    channel_table& t = table();
    std::lock_guard<std::recursive_mutex> lock(t.m);
    // Mutable through the const handle: the table owns the storage.
    const_cast<logger*>(this)->ownLevel_ = static_cast<int>(lvl);
}

void logger::inheritLevel() const {
    channel_table& t = table();
    std::lock_guard<std::recursive_mutex> lock(t.m);
    const_cast<logger*>(this)->ownLevel_ = kInherit;
}

void logger::emit(level lvl, const std::string& text) const {
    if (!enabled(lvl)) {
        return;
    }
    // Boost's own severity filter is left wide open (see init()): toffy's layer
    // is the single source of truth for what is emitted, so a channel can be
    // more verbose than the root and the rule is the same one log4cplus will
    // implement natively.
    switch (lvl) {
        case level::trace:
            BOOST_LOG_TRIVIAL(trace) << text;
            break;
        case level::debug:
            BOOST_LOG_TRIVIAL(debug) << text;
            break;
        case level::info:
            BOOST_LOG_TRIVIAL(info) << text;
            break;
        case level::warning:
            BOOST_LOG_TRIVIAL(warning) << text;
            break;
        case level::error:
            BOOST_LOG_TRIVIAL(error) << text;
            break;
        case level::fatal:
            BOOST_LOG_TRIVIAL(fatal) << text;
            break;
        case level::off:
            break;
    }
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
        lowered.push_back(static_cast<char>(
            std::tolower(static_cast<unsigned char>(c))));
    }
    if (lowered == "trace") return level::trace;
    if (lowered == "debug") return level::debug;
    if (lowered == "info") return level::info;
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
    channel_table& t = table();

    if (!cfg.propertiesFile.empty()) {
        // The properties file is the log4cplus half of P2-17. Until the backend
        // is log4cplus there is nothing here that can read it, and saying
        // nothing would mean a deployment silently logging to the console
        // because a file it pointed at was never opened.
        root_channel().emit(level::warning,
                            "log config: properties file '" + cfg.propertiesFile +
                                "' ignored -- this build's backend is Boost.Log "
                                "and cannot read it (see cleanup/plan/logging.md)");
    }

    // Boost's core filter stays wide open; toffy's layer decides. Setting it to
    // the root level instead would make a channel that is more verbose than the
    // root silently lose records, which is a rule the log4cplus backend will
    // not have.
    boost::log::core::get()->set_filter(
        boost::log::trivial::severity >= boost::log::trivial::trace);

    if (cfg.fileSink) {
        // At most once per process: Boost exposes no way to enumerate or
        // replace registered sinks, so a second add_file_log() would leave two
        // file sinks and duplicate every record. This is the constraint P2-8
        // recorded on Player's ctor, moved to where the configuration lives.
        if (!t.fileSinkInstalled) {
            t.fileSinkInstalled = true;
            boost::log::add_file_log(
                boost::log::keywords::file_name = "toffy_%N.log",
                boost::log::keywords::rotation_size = 10 * 1024 * 1024,
                boost::log::keywords::time_based_rotation =
                    boost::log::sinks::file::rotation_at_time_point(0, 0, 0),
                boost::log::keywords::format = "[%TimeStamp%]: %Message%",
                boost::log::keywords::auto_flush = true,
                boost::log::keywords::open_mode =
                    (std::ios::out | std::ios::app));
        } else {
            root_channel().emit(level::warning,
                                "log config: a file sink is already installed; "
                                "this build cannot replace it, level only");
        }
    }

    boost::log::add_common_attributes();

    {
        std::lock_guard<std::recursive_mutex> lock(t.m);
        t.configured = true;
    }

    // Applied last, so that the warnings above always reach the user whatever
    // the new level says.
    if (cfg.propertiesFile.empty()) {
        root_channel().setLevel(cfg.minLevel);
    }
}

void initOnce(const config& cfg) {
    channel_table& t = table();
    {
        std::lock_guard<std::recursive_mutex> lock(t.m);
        if (t.configured) {
            return;
        }
    }
    init(cfg);
}

bool isConfigured() {
    channel_table& t = table();
    std::lock_guard<std::recursive_mutex> lock(t.m);
    return t.configured;
}

}  // namespace log
}  // namespace toffy
