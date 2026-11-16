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
 * @brief toffy's logging API: levels, channels, stream records.
 * @ingroup Core
 *
 * This header includes `<memory>`, `<sstream>`, `<string>` and `<utility>` and
 * nothing else -- no logging backend, no Boost. That is the entire point of it:
 * five installed headers used to include `boost/log/trivial.hpp`, and
 * `toffy::Player::Player(boost::log::v2_mt_posix::trivial::severity_level, bool)`
 * was a symbol in both consumers' undefined-symbol tables. A consumer's ELF
 * named a Boost type. See cleanup/plan/logging.md (P2-17).
 *
 * The backend is chosen in `src/logging.cpp`, not here, so swapping it changes
 * no call site and no header.
 *
 * Two names are deliberately absent: a free function `level()` and a
 * `config::level` member. Both would collide with the `level` type in their
 * scope. Hence `enabled()`, `effectiveLevel()` and `minLevel`.
 */
#ifndef TOFFY_LOGGING_HPP
#define TOFFY_LOGGING_HPP

#include <memory>
#include <sstream>
#include <string>
#include <utility>

namespace toffy {
namespace log {

/**
 * @brief Severity levels.
 *
 * The numbers are a wire format, not an implementation detail: `Filter` writes
 * its level into its config as an integer (`options.loglvl`) and reads it back
 * clamped to `trace..fatal`, so deployed pipeline files contain 0..5.
 * Renumbering this enum silently re-levels every one of them.
 *
 * Lowercase enumerators are deliberate too: they let the existing `LOG(debug)`
 * macro expand to `toffy::log::level::debug` and keep its 100 call sites
 * untouched, and they make `toffy::log::debug()` and `level::debug` read the
 * same way.
 */
enum class level {
    trace = 0,
    debug = 1,
    info = 2,
    warning = 3,
    error = 4,
    fatal = 5,
    off = 6,  ///< above fatal: nothing is emitted
};

class logger;

/**
 * @brief One log record: stream into it, it is written when the statement ends.
 *
 * Obtained from a `logger` (`logger::debug()`, ...), or from the free
 * functions below, which are the root channel's.
 *
 * Two properties worth knowing before using one in a loop:
 *
 *   - The operands of `<<` are always evaluated, even when the level is off:
 *     `debug() << describe(frame)` calls `describe()`. This is what
 *     `BOOST_LOG_TRIVIAL` does too, and it is not fixable without giving up
 *     `operator<<`. `logger::enabled()` is the way to not call it.
 *   - A disabled record allocates nothing. The stream buffer is created on the
 *     first `<<` and only if the level is on, because 455 of toffy's 658 log
 *     sites are `debug` and P2-8 had to remove three log calls off the
 *     per-frame path.
 */
class record {
public:
    record(const logger& owner, level lvl);

    /** Writes the record. Never throws: a log line must not break a pipeline. */
    ~record() noexcept;

    record(record&& other) noexcept;
    record(const record&) = delete;
    record& operator=(const record&) = delete;

    template <typename T>
    record& operator<<(T&& value) {
        if (on_) {
            buffer() << std::forward<T>(value);
        }
        return *this;
    }

    /**
     * @brief Stream manipulators: `std::endl`, `std::flush`, `std::hex`, ...
     *
     * Not covered by the template above: `std::endl` is an overloaded function
     * template, so `T&&` cannot deduce it. It needs the exact signature. This
     * is not hypothetical -- `<< endl` appears in toffy's log lines today, and
     * a record that could not take one would not be a drop-in for the stream
     * `BOOST_LOG_TRIVIAL` handed back.
     */
    record& operator<<(std::ostream& (*manip)(std::ostream&)) {
        if (on_) {
            buffer() << manip;
        }
        return *this;
    }

private:
    /// The buffer, created on the first thing actually streamed. A record whose
    /// level is off never gets here, which is the point: constructing a
    /// std::ostringstream is not free and 455 of toffy's 658 log sites are
    /// `debug`.
    std::ostringstream& buffer() {
        if (!buf_) {
            buf_.reset(new std::ostringstream());
        }
        return *buf_;
    }

    const logger* owner_;
    level lvl_;
    bool on_;
    std::unique_ptr<std::ostringstream> buf_;
};

/**
 * @brief A named channel. Dotted names form a hierarchy.
 *
 * `"toffy.filters.rectify"` inherits from `"toffy.filters"` inherits from the
 * root channel (`""`). A channel exists to answer one question -- is a record
 * at this level emitted here -- so that one filter class can be turned up
 * without turning the whole process up with it.
 *
 * Channel names are not written into the message. Formatting is the backend's
 * and the application's business (log4cplus can put `%c` in its pattern); the
 * message text stays whatever the call site streamed.
 *
 * Instances are held by the channel table and are cheap to copy: a `logger` is
 * a handle to stable storage, so holding one in a member is safe.
 */
class logger {
public:
    /**
     * @brief Look up (creating on first use) a channel.
     *
     * The returned reference stays valid for the process, so it may be cached.
     */
    static const logger& get(const std::string& name);

    /** The root channel, name `""`. Everything inherits from it. */
    static const logger& root();

    const std::string& name() const;

    record trace() const { return record(*this, level::trace); }
    record debug() const { return record(*this, level::debug); }
    record info() const { return record(*this, level::info); }
    record warning() const { return record(*this, level::warning); }
    record error() const { return record(*this, level::error); }
    record fatal() const { return record(*this, level::fatal); }
    record message(level lvl) const { return record(*this, lvl); }

    /** @brief True if a record at `lvl` would reach a sink here. */
    bool enabled(level lvl) const;

    /** @brief Pin this channel's level, overriding any inherited one. */
    void setLevel(level lvl) const;

    /** @brief Stop pinning; inherit from the parent again. */
    void inheritLevel() const;

    /** @brief Own level if pinned, else the nearest ancestor's, else root's. */
    level effectiveLevel() const;

    /**
     * @brief Write one already-formatted record.
     *
     * The path `record`'s destructor takes. Filters through `enabled()`, so it
     * is safe to call unconditionally.
     */
    void emit(level lvl, const std::string& text) const;

private:
    /** Held by the channel table only; `get()` and `root()` are the entrances. */
    logger(std::string name, int ownLevel);

    std::string name_;
    int ownLevel_;  ///< -1: inherit from the parent; else a `level` value
};

/** @brief The root channel. */
inline const logger& root() { return logger::root(); }

/** @brief Look up a channel by dotted name. */
inline const logger& getLogger(const std::string& name) { return logger::get(name); }

inline record trace() { return root().trace(); }
inline record debug() { return root().debug(); }
inline record info() { return root().info(); }
inline record warning() { return root().warning(); }
inline record error() { return root().error(); }
inline record fatal() { return root().fatal(); }
inline record message(level lvl) { return root().message(lvl); }
inline bool enabled(level lvl) { return root().enabled(lvl); }

/**
 * @brief Parse a level name, case-insensitively.
 *
 * Returns `fallback` for anything unparseable, which is what
 * `Filter::setLogLevel()` has always done with an unusable `<loglvl>`.
 */
level parseLevel(const std::string& name, level fallback = level::info);

/** @brief The canonical lowercase name; round-trips through parseLevel(). */
const char* name(level lvl);

/**
 * @brief Process-wide logging configuration. Call it first thing in `main()`.
 *
 * Logging is application state. It is not read from pipeline XML, and nothing
 * in toffy will change it when a pipeline is loaded -- that conflation is the
 * bug P2-8 removed, when a per-filter method was reconfiguring the whole
 * process three times per child per frame.
 */
struct config {
    /**
     * @brief Minimum severity that is emitted.
     *
     * Ignored when `propertiesFile` is set, so that a carefully written
     * backend config is not clobbered by a caller's default.
     */
    level minLevel = level::debug;

    /**
     * @brief Path to a backend config file (log4cplus properties format).
     *
     * When set, the backend owns levels, formats, appenders and per-channel
     * levels and `minLevel` is not applied.
     */
    std::string propertiesFile;

    /**
     * @brief Log to a file instead of the console.
     *
     * Fixed parameters (`toffy_%N.log`, 10 MiB, daily rotation, append,
     * unbuffered) because that is what `Player(level, true)` has always done.
     * A properties file is how you get to choose; until the backend can read
     * one this is the only knob.
     */
    bool fileSink = false;
};

/**
 * @brief Apply `cfg`. Repeatable: the level follows the last call.
 *
 * A file sink is installed at most once per process, because the Boost.Log
 * core exposes no way to enumerate or replace the sinks already registered;
 * calling init() twice with `fileSink` changes the level and says so.
 */
void init(const config& cfg);

/** @brief Apply `cfg` only if nothing has configured logging yet. */
void initOnce(const config& cfg);

/** @brief True once init() or initOnce() has applied a config. */
bool isConfigured();

}  // namespace log
}  // namespace toffy

#endif
