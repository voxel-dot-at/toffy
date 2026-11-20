# P2-17 — a toffy-owned logging API, so the backend becomes a choice

New item, opened by the log4cplus decision. Not in the original `CLEANUP_PLAN.md`.

## Why

Boost.Log is the only reason `libtoffy.so` pulls `log` and `log_setup`, and the
only reason a consumer needs Boost.Log **headers** to compile against toffy.
Measured on `main` (2026-11-16):

| | |
|---|---|
| `BOOST_LOG_TRIVIAL(...)` sites | 658 in 58 files (`debug` 455, `warning` 131, `error` 33, `info` 30, `trace` 8, `fatal` 0) |
| `LOG(lvl)` sites (toffy's own macro) | 100 — consumers use it **0** times |
| `boost/log` includes | 76, of which **5 are in installed headers**: `filter.hpp`, `filter_helpers.hpp`, `player.hpp`, `base/distAmpl.hpp`, `cam/cameraParams.hpp` |
| Public signatures carrying a Boost.Log type | `Filter::setGlobalLogLevel(boost::log::trivial::severity_level)`, `Player(boost::log::trivial::severity_level, bool)` |
| Consumer lines that name Boost.Log | **3**, all `new toffy::Player(boost::log::trivial::debug, false)` |
| `boxes` own logging | 378 sites in 29 files, plus its own `find_package(Boost COMPONENTS log_setup log …)` |

The last line of `ci/consumer-link-surface.txt` is the dependency made visible
in ELF:

```
becom-knapp-wurthbox toffy::Player::Player(boost::log::v2_mt_posix::trivial::severity_level, bool)
```

A consumer's undefined-symbol table names a Boost type. That is the thing to
remove; the 658 `.cpp` sites are what makes the backend swappable, not what
makes it required.

## Decisions

| # | Decision | Outcome |
|---|---|---|
| 1 | Level spelling | **lowercase** — `enum class level { trace, debug, … }`, so the existing `LOG(debug)` keeps compiling and `toffy::log::debug()` reads the same way |
| 2 | Channels | **now**, not later |
| 3 | Version | **stay on major 2**. 2.0 is unreleased and the upgrade is in flight, so the ABI break is free; no compat shims, no `logging_boost_compat.hpp` |
| 4 | Logging configuration | **in code, from `main()`** — see below |
| 5 | Default sink | console at `info`; a log4cplus properties file overrides everything |

### 4, and why it is not a `<globals>` node

The obvious place — `<globals><logging>` in the pipeline XML — is the place
`P2-8` deleted. `P2-8`'s bug was `Filter::setLoggingLvl()` calling
`logging::core::get()->set_filter(...)` from inside a per-filter method, three
times per child per frame, so the process-wide level was whatever the last
filter to run wanted. The fix, recorded at `filterbank.cpp:49`: *"The logging
level is application state now."*

A `<logging>` node inside the pipeline config re-imports that bug with a
different spelling: `FilterBank::loadGlobals()` runs from `Player::loadConfig()`,
which is callable at runtime, so **loading a pipeline would reconfigure process
logging**. Same shape, same failure. Pipeline XML describes a pipeline; logging
describes the process. They do not share a file.

So: the application configures logging in code, once, before it touches a
pipeline. `toffy::log::init()` takes the log4cplus properties path; where that
path came from (CLI flag, the app's own config, a hardcoded default) is the
application's business and toffy has no opinion. Neither consumer currently has
an application-level config file — both take a pipeline XML as their only
argument — so there is nothing to move and nothing to break.

## The API

`modules/core/include/toffy/logging.hpp`. Includes `<memory> <sstream> <string>
<utility>` and nothing else — no backend, no Boost.

```cpp
namespace toffy {
namespace log {

/**
 * Severity. The numbers are a wire format: Filter writes its level to config as
 * an integer (`options.loglvl`) and reads it back clamped to trace..fatal, so
 * deployed configs contain 0..5. Renumbering silently re-levels them.
 */
enum class level { trace = 0, debug = 1, info = 2, warning = 3, error = 4,
                   fatal = 5, off = 6 };

class logger;

/** One record. Streams into it; it is written when the statement ends. */
class record {
public:
    record(const logger& owner, level lvl);
    ~record() noexcept;                       // emission never throws
    record(record&&) noexcept;
    record(const record&) = delete;
    record& operator=(const record&) = delete;

    template <typename T>
    record& operator<<(T&& value) {
        if (on_) { *buf_ << std::forward<T>(value); }
        return *this;
    }

private:
    const logger* owner_;
    level lvl_;
    bool on_;
    std::unique_ptr<std::ostringstream> buf_;   // allocated only when on_
};

/** A named channel. Dotted names form a hierarchy: "toffy.filters.rectify"
 *  inherits from "toffy.filters" inherits from root. */
class logger {
public:
    const std::string& name() const;

    record trace() const;   record debug() const;   record info() const;
    record warning() const; record error() const;   record fatal() const;
    record message(level lvl) const;

    bool enabled(level lvl) const;             // lvl >= effectiveLevel()
    void setLevel(level lvl);                  // pin this channel
    void inheritLevel();                       // follow the parent again
    level effectiveLevel() const;              // own, else nearest ancestor's
};

const logger& root();                          // name ""
const logger& getLogger(const std::string& name);   // cached; reference is stable

inline record trace()             { return root().trace(); }
inline record debug()             { return root().debug(); }
inline record info()              { return root().info(); }
inline record warning()           { return root().warning(); }
inline record error()             { return root().error(); }
inline record fatal()             { return root().fatal(); }
inline record message(level lvl)  { return root().message(lvl); }
inline bool enabled(level lvl)    { return root().enabled(lvl); }

level parseLevel(const std::string& name, level fallback = level::info);
const char* name(level lvl);                   // round-trips through parseLevel

/**
 * Process-wide configuration. Call it first thing in main().
 *
 * `propertiesFile` non-empty: the backend is configured from it and owns
 * levels, formats, appenders and per-channel levels; `minLevel` is then
 * ignored, so a carefully written file is not clobbered by a caller's default.
 * Empty: one console appender, records at `minLevel`.
 */
struct config {
    level minLevel = level::info;
    std::string propertiesFile;
};

void init(const config& cfg);
void initOnce(const config& cfg);   // applies only if nothing has configured yet
bool isConfigured();

}  // namespace log
}  // namespace toffy
```

Two names are deliberately *not* `level()`: the free getter and the
`config::level` member would both collide with the enum type name in their
scope. Hence `enabled()`/`effectiveLevel()`/`minLevel`.

### Call sites

```cpp
toffy::log::warning() << "frame " << n << " has no depth";

if (toffy::log::enabled(toffy::log::level::debug))
    toffy::log::debug() << frame.dumpForDebug();      // argument is expensive

// inside a Filter -- keeps the id()::func() prefix the macro gives today
LOG(debug) << "value=" << v;
```

`LOG(lvl)` stays in `filter_helpers.hpp`, redefined:

```cpp
#define LOG(lvl) ::toffy::log::detail::filterRecord(*this, ::toffy::log::level::lvl, __func__)
```

`filterRecord` is a `protected` member of `Filter`, not a free function in
`toffy::log::detail`: it starts a record on the filter's channel and writes the
`id()::func() : ` prefix. Keeping it a member avoids `logging.hpp` needing
`Filter` at all — the include only goes one way. Lowercase enumerators are what
keep all 100 existing sites compiling unchanged.

## Semantics that must survive the backend swap

These are the places where "it's just a logging library" hides behaviour:

1. **Arguments are always evaluated.** `debug() << describe(frame)` calls
   `describe()` even when debug is off — same as `BOOST_LOG_TRIVIAL`. `enabled()`
   is the way to not call it. Not a defect to fix here; a documented property,
   because `P2-8` had to remove three log calls off the per-frame path and the
   level switch is only usable in a loop if it is cheap.
2. **A disabled record allocates nothing.** The `ostringstream` is created on
   the first `<<` and only if the level is on. 455 of the 658 sites are `debug`.
3. **Unconfigured means stderr at `debug`.** That is Boost.Log trivial's
   behaviour today, and it is *not* log4cplus's — an unconfigured log4cplus root
   has no appender and silently drops everything. The log4cplus backend must
   install a default appender itself, or a program that never calls `init()`
   goes quiet. This is the single most likely silent regression in the whole
   item.
4. **One record is emitted atomically.** Per-record, not per-`<<`.
5. **`~Player` and `~Filter` never touch sinks or levels.** `P2-8` removed both
   violations; the API has no method that could reintroduce them.
6. **`options.loglvl` keeps its integer format and its precedence** (`options.loglvl`
   over `loglvl`, clamped to `trace..fatal`) — pinned by `test_logging`.

### Interim limitation, removed by the swap

While the backend is Boost.Log, a channel can be **more restrictive** than the
root but not more verbose: Boost's core filter is one global predicate. The
implementation sets that filter to the root level and does channel filtering in
toffy's layer, so the observable rule is
`emitted iff lvl >= channel.effectiveLevel() and lvl >= root.level()`. log4cplus
has per-logger levels natively, and the second clause goes away. Recorded here
so nobody discovers it as a surprise in `boxes`.

## Step C landed

One backend, not two. Keeping Boost.Log selectable was considered and rejected:
the semantics tests are the reason to believe the swap is safe, they pass
against log4cplus, and a second backend that nothing exercises is the thing this
programme deletes on sight.

The header lost one member in the process: `logger` used to carry its own
`ownLevel_`, which was correct for the Boost backend (which had no hierarchy and
so needed one invented) and wrong for log4cplus (which has one and owns the
levels). A handle is a name now, and `effectiveLevel()`, `setLevel()` and
`inheritLevel()` are the backend's. That is less code in both backends, not
just one.

Three things log4cplus does that the abstraction had been doing by hand, and one
it does not:

* **Inheritance, including through names that were never requested** —
  `Logger::getInstance("a.b.c")` creates `"a.b"` implicitly, and
  `getChainedLogLevel()` resolves the chain. The hand-written walk in the Boost
  backend existed only to agree with this, and it is gone.
* **`NOT_SET_LOG_LEVEL == -1` is exactly `inheritLevel()`** — the sentinel the
  first implementation invented for its own table is the library's own value.
* **`OFF_LOG_LEVEL` exists natively**, so `level::off` maps instead of being
  emulated by "above fatal".
* **It does not give you a default appender.** An unconfigured log4cplus root
  logger has no appender and discards every record without a word. Boost.Log's
  trivial logger creates a stderr sink on first use, so "the application
  configured nothing" produced output. This is the silent-regression case the
  semantics table called out, and it is now handled by installing a stderr
  `ConsoleAppender` on the root at `TRACE` on first use of the API.

### The Ubuntu package's exported CMake target is broken, so log4cplus is found by pkg-config

`find_package(log4cplus)` does not work on a stock Ubuntu install of
`liblog4cplus-dev`. Its `log4cplusTargets.cmake` exports

```
INTERFACE_LINK_LIBRARIES "/usr/lib/aarch64-linux-gnu/librt.a;/usr/lib/aarch64-linux-gnu/libnsl.so"
```

and `apt-cache depends liblog4cplus-dev` lists only `liblog4cplus-2.0.5t64` — no
`libnsl-dev`. So the imported target names `/usr/lib/.../libnsl.so`, a file that
`apt-get install liblog4cplus-dev` does not put there (this machine has
`libnsl.so.1` and `libnsl.so.2` and no `libnsl.so`), and every link dies with
`No rule to make target '/usr/lib/aarch64-linux-gnu/libnsl.so'`. Measured, not
read: that is the first thing that happened when step C was built.

So toffy finds it with `pkg_search_module(LOG4CPLUS REQUIRED log4cplus)` and
links `-llog4cplus`, and `toffyConfig.cmake` has no `find_dependency(log4cplus)`
because there is no imported target for a consumer to have to have found. Both
halves switch back together if the package is ever fixed; the imported target is
the better mechanism and this is a workaround, written as one.

This matters beyond toffy: it is the precondition question for the whole
log4cplus move, and it will bite on the CI runner and on any sysroot that
installs the dev package without `libnsl-dev` — which is the normal shape.

### One behaviour difference, chosen

`config::fileSink` (what `Player(level, true)` means) wrote `toffy_%N.log` with
**both** a 10 MiB and a midnight rotation configured. log4cplus does one or the
other per appender, so it is now `toffy.log` with size rotation at 10 MiB and
five backups — the closer of the two, and the one that bounds disk use, which is
what the setting was for. `propertiesFile` is how you get anything else.

## Step A landed

Steps A and B arrived together, because they could not be separated: deleting
`#include <boost/log/trivial.hpp>` from `filter.hpp` and `filter_helpers.hpp`
broke every `.cpp` that had been relying on the transitive include, so the
conversion of the 658 sites was not a follow-up to the header fix, it was the
price of it. Measured on the landed commit:

| | |
|---|---|
| `BOOST_LOG_TRIVIAL` call sites | **0** (was 658) — 677 `toffy::log::<level>()` and 103 `LOG(...)` |
| `boost/log` includes outside `src/logging.cpp` | **0** in shipped code; 5 in `tests/test_logging.cpp`, which installs a sink to observe output and is the one documented exception |
| installed headers including `boost/log` | **0** (was 5) |
| installed-header check | 88 headers × 2 = **176 checks, 0 broken** |
| `ctest` | **16/16** in Release/PCL-on/BTA-on, `WITHOUT_BTA=ON` and `WITHOUT_PCL=ON` |
| tree warnings | **25**, unchanged |
| `abi_surface.sh` | re-blessed from a **BTA-off** build (the baseline's stated configuration — blessing from the BTA-on build I had to hand added 26 `toffy::bta` symbols and would have made the next CI run red on a non-break, which is the trap its header comment describes) |
| consumer link surface | `boxes`: exactly one line changed, `Player(boost::log::v2_mt_posix::trivial::severity_level, bool)` → `Player(toffy::log::level, bool)`. `toffy-oatpp`: 3 `toffy::log::` additions and **no `libboost_log` in `NEEDED` at all** |

Three things the design did not predict, all found by building:

1. **`std::endl` needs its own overload.** `record`'s templated `operator<<`
   cannot deduce it — `std::endl` is an overloaded function template, not a
   function pointer the template can bind. `<< endl` appears in toffy's log
   lines, so a record without that overload is not a drop-in for the stream
   `BOOST_LOG_TRIVIAL` returned. Found as a compile error in 12 files.
2. **`LOG(warning) " text "` was legal, by string-literal concatenation.** The
   old macro ended with `<< "() : "`, so `filterbank.cpp:161`'s missing `<<`
   spliced two literals together and worked. One call site fixed.
3. **`Filter::dbg` was load-bearing in `boxes`.** Three uses in
   `colorblob.cpp` resolve to the member, not to a file-local `dbg` — the
   file-static debug flags in this tree are a separate, unrelated convention and
   10 filter files declare their own. Removing the cached flag is still the
   right call (it is what made "`dbg` disagrees with the level" representable,
   which was half of `P2-8`), but it is three consumer lines and it was not in
   the break surface above.

`BtaWrapper::loadFrame()` also went: a `#ifdef CM_DEBUG` block calling
`logging::core::get()->set_filter(severity >= debug)` **from inside a function
that runs once per frame**. In a debug build that re-pinned the process-wide
level to debug on every frame and nothing else in the application could turn it
back down — the `P2-8` defect, alive in the BTA module, found only because the
conversion made every `set_filter` call in the tree visible at once.

## Sequencing

| Step | What | Evidence |
|---|---|---|
| **A+B** | ✅ **landed.** `logging.hpp` + `logging.cpp` backed by **Boost.Log**. `LOG(lvl)` redefined. `Filter`'s and `Player`'s signatures switch to `toffy::log::level`. The 5 installed headers stop including `boost/log`. All 658 + 100 call sites converted, because the transitive include made that unavoidable. | `test_logging_abstraction` (levels, names, channel hierarchy, inheritance through gaps, `initOnce`, `enabled`); existing `test_logging` green and extended (round-trip through config, `warn` alias, `off` unreachable from config); 176/176 installed-header checks; both consumers rebuilt |
| ~~**B**~~ | merged into A above — see *Step A landed* | |
| **C** | ✅ **landed.** Backend is log4cplus; `log log_setup` out of `find_package(Boost)`; `logging.cpp` reimplemented against it and the Boost.Log backend deleted rather than kept selectable. | **the point of the item, measured**: `readelf -d libtoffy.so.2.0.0` has **no `libboost_log`** and does have `liblog4cplus-2.0.so.3`; `test_logging_abstraction` — which names no backend — passes **unchanged** against the new one, which is the first real evidence that the abstraction is an abstraction; both consumers rebuilt, both link surfaces unchanged; 16/16 `ctest` in Release, Debug, `WITHOUT_BTA=ON`, `WITHOUT_PCL=ON`; 25 tree warnings unchanged; `abi_surface.sh` re-blessed (BTA-off) with one change, `logger(std::string, int)` → `logger(std::string)` |
| **D** | `boxes` migrates its 378 sites, on its own schedule. | its own; toffy does not wait for it |

A was the dependency fix for consumers, and it paid for itself before C: with
its one `BOOST_LOG` site converted, `toffy-oatpp` dropped `log log_setup` from
its own `find_package(Boost)` and has no `libboost_log` in `NEEDED` any more.
`libtoffy.so` still links Boost.Log — that is C.

### Semantic corrections made during A

Two places where the interim backend would have disagreed with log4cplus, fixed
rather than documented, because "the swap changes which records appear" is not a
swap:

- **Inheritance walks past missing ancestors.** `"toffy.filters.rectify"` must
  follow a pinned `"toffy"` even when nothing ever asked for `"toffy.filters"`.
  log4cplus creates those implicitly; the first implementation stopped at the
  gap and fell back to root. Pinned by
  `Channel.InheritanceSkipsOverAncestorsThatWereNeverRequested`.
- **The Boost severity filter is left wide open** and toffy's layer decides,
  rather than mirroring the root level into it. That makes *"a channel can be
  more verbose than the root"* true here as it is true under log4cplus, so the
  interim limitation this section previously described does not exist.

Not fixed, and it should be decided at C: log4cplus treats logger names as
case-**insensitive** by default (`Logger::getLogger`'s `CaseInsensitive`), and
this table is case-sensitive. Toffy's names come from `Filter::type()`, which is
lowercase, so nothing observable depends on it today.

## Break surface, stated exactly

- `toffy::Player::Player(boost::log::v2_mt_posix::trivial::severity_level, bool)`
  → `toffy::Player::Player(toffy::log::level, bool)`. **One line in
  `ci/consumer-link-surface.txt` changes**, and the gate fails on *additions*,
  so the baseline is regenerated in the same commit with the reason recorded.
- `Filter::setGlobalLogLevel`, `Filter::logLvl()` change mangled names. Neither
  consumer calls them (measured: 0 sites).
- `ci/abi-surface.txt` regenerated: boost-typed symbols out, `toffy::log::*` in.
- SONAME stays 2 (decision 3).
