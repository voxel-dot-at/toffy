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
#pragma once

// Internal POSIX filesystem helpers. NOT public API.
//
// toffy's shipped code pulled in `<boost/filesystem.hpp>` from nine places. Two
// of them used it (`capturerFilter.cpp`, `bta.cpp`); seven included it and used
// nothing, including one *installed* header (`toffy/base/distAmpl.hpp`). This
// replaces the two so that `libtoffy` stops linking Boost.Filesystem and
// `filesystem` can come out of `find_package(Boost)` -- the same dependency
// surface `P2-17` removed for Boost.Log.
//
// Why this lives under `libraries/internal/` rather than `libraries/include/`:
// `libraries/CMakeLists.txt` installs `include/` wholesale, so anything placed
// there becomes shipped API. This does not. It is also why the include is
// `<toffy_internal/...>` and not `<toffy/...>` -- the path says whether a
// consumer is allowed to rely on it.
//
// Deliberate properties:
//
//   * **No exceptions.** `boost::filesystem` threw `filesystem_error`, and both
//     call sites caught it for the sole purpose of logging a warning and
//     continuing. An `errno` out-parameter says the same thing without an
//     exception type in the signature. (One exception is kept where throwing was
//     load-bearing -- see `toInt()`.)
//
//   * **POSIX only**, consistent with toffy having no Windows build (`P2-2`).
//
//   * **`createDirectories()` does not resolve symlinks or require the path to
//     exist**, matching `boost::filesystem::create_directories(
//     boost::filesystem::absolute(p))`, which is exactly what both call sites
//     did. A `realpath()`-based version would have been shorter and would have
//     failed on the not-yet-existing directories the call exists to create.

#include <dirent.h>
#include <limits.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace toffy {
namespace fsx {

/// Last path component, or the whole string if it contains no '/'.
inline std::string basenameOf(const std::string &p)
{
    const std::string::size_type slash = p.find_last_of('/');
    if (std::string::npos == slash) return p;
    return p.substr(slash + 1);
}

/// Everything before the last '/', matching `boost::filesystem::parent_path`:
/// "a/b" -> "a", "/b" -> "/", "b" -> "".
inline std::string parentPath(const std::string &p)
{
    const std::string::size_type slash = p.find_last_of('/');
    if (std::string::npos == slash) return std::string();
    if (0 == slash) return std::string("/");
    return p.substr(0, slash);
}

/// Extension of the last component *including* the dot, or "" -- matching
/// `boost::filesystem::path::extension()`. A leading dot is not an extension
/// (".bltstream" as a whole filename has none), and neither is a trailing one.
inline std::string extension(const std::string &p)
{
    const std::string name = basenameOf(p);
    const std::string::size_type dot = name.find_last_of('.');
    if (std::string::npos == dot || 0 == dot) return std::string();
    return name.substr(dot);
}

/// Last component without its extension, matching `path::stem()`.
inline std::string stem(const std::string &p)
{
    const std::string name = basenameOf(p);
    const std::string::size_type dot = name.find_last_of('.');
    if (std::string::npos == dot || 0 == dot) return name;
    return name.substr(0, dot);
}

/// `base /= part`, without doubling a separator.
inline std::string append(const std::string &base, const std::string &part)
{
    if (base.empty()) return part;
    if (part.empty()) return base;
    if (base.back() == '/') return base + part;
    return base + "/" + part;
}

inline bool exists(const std::string &p)
{
    struct stat st;
    return 0 == stat(p.c_str(), &st);
}

inline bool isDirectory(const std::string &p)
{
    struct stat st;
    return (0 == stat(p.c_str(), &st)) && S_ISDIR(st.st_mode);
}

inline bool isRegularFile(const std::string &p)
{
    struct stat st;
    return (0 == stat(p.c_str(), &st)) && S_ISREG(st.st_mode);
}

/// Make every component of `p`, mkdir -p style. Relative paths are resolved
/// against the working directory by prefixing it textually -- deliberately not
/// `realpath()`, which would require `p` to already exist.
///
/// Returns true if `p` is a directory afterwards (including "it already was"),
/// false with `*err` set on failure.
inline bool createDirectories(const std::string &p, int *err = nullptr)
{
    if (p.empty()) {
        if (err) *err = EINVAL;
        return false;
    }

    std::string full = p;
    if (full.front() != '/') {
        char cwd[PATH_MAX];
        if (getcwd(cwd, sizeof(cwd))) {
            full = append(cwd, full);
        }
    }

    // Walk the components, creating each. EEXIST on an intermediate is fine --
    // it may already be a directory -- but a non-directory of that name is not.
    std::string acc;
    std::string::size_type i = 0;
    while (i <= full.size()) {
        const std::string::size_type slash = full.find('/', i);
        acc = (std::string::npos == slash) ? full : full.substr(0, slash);
        if (!acc.empty() && acc != "/") {
            if (0 != mkdir(acc.c_str(), 0755)) {
                if (EEXIST != errno) {
                    if (err) *err = errno;
                    return isDirectory(full);
                }
                if (!isDirectory(acc)) {
                    if (err) *err = ENOTDIR;
                    return false;
                }
            }
        }
        if (std::string::npos == slash) break;
        i = slash + 1;
    }

    if (err) *err = 0;
    return isDirectory(full);
}

/// Regular files in `dir` whose extension is exactly `ext` (dot included), in
/// directory order. Replaces the `boost::filesystem::directory_iterator` loop
/// in `CapturerFilter::loadPath()`.
inline std::vector<std::string> listRegularWithExtension(const std::string &dir,
                                                         const std::string &ext)
{
    std::vector<std::string> out;
    DIR *d = opendir(dir.c_str());
    if (!d) return out;

    struct dirent *ent = nullptr;
    while ((ent = readdir(d)) != nullptr) {
        const std::string name(ent->d_name);
        if ("." == name || ".." == name) continue;
        const std::string full = append(dir, name);
        if (extension(full) == ext && isRegularFile(full)) out.push_back(full);
    }
    closedir(d);
    return out;
}

/// `boost::lexical_cast<int>` equivalent, kept throwing on purpose:
/// `CapturerFilter::loadPath()` has always propagated a bad numeric stem rather
/// than skipping the file, and silently reading it as 0 would reorder the
/// begin/end file selection instead of reporting it.
inline int toInt(const std::string &s) { return std::stoi(s); }

inline const char *strError(int err)
{
    return (0 == err) ? "success" : std::strerror(err);
}

}  // namespace fsx
}  // namespace toffy
