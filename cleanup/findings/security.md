# H — Security & robustness

From the second-pass audit (`S`/`N` ids in `plan/second-pass.md`).

### N4 — `system()` on a configuration string

`modules/filters/src/detection/objectTrack.cpp:196-204`:

```cpp
sb << updateScript << " " << cnt << " " << numObjects << "&";
system(sb.str().c_str());
```

`updateScript` comes straight from XML (`pt.get<string>("options.script", ...)` at `:82`), so
any shell metacharacter in a config file is executed with the process's privileges. The
trailing `&` is inside the string, so the call also does not wait. GCC already points at it —
`-Wunused-result: ignoring return value of ‘int system(...)’` — and the warning is ignored.

**Suggestion S5 — replace with `posix_spawn`/`fork`+`execv` on an argv array**, or delete the
feature. `objectTrack` is in `modules/filters`, outside the programme's stated scope, but this
one is a one-function change and it is the only `system()` call in the tree.

### N5 — config string used as a `printf` format string

`modules/filters/src/capture/csv_source.cpp:162,183`:

```cpp
snprintf(path, sizeof(path), _depthPattern.c_str(), sequence);
snprintf(path, sizeof(path), _amplPattern.c_str(), sequence);
```

`_depthPattern`/`_amplPattern` are read from XML (`:53-54,98-99`). A pattern that is not a
valid format string is undefined behaviour; `-Wformat` cannot help because the format is not a
literal. Two lines below each, `fscanf(f, "%g;", &val)` and `fscanf(f, "%d;", &val)` (`:174`,
`:195`) discard their return value, so a short or malformed CSV writes uninitialised `val` into
the image and the filter reports success. Both sites are already flagged by `-Wunused-result`.

**Suggestion S6 — validate the pattern once at config time, and check the `fscanf` return.**
Small, local, and it turns a silently-corrupt frame into a logged error.
