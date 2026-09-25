# F — Performance

These are not micro-optimisations; several are on the per-frame or per-config-read path.

**Config helpers copy the whole property tree on every call.**
`filter_helpers.hpp:38,54,66` declare their parameter as `boost::property_tree::ptree`
**by value**:

```cpp
template<typename T> bool pt_optional_get(const boost::property_tree::ptree pt, ...
```

Every optional config read deep-copies the tree. Changing all three to `const ptree&` is a
one-line, no-risk fix.

**`Frame::getData()` looks a key up twice.** `frame.cpp:40-56` calls `data.find(key)` and
then `data.at(key)`. A single `find()` and reuse of the iterator is enough.

**Every `opt*` accessor costs three lookups.** `hasKey()` does one `find`, then the getter
calls `getData()` which does `find` + `at`. See `frame.hpp:359-390`.

**`removeData()` searches three times.** `frame.cpp:72-79` does `find`, then `find` again
inside the `if`, then `erase(iterator)` which searches once more.

**`Frame::info()` does two extra lookups per entry.** `frame.cpp:84-95` iterates `data`
but then calls `getDataType(key)` and `getDescription(key)`, each a fresh map lookup,
instead of walking `meta`/`desc` alongside.

**~~`FilterBank::filter()` reconfigures logging three times per filter, per frame.~~ FIXED
(`P2-8`).** It called the bank's own level before the loop, each child's inside the loop, and
the bank's again after every child — three `set_filter()` calls on the shared logging core per
filter per frame, each constructing a new filter expression and taking the core's lock. All
three are gone, along with the one in `FilterBank::loadGlobals()` (see D).

**Wall-clock timing uses local time.** `filterbank.cpp:54,66` use
`microsec_clock::local_time()`, which jumps on DST and NTP adjustments. Use
`steady_clock` (or `universal_time()` if a calendar stamp is genuinely wanted).

**`createFilter()` is a ~120-line string chain.** `filterfactory.cpp:148-265` compares the
requested type against ~30 literals with `if/else if`, on every instantiation — while the
`creators` map that exists precisely to replace this sits unused for built-ins. See P2.
