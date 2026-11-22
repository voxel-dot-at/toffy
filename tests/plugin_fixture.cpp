/*
 * A real plug-in, built as a MODULE, so that tests/test_plugin_loading.cpp can
 * dlopen() it and prove the loader actually loads.
 *
 * Until now the plug-in path was only ever exercised with a library that does not
 * exist, which tests the failure branch and nothing else -- including the part
 * that matters, that init() runs and what it registers becomes usable.
 */
#include <toffy/common/plugins.hpp>
#include <toffy/filter.hpp>
#include <toffy/filterfactory.hpp>

namespace {
/// Minimal filter the plug-in registers, so the test can observe that init() ran
/// and that what it registered is reachable through the factory afterwards.
class PluginMarkerFilter : public toffy::Filter
{
   public:
    PluginMarkerFilter() : toffy::Filter("p210plugin") {}
    bool filter(const toffy::Frame &, toffy::Frame &) override
    {
        return true;
    }
};
}  // namespace

extern "C" void init(toffy::FilterFactory *ff)
{
    ff->registerCreator("test.p210plugin",
                        []() -> toffy::Filter * { return new PluginMarkerFilter(); });
}
