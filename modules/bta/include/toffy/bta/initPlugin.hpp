
#include <toffy/common/plugins.hpp>
#include "toffy/bta/bta.hpp"

#ifdef WITH_CONTROL
    #include <toffy_web/controllerFactory.hpp>
#endif

namespace toffy {
  namespace bta {
    void init(toffy::FilterFactory *ff);

#   ifdef WITH_CONTROL
    void initUI(control::ControllerFactory *cf);
#   endif
  }
}
