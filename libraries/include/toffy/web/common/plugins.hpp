#pragma once

#include <toffy/toffy_export.h>

#include <toffy/web/controllerFactory.hpp>

namespace toffy {
namespace commons {
namespace plugins {
extern "C" {
void TOFFY_EXPORT initUI(control::ControllerFactory *cf);
}
typedef void (*initUI_t)(control::ControllerFactory *);

}}}
