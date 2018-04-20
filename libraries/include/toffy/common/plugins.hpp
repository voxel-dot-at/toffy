#pragma once

#include "toffy/toffy_export.h"
#include "toffy/filterfactory.hpp"

namespace toffy {
namespace commons {
namespace plugins {
extern "C" {
void TOFFY_EXPORT init(FilterFactory *ff);
}
typedef void (*init_t)(FilterFactory *);

}}}
