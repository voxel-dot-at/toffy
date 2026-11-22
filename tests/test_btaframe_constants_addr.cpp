/*
 * Second translation unit for the P2-9 btaFrame.cpp fence.
 *
 * The whole point of the change is that the bta* slot-name constants exist once
 * per process instead of once per including TU. That property is not observable
 * from a single file -- inside one TU the header's own copy and the real object
 * are the same thing either way -- so this file exists only to hand the test a
 * pointer taken from somewhere else.
 */
#include "toffy/btaFrame.hpp"

namespace toffy {
const std::string *btaDepthFromAnotherTu() { return &btaDepth; }
const std::string *btaMfFromAnotherTu() { return &btaMf; }
}  // namespace toffy
