/*
 * toffy cleanup -- P2-9, module layering.
 *
 * `Controller::loopFilters()` counts the filters of one particular type in the
 * bank to decide whether it has to pump the OpenCV HighGUI event loop while the
 * pipeline runs. It used to obtain that type name by including
 * <toffy/viewers/imageview.hpp>, which made modules/core depend on and link the
 * viewers module for one identifier.
 *
 * Core owns the string now (`guiViewerTypeName` in controller.hpp). Duplicating
 * a constant buys layering at the price of drift, so this file is what keeps the
 * price from being paid silently: if viewers ever renames its filter, or core
 * mistypes the literal, the two disagree and this fails.
 *
 * Copyright 2026 Simon Vogl <svogl@voxel.at>
 * Licensed under the Apache License, Version 2.0.
 */

#include <string>

#include <gtest/gtest.h>

#include <toffy/controller.hpp>
#include <toffy/filterfactory.hpp>
#include <toffy/viewers/imageview.hpp>

TEST(LayeringP29, CoreOwnsTheGuiViewerTypeName)
{
    // The point of the constant is that core does not include viewers to get it.
    // This test file is allowed to include both -- tests sit above every module --
    // and that is exactly what makes the comparison possible.
    EXPECT_STREQ("imageview", toffy::guiViewerTypeName);
}

TEST(LayeringP29, CoreConstantStillMatchesImageViewIdName)
{
    // The drift fence. ImageView::id_name is the real name of the filter that
    // opens the window; guiViewerTypeName is core's copy of it. If this fails,
    // Controller has stopped pumping the GUI for a pipeline that still needs it:
    // the windows freeze and the loop looks wedged, in a way that reads as an
    // OpenCV problem rather than as a renamed string.
    EXPECT_EQ(toffy::ImageView::id_name, std::string(toffy::guiViewerTypeName))
        << "core's guiViewerTypeName has drifted from ImageView::id_name; "
           "Controller::loopFilters() would no longer pump the HighGUI loop";
}

TEST(LayeringP29, ImageViewIsStillTheFilterCoreIsCounting)
{
    // Guards the other direction: the name must still resolve to a filter that
    // core can actually find. If ImageView were deleted or unregistered, the
    // constant would be a dangling reference to a type that no longer exists and
    // countFiltersByType() would quietly return 0 forever.
    //
    // getInstance() first, deliberately: the built-in creators are registered by
    // the first call to it, so hasCreator() before that answers false for every
    // name -- the same trap tests/test_filter_registry.cpp documents in
    // warmRegistry(). Asking without warming would make this test pass by being
    // vacuous in the opposite direction, which is the failure mode the P2-4 fence
    // was rewritten to refuse.
    toffy::FilterFactory::getInstance();
    EXPECT_TRUE(toffy::FilterFactory::hasCreator(toffy::guiViewerTypeName))
        << "guiViewerTypeName names a filter type that is not registered";
}
