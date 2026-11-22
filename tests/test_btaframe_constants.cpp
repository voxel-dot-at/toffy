/*
 * toffy cleanup -- P2-9, the btaFrame.hpp slot-name constants.
 *
 * btaFrame.hpp used to *define* them. A namespace-scope `const` object has
 * internal linkage, so every translation unit that included that (installed)
 * header got its own std::string, its own dynamic initialiser, and its own
 * address. Values compared fine because std::string compares by content; the
 * cost was per-TU objects and a shared header that was not actually shared.
 */
#include <gtest/gtest.h>

#include <toffy/btaFrame.hpp>

namespace toffy {
// Defined in tests/test_btaframe_constants_addr.cpp, a different TU.
const std::string *btaDepthFromAnotherTu();
const std::string *btaMfFromAnotherTu();
}  // namespace toffy

TEST(BtaFrameConstantsP29, ValuesAreUnchanged)
{
    // The names are frame-slot keys that pipeline configs and filters agree on.
    // Moving the definitions must not change a single character of them.
    EXPECT_EQ("mf", toffy::btaMf);
    EXPECT_EQ("it", toffy::btaIt);
    EXPECT_EQ("fc", toffy::btaFc);
    EXPECT_EQ("depth", toffy::btaDepth);
    EXPECT_EQ("ampl", toffy::btaAmpl);
}

TEST(BtaFrameConstantsP29, OneInstanceAcrossTranslationUnits)
{
    // The fence. Before P2-9 these two pointers differed, because each TU had its
    // own copy of the object. Now there is one definition in libtoffy and both TUs
    // bind to it.
    EXPECT_EQ(&toffy::btaDepth, toffy::btaDepthFromAnotherTu())
        << "btaDepth exists once per translation unit: it is defined in a header";
    EXPECT_EQ(&toffy::btaMf, toffy::btaMfFromAnotherTu());
}

TEST(BtaFrameConstantsP29, BtaFrameStillReadsDepthThroughTheConstant)
{
    // getDepth()'s default argument is btaDepth. A default argument that names an
    // extern const is fine, but it is exactly the kind of thing that stops
    // compiling if someone "tidies" the declaration into a function, so pin it.
    toffy::BtaFrame bf;
    toffy::matPtr m = std::make_shared<cv::Mat>(4, 4, CV_32F);
    bf.addData(toffy::btaDepth, m);
    ASSERT_NE(nullptr, bf.getDepth());
    EXPECT_EQ(4, bf.getDepth()->rows);
}
