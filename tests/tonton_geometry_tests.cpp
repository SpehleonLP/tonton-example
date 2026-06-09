#include <gtest/gtest.h>
#include <glm/glm.hpp>
#include "tonton_tensors.hpp"

using namespace TonTon;

// I = tr(C)·𝟙 − C for a known diagonal second moment.
TEST(Geometry, ToInertiaDiagonal) {
    SecondMomentTensor c{ glm::dmat3(0.0) };
    c.C[0][0] = 1.0; c.C[1][1] = 2.0; c.C[2][2] = 4.0;   // tr = 7
    InertiaTensor got = ToInertia(c);
    EXPECT_DOUBLE_EQ(got.I[0][0], 7.0 - 1.0); // 6
    EXPECT_DOUBLE_EQ(got.I[1][1], 7.0 - 2.0); // 5
    EXPECT_DOUBLE_EQ(got.I[2][2], 7.0 - 4.0); // 3
    EXPECT_DOUBLE_EQ(got.I[0][1], 0.0);
}

// Off-diagonal products negate: I_xy = -C_xy.
TEST(Geometry, ToInertiaOffDiagonal) {
    SecondMomentTensor c{ glm::dmat3(0.0) };
    c.C[0][0] = 1.0; c.C[1][1] = 1.0; c.C[2][2] = 1.0;
    c.C[0][1] = c.C[1][0] = 0.5;
    InertiaTensor got = ToInertia(c);
    EXPECT_DOUBLE_EQ(got.I[0][1], -0.5);
    EXPECT_DOUBLE_EQ(got.I[1][0], -0.5);
    EXPECT_DOUBLE_EQ(got.I[0][0], 3.0 - 1.0); // tr=3
}
