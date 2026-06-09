#include <gtest/gtest.h>
#include <glm/glm.hpp>
#include "tonton_tensors.hpp"
#include "tonton.h"

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

// A long-thin body along +x: large extent in x, small in y/z.
// SecondMoment: x-eigenvalue is LARGEST (most spread) → Small = thin axis (y or z).
// Inertia: rotation about the long axis is SMALLEST → Small = long axis (x).
TEST(Geometry, EigenPolarityReversal) {
    // second moment of a rod along x: Cxx >> Cyy == Czz
    SecondMomentTensor c{ glm::dmat3(0.0) };
    c.C[0][0] = 100.0; c.C[1][1] = 1.0; c.C[2][2] = 1.0;

    auto [q_c, v_c] = TonTon::EigenDecomposition(c);
    // ascending: smallest is a thin axis (value 1), largest is the long axis (100)
    EXPECT_NEAR(v_c[0], 1.0, 1e-6);
    EXPECT_NEAR(v_c[2], 100.0, 1e-6);

    InertiaTensor i = ToInertia(c);
    auto [q_i, v_i] = TonTon::EigenDecomposition(i);
    // I = tr-C: Ixx = (1+1)=2 (about long axis, smallest), Iyy=Izz=101
    EXPECT_NEAR(v_i[0], 2.0, 1e-6);
    EXPECT_NEAR(v_i[2], 101.0, 1e-6);
}
