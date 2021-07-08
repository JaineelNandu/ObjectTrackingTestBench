#include "sqrt.cpp"
#include <gtest/gtest.h>

TEST(SquareRootTest, PositiveNumbers) {
    ASSERT_EQ(6, squareRoot(36.0));
}

TEST(SquareRootTestNeg, NegativeNumbers) {
    ASSERT_NE(-1.0, squareRoot(-7.0));
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}