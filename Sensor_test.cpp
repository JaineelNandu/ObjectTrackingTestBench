#include <bits/stdc++.h>
#include <gtest/gtest.h>
#include "CommonFunctions.cpp"
#include <vector>
#include "Sensor.cpp"

TEST(SensorFOVTests, SensorFOVTest1) {
    Sensor s1({270, 40, 3, 150});
    vector<bool> truth = {false, true, false, false, true, true};
    vector<vector<double> > points = {{20.5, 11, 12}, {4, 5, 1}, {4, 5, 10}, {1, 2, 36}, {143, 2, 1}, {-39, -60, 1}};
    vector<bool> inFOV = s1.areInFOV(points);
    ASSERT_TRUE(areEqualVectors(inFOV, truth));
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}