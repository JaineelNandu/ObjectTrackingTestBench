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

TEST(SensorNoiseGenTest, SensorNoiseGenTest) {
    Sensor s1({270, 40, 3, 150}, {0.01, 0.05, 0.5});
    vector<vector<double> > noise = s1.getNoise(0);
    ASSERT_EQ(0, noise.size());
    noise = s1.getNoise(10);
    ASSERT_EQ(10, noise.size());
}

TEST(SensorFOVTests, SensorFOVNoisyPassedTest) {
    Sensor s1({270, 40, 3, 150}, {0.01, 0.05, 0.5});
    vector<int> truth_active = {2, 5, 6};
    vector<int> list_active = {1, 2, 3, 4, 5, 6};
    vector<vector<double> > truth_passed = {{4, 5, 1}, {143, 2, 1}, {-39, -60, 1}};
    vector<vector<double> > points = {{20.5, 11, 12}, {4, 5, 1}, {4, 5, 10}, {1, 2, 36}, {143, 2, 1}, {-39, -60, 1}};
    s1.passThrough(list_active, points);
    vector<int> passed_active = s1.getActivePassed();
    vector<vector<double> > passed_points = s1.getPassed();
    ASSERT_TRUE(areEqualVectors(passed_active, truth_active));
    EXPECT_TRUE(areEqualVectors(passed_points, truth_passed, 1.5)); // Tolerance is 3 std of maximum std i.e. 0.5
}

TEST(ObstacleListTest, GetPassedObstacleList) {
    Sensor s1({270, 40, 3, 150}, {0.01, 0.05, 0.5}, SensorType::Lidar2);
    vector<int> list_active = {1, 2, 3, 4, 5, 6};
    vector<vector<double> > points = {{20.5, 11, 12}, {4, 5, 1}, {4, 5, 10}, {1, 2, 36}, {143, 2, 1}, {-39, -60, 1}};
    s1.passThrough(list_active, points);
    vector<int> passed_active = s1.getActivePassed();
    vector<vector<double> > passed_points = s1.getPassed();
    vector<Obstacle> passed_obstacles = s1.pointToObstacle(0.055);
    for(int i = 0; i < passed_active.size(); i++) {
        Obstacle o1;
        o1.setPosition(passed_points[i]);
        o1.setSensorType(SensorType::Lidar2);
        o1.setTimeStamp(0.055);
        ASSERT_EQ(o1, passed_obstacles[i]);
    }
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}