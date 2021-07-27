#include <bits/stdc++.h>
#include <gtest/gtest.h>
#include "CommonFunctions.cpp"
#include <vector>
#include "Sensor.cpp"
#include "KFHandler.cpp"

TEST(AssignmentTests, AssignmenTests) {
    KFHandler k1(3);
    ASSERT_EQ(3, k1.getNumObstacleAvailable());
    Sensor s1({270, 40, 3, 150}, {0.01, 0.05, 0.5});
    vector<int> truth_active = {2, 5, 6};
    vector<int> list_active = {1, 2, 3, 4, 5, 6};
    vector<vector<double> > truth_passed = {{4, 5, 1}, {143, 2, 1}, {-39, -60, 1}};
    vector<vector<double> > points = {{20.5, 11, 12}, {4, 5, 1}, {4, 5, 10}, {1, 2, 36}, {143, 2, 1}, {-39, -60, 1}};
    s1.passThrough(list_active, points);
    vector<int> passed_active = s1.getActivePassed();
    vector<vector<double> > passed_points = s1.getPassed();
    vector<Obstacle> passed_obstacles = s1.pointToObstacle(0.055);
    k1.assignKF(passed_obstacles, truth_passed);
    ASSERT_EQ(0, k1.getNumObstacleAvailable());

    list_active = {1, 2, 3, 4, 5, 7, 6};
    truth_passed = {{4, 5, 1}, {143, 2, 1}, {-1, -20, 4}, {-39, -60, 1}};
    points = {{20.5, 11, 12}, {4, 5, 1}, {4, 5, 10}, {1, 2, 36}, {143, 2, 1}, {-1, -20, 4}, {-39, -60, 1}};
    s1.passThrough(list_active, points);
    passed_obstacles = s1.pointToObstacle(0.065);
    passed_obstacles[0].setKFInstances({0, 1, 2});
    passed_obstacles[1].setKFInstances({3, 4, 5});
    passed_obstacles[3].setKFInstances({6, 7, 8});
    k1.assignKF(passed_obstacles, truth_passed);
    ASSERT_EQ(0, k1.getNumObstacleAvailable());

    list_active = {1, 3, 4, 5, 7, 6};
    truth_passed = { {143, 2, 1}, {-1, -20, 4}, {-39, -60, 1}};
    points = {{20.5, 11, 12}, {4, 5, 10}, {1, 2, 36}, {143, 2, 1}, {-1, -20, 4}, {-39, -60, 1}};
    s1.passThrough(list_active, points);
    passed_obstacles = s1.pointToObstacle(0.065);
    passed_obstacles[0].setKFInstances({3, 4, 5});
    passed_obstacles[1].setKFInstances({6, 7, 8});
    k1.assignKF(passed_obstacles, truth_passed);
    ASSERT_EQ(0, k1.getNumObstacleAvailable());

    list_active = {1, 3, 4, 5};
    truth_passed = { {143, 2, 1}};
    points = {{20.5, 11, 12}, {4, 5, 10}, {1, 2, 36}, {143, 2, 1}};
    s1.passThrough(list_active, points);
    passed_obstacles = s1.pointToObstacle(0.065);
    passed_obstacles[0].setKFInstances({3, 4, 5});
    k1.assignKF(passed_obstacles, truth_passed);
    ASSERT_EQ(2, k1.getNumObstacleAvailable());

    list_active = {1, 3, 4, 5, 7, 6};
    truth_passed = { {143, 2, 1}, {-1, -20, 4}, {-39, -60, 1}};
    points = {{20.5, 11, 12}, {4, 5, 10}, {1, 2, 36}, {143, 2, 1}, {-1, -20, 4}, {-39, -60, 1}};
    s1.passThrough(list_active, points);
    passed_obstacles = s1.pointToObstacle(0.065);
    passed_obstacles[0].setKFInstances({3, 4, 5});
    k1.assignKF(passed_obstacles, truth_passed);
    ASSERT_EQ(0, k1.getNumObstacleAvailable());
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}