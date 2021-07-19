#include <bits/stdc++.h>
#include <gtest/gtest.h>
#include <vector>
#include "CommonFunctions.cpp"
#include "Obstacle.cpp"

using namespace std;

TEST(ObstacleSortTest, ObsSortertest) {
    vector<Obstacle> v1;
    v1.push_back(Obstacle());
    v1[0].setPosition({20,12,24});
    v1.push_back(Obstacle());
    v1[1].setPosition({3,1,2});
    v1.push_back(Obstacle());
    v1[2].setPosition({1,2,1});
    v1.push_back(Obstacle());
    v1[3].setPosition({5,10,15});
    sort(v1.begin(), v1.end());
    ASSERT_EQ(2, v1[0].getPosition().at(1));
    ASSERT_EQ(24, v1[3].getPosition().at(2));
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}