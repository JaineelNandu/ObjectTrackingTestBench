#include <bits/stdc++.h>
#include <gtest/gtest.h>
#include <vector>
#include "CommonFunctions.cpp"
#include "Obstacle.cpp"
#include "ObstacleType.hpp"
#include "SensorType.hpp"

using namespace std;

TEST(GetterSetterTest, PositionCoordinatesTest)
{
    Obstacle o1;
    vector<double> pos = o1.getPosition();
    ASSERT_EQ(0, pos[0]);
    ASSERT_EQ(0, pos[1]);
    ASSERT_EQ(0, pos[2]);
    o1.setPosition({3, 1, 2});
    pos = o1.getPosition();
    ASSERT_EQ(3, pos[0]);
    ASSERT_EQ(1, pos[1]);
    ASSERT_EQ(2, pos[2]);
    o1.setPosition({5, 2, 5});
    pos = o1.getPosition();
    ASSERT_NE(3, pos[0]);
    ASSERT_NE(1, pos[1]);
    ASSERT_NE(2, pos[2]);
    ASSERT_EQ(5, pos[0]);
    ASSERT_EQ(2, pos[1]);
    ASSERT_EQ(5, pos[2]);
}

TEST(GetterSetterTest, VelocityEstimatesTest)
{
    Obstacle o1;
    vector<double> vel = o1.getVelocity();
    ASSERT_EQ(0, vel[0]);
    ASSERT_EQ(0, vel[1]);
    ASSERT_EQ(0, vel[2]);
    o1.setVelocity({3, 1, 2});
    vel = o1.getVelocity();
    ASSERT_EQ(3, vel[0]);
    ASSERT_EQ(1, vel[1]);
    ASSERT_EQ(2, vel[2]);
    o1.setVelocity({5, 2, 5});
    vel = o1.getVelocity();
    ASSERT_NE(3, vel[0]);
    ASSERT_NE(1, vel[1]);
    ASSERT_NE(2, vel[2]);
    ASSERT_EQ(5, vel[0]);
    ASSERT_EQ(2, vel[1]);
    ASSERT_EQ(5, vel[2]);
}

TEST(GetterSetterTest, AccelerationEstimatesTest)
{
    Obstacle o1;
    vector<double> acc = o1.getAcceleration();
    ASSERT_EQ(0, acc[0]);
    ASSERT_EQ(0, acc[1]);
    ASSERT_EQ(0, acc[2]);
    o1.setAcceleration({3, 1, 2});
    acc = o1.getAcceleration();
    ASSERT_EQ(3, acc[0]);
    ASSERT_EQ(1, acc[1]);
    ASSERT_EQ(2, acc[2]);
    o1.setAcceleration({5, 2, 5});
    acc = o1.getAcceleration();
    ASSERT_NE(3, acc[0]);
    ASSERT_NE(1, acc[1]);
    ASSERT_NE(2, acc[2]);
    ASSERT_EQ(5, acc[0]);
    ASSERT_EQ(2, acc[1]);
    ASSERT_EQ(5, acc[2]);
}

TEST(GetterSetterTest, TypeOfObstacleTest)
{
    Obstacle o1;
    ObstacleType tp = o1.getObstacleType();
    ASSERT_EQ(ObstacleType::unclassified, tp);
    o1.setObstacleType(ObstacleType::pedestrian);
    tp = o1.getObstacleType();
    ASSERT_EQ(ObstacleType::pedestrian, tp);
    o1.setObstacleType(ObstacleType::deer);
    tp = o1.getObstacleType();
    ASSERT_EQ(ObstacleType::deer, tp);
}

TEST(GetterSetterTest, TypeOfSensorTest)
{
    Obstacle o1;
    SensorType tp = o1.getSensor();
    ASSERT_EQ(SensorType::global, tp);
    o1.setSensorType(SensorType::Camera1);
    tp = o1.getSensor();
    ASSERT_EQ(SensorType::Camera1, tp);
    o1.setSensorType(SensorType::Lidar1);
    tp = o1.getSensor();
    ASSERT_EQ(SensorType::Lidar1, tp);
}

TEST(GetterSetterTest, TimeStampsTest)
{
    Obstacle o1;
    ASSERT_EQ(-1, o1.getTimeStamp());
    o1.setTimeStamp(0.003);
    ASSERT_EQ(0.003, o1.getTimeStamp());
    o1.setTimeStamp(2.5003);
    ASSERT_EQ(2.5003, o1.getTimeStamp());
}

TEST(GetterSetterTest, KFInstancesTest)
{
    Obstacle o1;
    vector<int> KFI = o1.getKFInstances();
    ASSERT_EQ(-1, KFI[0]);
    ASSERT_EQ(-1, KFI[1]);
    ASSERT_EQ(-1, KFI[2]);
    o1.setKFInstances({3, 1, 2});
    KFI = o1.getKFInstances();
    ASSERT_EQ(3, KFI[0]);
    ASSERT_EQ(1, KFI[1]);
    ASSERT_EQ(2, KFI[2]);
    o1.setKFInstances({5, 2, 5});
    KFI = o1.getKFInstances();
    ASSERT_NE(3, KFI[0]);
    ASSERT_NE(1, KFI[1]);
    ASSERT_NE(2, KFI[2]);
    ASSERT_EQ(5, KFI[0]);
    ASSERT_EQ(2, KFI[1]);
    ASSERT_EQ(5, KFI[2]);
}

TEST(GetterSetterTest, ObstacleIDTest)
{
    Obstacle o1;
    ASSERT_EQ(-1, o1.getID());
    o1.setObstacleID(3);
    ASSERT_EQ(3, o1.getID());
    o1.setObstacleID(5);
    ASSERT_EQ(5, o1.getID());
}

TEST(GetterSetterTest, OutFOVCounterTest)
{
    Obstacle o1;
    ASSERT_EQ(20, o1.getOutFOVCounter());
    o1.decrementFOVCounter();
    ASSERT_EQ(19, o1.getOutFOVCounter());
    o1.decrementFOVCounter();
    ASSERT_EQ(18, o1.getOutFOVCounter());
    o1.decrementFOVCounter();
    ASSERT_EQ(17, o1.getOutFOVCounter());
    o1.resetFOVCounter();
    ASSERT_EQ(20, o1.getOutFOVCounter());
}

TEST(ObstacleSortTest, ObsSortertest)
{
    vector<Obstacle> v1;
    v1.push_back(Obstacle());
    v1[0].setPosition({20, 12, 24});
    v1.push_back(Obstacle());
    v1[1].setPosition({3, 1, 2});
    v1.push_back(Obstacle());
    v1[2].setPosition({1, 2, 1});
    v1.push_back(Obstacle());
    v1[3].setPosition({5, 10, 15});
    sort(v1.begin(), v1.end());
    ASSERT_EQ(2, v1[0].getPosition().at(1));
    ASSERT_EQ(24, v1[3].getPosition().at(2));
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}