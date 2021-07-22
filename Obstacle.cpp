#ifndef OBSTACLE_CPP
#define OBSTACLE_CPP

#include "ObstacleType.hpp"
#include "SensorType.hpp"
#include "CommonFunctions.cpp"
#include <vector>

using namespace std;

class Obstacle
{
private:
    vector<double> position_coordinates = {0.0, 0.0, 0.0};
    vector<double> velocity_estimates = {0.0, 0.0, 0.0};
    vector<double> acceleration_estimates = {0.0, 0.0, 0.0};
    ObstacleType type_of_obstacle = ObstacleType::unclassified;
    SensorType sensor = SensorType::global;
    double time_stamp = -1;                  // Unassigned
    vector<int> KF_instances = {-1, -1, -1}; // Unassigned
    int obstacle_ID = -1;                    // Unassigned
    int out_FOV_count = 20;

public:
    Obstacle()
    {
    }

    vector<double> getPosition()
    {
        return position_coordinates;
    }

    vector<double> getVelocity()
    {
        return velocity_estimates;
    }

    vector<double> getAcceleration()
    {
        return acceleration_estimates;
    }

    ObstacleType getObstacleType()
    {
        return type_of_obstacle;
    }

    SensorType getSensor()
    {
        return sensor;
    }

    double getTimeStamp()
    {
        return time_stamp;
    }

    vector<int> getKFInstances()
    {
        return KF_instances;
    }

    int getID()
    {
        return obstacle_ID;
    }

    int getOutFOVCounter()
    {
        return out_FOV_count;
    }

    void setPosition(vector<double> new_position)
    {
        position_coordinates = new_position;
    }

    void setVelocity(vector<double> new_velocity)
    {
        velocity_estimates = new_velocity;
    }

    void setAcceleration(vector<double> new_acceleration)
    {
        acceleration_estimates = new_acceleration;
    }

    void setObstacleType(ObstacleType new_type)
    {
        type_of_obstacle = new_type;
    }

    void setSensorType(SensorType new_sensor)
    {
        sensor = new_sensor;
    }

    void setTimeStamp(double new_time)
    {
        time_stamp = new_time;
    }

    void setObstacleID(int new_ID)
    {
        obstacle_ID = new_ID;
    }

    void setKFInstances(vector<int> instances)
    {
        KF_instances = instances;
    }

    void decrementFOVCounter()
    {
        out_FOV_count--;
    }

    void resetFOVCounter()
    {
        out_FOV_count = 20;
    }

    bool operator<(const Obstacle &other) const
    {
        return vectorMagnitude(position_coordinates) < vectorMagnitude(other.position_coordinates);
    }
};
#endif