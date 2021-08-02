#ifndef OBSTACLE_CPP
#define OBSTACLE_CPP

#include "ObstacleType.hpp"
#include "SensorType.hpp"
#include "CommonFunctions.cpp"
#include <vector>
#include <string>
#include <iostream>

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

    bool operator==(const Obstacle &other) const
    {
        bool result;
        result = areEqualVectors(position_coordinates, other.position_coordinates);
        result = result && areEqualVectors(velocity_estimates, other.velocity_estimates);
        result = result && areEqualVectors(acceleration_estimates, other.acceleration_estimates);
        result = result && (type_of_obstacle == other.type_of_obstacle);
        result = result && (sensor == other.sensor);
        result = result && (time_stamp == other.time_stamp);
        result = result && (obstacle_ID == other.obstacle_ID);
        result = result && areEqualVectors(KF_instances, other.KF_instances);
        result = result && (out_FOV_count == other.out_FOV_count);
        return result;
    }

    bool operator!=(const Obstacle &other) const
    {
        bool result;
        result = !areEqualVectors(position_coordinates, other.position_coordinates);
        result = result || !areEqualVectors(velocity_estimates, other.velocity_estimates);
        result = result || !areEqualVectors(acceleration_estimates, other.acceleration_estimates);
        result = result || !(type_of_obstacle == other.type_of_obstacle);
        result = result || !(sensor == other.sensor);
        result = result || !(time_stamp == other.time_stamp);
        result = result || !(obstacle_ID == other.obstacle_ID);
        result = result || !areEqualVectors(KF_instances, other.KF_instances);
        result = result || !(out_FOV_count == other.out_FOV_count);
        return result;
    }

    void printObstacle() {
        cout << "ID: " << obstacle_ID << "\tTime: " << time_stamp <<"\tposition: ";
        printVector(position_coordinates);
        cout <<"\tVelocities: ";
        printVector(velocity_estimates);
        cout <<"\tAccelerations: ";
        printVector(acceleration_estimates);
        cout << "\tKF Instances: ";
        printVector(KF_instances);
        cout <<"\tOut of FOV: " << out_FOV_count;
        cout << "\n";
    }

    void predict(const double t) {
        for (int axis = 0; axis < 3; axis++) {
            position_coordinates[axis] = roundThisTo(position_coordinates[axis] + (t-time_stamp)*velocity_estimates[axis], 6); //
        }
        time_stamp = t;
    }

};
#endif