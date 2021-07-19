#ifndef SENSOR_CPP
#define SENSOR_CPP

#include "Obstacle.cpp"
#include "SensorType.hpp"
#include <vector>

using namespace std;

class Sensor {
    private:
    vector<double> fov_parameters; // { minimum distance, maximum distance, horizontal angle, vertical angle }
    vector<vector<double> > CHCTM; // Composite Homogenous Transformation Matrix.
    double dropout_probability;    // between [0, 1] Probability that a dropout will occur.
    double dropout_skew;           // between [0, 1] What kind of dropout is preferred? No data or retain previous data.
    vector<vector<Obstacle> > previous_data;    // Data that this sensor "detected" in the previous timestep.
    vector<double> noise_standard_deviations;   // [3 X 1] standard deviations along each axis (m).
};
#endif