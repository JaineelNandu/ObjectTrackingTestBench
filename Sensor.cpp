#ifndef SENSOR_CPP
#define SENSOR_CPP

#include "Obstacle.cpp"
#include "SensorType.hpp"
#include <vector>

using namespace std;

class Sensor
{
private:
    vector<double> fov_parameters; // { horizontal angle, vertical angle, minimum distance, maximum distance }
    // Composite Homogenous Transformation Matrix. Default is identity matrix.
    vector<vector<double>> CHCTM_fixed_to_mobile = {{1, 0, 0, 0},
                                                    {0, 1, 0, 0},
                                                    {0, 0, 1, 0},
                                                    {0, 0, 0, 1}};
    double dropout_probability = 0;           // between [0, 1] Probability that a dropout will occur.
    double dropout_skew = 0;                  // between [0, 1] What kind of dropout is preferred? No data or retain previous data.
    vector<vector<Obstacle>> previous_data;   // Data that this sensor "detected" in the previous timestep.
    vector<double> noise_standard_deviations; // [3 X 1] standard deviations along each axis (m).

public:
    Sensor(vector<double> fov, vector<double> st_dev = {0, 0, 0}, 
                                                        vector<vector<double> > chctm = {{1, 0, 0, 0},
                                                                                         {0, 1, 0, 0},
                                                                                         {0, 0, 1, 0},
                                                                                         {0, 0, 0, 1}},
                                                                                          double dp = 0, double sk = 0) {
        fov_parameters = fov;
        CHCTM_fixed_to_mobile = chctm;
        dropout_probability = dp;
        dropout_skew = sk;
        noise_standard_deviations = st_dev;
    }

    vector<vector<double> > getCHCTM (){
        return CHCTM_fixed_to_mobile;
    }

    vector<bool> areInFOV(vector<vector<double> > points) {
        vector<double> ones(points.size(), 1);
        points = matrixTranspose(points);
        points.push_back(ones);
        vector<vector<double> > pt_sensor_frame = matrixMultiply(CHCTM_fixed_to_mobile, points);
        vector<bool> inFOV;
        for (int pt = 0; points[0].size(); pt++) {
            double x = pt_sensor_frame[0][pt];
            double y = pt_sensor_frame[1][pt];
            double z = pt_sensor_frame[2][pt];
            double azim = roundThisTo<double>(180*atan2(y, x)/PI_6, 6);
            double elev = roundThisTo<double>(180*atan2(z, sqrt(pow(x,2)+pow(y,2))), 6);
            double dist = roundThisTo<double>(sqrt(pow(x,2) + pow(y,2) + pow(z,2)), 6);
            if (azim >= roundThisTo<double>(-fov_parameters[0]/2, 6) && azim <= roundThisTo<double>(fov_parameters[0]/2, 6)) {
                if (elev >= roundThisTo<double>(-fov_parameters[1]/2, 6) && elev <= roundThisTo<double>(fov_parameters[1]/2, 6)) {
                    if (dist >= roundThisTo<double>(fov_parameters[2], 6) && dist <= roundThisTo<double>(fov_parameters[3], 6)) {
                        inFOV.push_back(true);
                    }
                    else inFOV.push_back(false);
                }
                else inFOV.push_back(false);
            }
            else inFOV.push_back(false);
        }
        return inFOV;
    }
};
#endif