#ifndef SENSOR_CPP
#define SENSOR_CPP

#include "Obstacle.cpp"
#include "SensorType.hpp"
#include <vector>
#include <random>

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
    vector<vector<double> > passed_points;
    vector<vector<double> > passed_points_truth;
    vector<int> passed_obstacles;
    SensorType sensor_type;
    int update_rate;

public:
    Sensor(vector<double> fov, vector<double> st_dev = {0, 0, 0}, SensorType st = SensorType::global, int rate = 10,
                                                        vector<vector<double> > chctm = {{1, 0, 0, 0},
                                                                                         {0, 1, 0, 0},
                                                                                         {0, 0, 1, 0},
                                                                                         {0, 0, 0, 1}},
                                                                                          double dp = 0, double sk = 0) {
        fov_parameters = fov;
        update_rate = rate;
        CHCTM_fixed_to_mobile = chctm;
        dropout_probability = dp;
        dropout_skew = sk;
        noise_standard_deviations = st_dev;
        sensor_type = st;
    }

    vector<vector<double> > getCHCTM (){
        return CHCTM_fixed_to_mobile;
    }

    vector<int> getActivePassed() {
        return passed_obstacles;
    }

    vector<vector<double> > getPassed() {
        return passed_points;
    }

    vector<vector<double> > getPassedTruth() {
        return passed_points_truth;
    }

    vector<Obstacle> pointToObstacle(double time) {
        vector<Obstacle> obstacles;
        for (int obs = 0; obs < passed_points.size(); obs++) {
            Obstacle o1;
            o1.setPosition(passed_points[obs]);
            o1.setSensorType(sensor_type);
            o1.setTimeStamp(time);
            obstacles.push_back(o1);
        }
        return obstacles;
    }


    void passThrough(vector<int> active, vector<vector<double> > obstacle_truth) {
        vector<vector<double> > passed;
        vector<int> active_passed;
        vector<vector<double> > passed_truth;
        if (active.size() == 0) {
            passed_points = passed;
            passed_obstacles = active_passed;
        }
        else {
            vector<vector<double> > obs_truth_to_transform = obstacle_truth; 
            for (int obs; obs < obstacle_truth.size(); obs++) {
                obs_truth_to_transform[obs].push_back(1);
            }
            obs_truth_to_transform = matrixTranspose(obs_truth_to_transform);
        vector<vector<double> > obstacle_truth_sensor_frame = matrixMultiply(CHCTM_fixed_to_mobile, obs_truth_to_transform);
        obstacle_truth_sensor_frame.pop_back();
        obstacle_truth_sensor_frame = matrixTranspose(obstacle_truth_sensor_frame); 
        vector<vector<double> > noise = getNoise(active.size());
        vector<vector<double> > obstacle_noisy_sensor_frame = vectorAdd(obstacle_truth_sensor_frame, noise);
        for (int obs = 0; obs < obstacle_noisy_sensor_frame.size(); obs++) {
            obstacle_noisy_sensor_frame[obs].push_back(1);
        }
        obstacle_noisy_sensor_frame = matrixTranspose(obstacle_noisy_sensor_frame);
        vector<vector<double> > obstacle_noisy = matrixMultiply(invertCHCTM(CHCTM_fixed_to_mobile), obstacle_noisy_sensor_frame);
        obstacle_noisy.pop_back();
        obstacle_noisy = matrixTranspose(obstacle_noisy);
        vector<bool> inFov = areInFOV(obstacle_noisy);
        for (int obs = 0; obs < inFov.size(); obs++) {
            if (inFov[obs]) {
                passed.push_back(obstacle_noisy[obs]);
                passed_truth.push_back(obstacle_truth[obs]);
                active_passed.push_back(active[obs]);
            }
        }
        passed_points = passed;
        passed_obstacles = active_passed;
        passed_points_truth = passed_truth;
        }
    }   

    vector<vector<double> > getNoise(int size) {
        vector<vector<double> > noise;
        random_device rd;  // Will be used to obtain a seed for the random number engine
        mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
        normal_distribution<double> normalX(0, noise_standard_deviations[0]);
        normal_distribution<double> normalY(0, noise_standard_deviations[1]);
        normal_distribution<double> normalZ(0, noise_standard_deviations[2]);
        for (int obs = 0; obs < size; obs++) {
            vector<double> noise_obstacle;
            noise_obstacle.push_back(normalX(gen));
            noise_obstacle.push_back(normalY(gen));
            noise_obstacle.push_back(normalZ(gen));
            noise.push_back(noise_obstacle);
        }
        return noise;    
    }

    vector<bool> areInFOV(vector<vector<double> > points) {
        vector<double> ones(points.size(), 1);
        points = matrixTranspose(points); 
        points.push_back(ones);
        vector<vector<double> > pt_sensor_frame = matrixMultiply(CHCTM_fixed_to_mobile, points);
        vector<bool> inFOV;
        for (int pt = 0; pt < points[0].size(); pt++) {
            double x = pt_sensor_frame[0][pt];
            double y = pt_sensor_frame[1][pt];
            double z = pt_sensor_frame[2][pt];
            vector<double> obstacle_params = getObstacleParametes(x, y, z);
            double azim = obstacle_params[0];
            double elev = obstacle_params[1];
            double dist = obstacle_params[2];
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

    vector<double> getObstacleParametes(const double x, const double y, const double z) {
        vector<double> obs_params;
        obs_params.push_back(roundThisTo<double>(180*atan2(y, x)/PI_6, 6));
        obs_params.push_back(roundThisTo<double>(180*atan2(z, sqrt(pow(x,2)+pow(y,2)))/PI_6, 6));
        obs_params.push_back(roundThisTo<double>(sqrt(pow(x,2) + pow(y,2) + pow(z,2)), 6));
        return obs_params;
    }

    vector<double> getObstacleParametes(vector<double> obstacle) {
        double x = obstacle[0], y = obstacle[1], z = obstacle[2];
        vector<double> obs_params;
        obs_params.push_back(roundThisTo<double>(180*atan2(y, x)/PI_6, 6));
        obs_params.push_back(roundThisTo<double>(180*atan2(z, sqrt(pow(x,2)+pow(y,2)))/PI_6, 6));
        obs_params.push_back(roundThisTo<double>(sqrt(pow(x,2) + pow(y,2) + pow(z,2)), 6));
        return obs_params;
    }

    bool isUpdateSample(int baserate, int current_sample) {
        return ((current_sample % (baserate/update_rate)) == 0);
    }
};
#endif