#ifndef OBSGEN_CPP
#define OBSGEN_CPP
#include <random>
#include <vector>
#include <math.h>
#include <iostream>
#include "VerletIntegration.cpp"
#include "CommonFunctions.cpp"

using namespace std;

class ObsGenerator {
    private:
    vector<int> start_samples;              // N x 1 array, defining at which number of sample the object will start to appear.
    vector<int> end_samples;                // N x 1 array, defining at which number of sample the object will stop to appear.
    int num_of_obstacles;                   // N value
    int base_rate;                          // Rate at which we will provide samples.   
    double jerk_max;        // (m/s^3) Maximum Magnitude Jerk the object may experience.
    double acc_max;         // (m/s^2) Maximum Magnitude of Acceleration the object may experience.
    double vel_max;          // (m/s)   Maximum Magnitude of Velocity the object may experience.
    /*
    /*
    x_mode, y_mode, z_mode - which mode the motion will be
    0 - random start point, random constant start velocity, zero acceleration and jerk.
    1 - random start point and velocity, random constant acceleration, no jerk.
    2 - random start point, velocity and acceleration, random constant jerk.
    3 - random start point, velocity and acceleration, varying jerk.
    */
    vector<vector<int> > motion_modes;      // 3 x N matrix for x,y,z modes of motion for N objects.
    vector<double> time_stamps;              // number of samples X 1 array
    int current_sample;
    vector<Verlet> verlet;

    public:
    ObsGenerator(int num, double start, double end, int rate, double j_max, double a_max, double v_max) {
        using namespace std;
        num_of_obstacles = num;
        base_rate = rate;
        jerk_max = roundThisTo<double>(j_max, 6);   // Rounding to 6 decimal places due to precision issues.
        acc_max = roundThisTo<double>(a_max, 6);
        vel_max = roundThisTo<double>(v_max, 6);
        random_device rd;  // Will be used to obtain a seed for the random number engine
        mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
        uniform_int_distribution<> dis((int)(start*base_rate +1), (int)(end*base_rate - rate*(0.2*(end-start))));
        uniform_int_distribution<> dis2((int)(start*base_rate +base_rate*(0.2*(end-start))), (int)(end*base_rate - 1));
        uniform_int_distribution<> dis_i(0,3); // in range [0,3]
        vector<int> x_modes; // N x 1 array
        vector<int> y_modes;
        vector<int> z_modes;
        for (int i = 0; i < num_of_obstacles; i++) {
            x_modes.push_back(dis_i(gen));
            y_modes.push_back(dis_i(gen));
            z_modes.push_back(dis_i(gen));
            start_samples.push_back(dis(gen)+1);
            int end_sample = start_samples.at(i);
            do {
                end_sample = dis2(gen);
            } while ((end_sample <= start_samples.at(i)));
            end_samples.push_back(end_sample);
        }
        motion_modes.push_back(x_modes);
        motion_modes.push_back(y_modes);
        motion_modes.push_back(z_modes);
        int max_sample = (int)(base_rate*end);
        for (int i = 0; i < max_sample; i++) {
            time_stamps.push_back(roundThisTo<double>((i*(1.0/((double)base_rate))), 10)); // Rounding to 9 decimal places i.e. ns
        }
        current_sample = 0;
        initializeVerlet();
    }

    
    void initializeVerlet() {
        for (int obs = 0; obs < num_of_obstacles; obs++) {
            int modeX = motion_modes[0][obs];
            int modeY = motion_modes[1][obs];
            int modeZ = motion_modes[2][obs];
            Verlet v1(modeX, modeY, modeZ, jerk_max, acc_max, vel_max, base_rate);
            verlet.push_back(v1);
        }
    }

    vector<int> getStartSamples() {             // For Testing
        return start_samples;
    }

    vector<int> getEndSamples() {               // For Testing
        return end_samples;
    }

    vector<vector<int> > getMotionModes() {     // For Testing
        return motion_modes;
    }

    vector<double> getTimeStamps() {             // For Testing
        return time_stamps;
    }


    bool hasNext() {
        return current_sample < time_stamps.size()-1;
    }

    void iterate() {
        current_sample++;
    }

    vector<int> activeObjects() {
        vector<int> retVec;
        for (int obs = 1; obs <= num_of_obstacles; obs++) {
            if (current_sample >= start_samples[obs-1] && current_sample < end_samples[obs-1]) {
                retVec.push_back(obs);
            }   
        } 
        return retVec;
    }
    
    vector<vector<double> > getNext() {
        vector<vector<double> > retVec;
        for (int obs = 0; obs < num_of_obstacles; obs++) {
            if(current_sample >= start_samples[obs] && current_sample < end_samples[obs]) {
                retVec.push_back(verlet[obs].getPos());
                verlet[obs].update();
            }
        }
        current_sample++;
        return retVec;
    }

    double currentTime() {
        return time_stamps[current_sample];
    }

    double currentSample() {
        return current_sample;
    }

    vector<vector<vector<double> > > testingEmptyVectors() {
        vector<vector<vector<double> > > retVec;
            for(int i = 0; i < 10; i++) {
                vector<vector<double> > objVec;
                for (int j = 0; j < 5; j++) {
                    vector<double> empVec;
                    objVec.push_back(empVec);
                }
                retVec.push_back(objVec);
            }
            for(int i = 0; i < 10; i++) {
                vector<vector<double> > objVec;
                for (int j = 0; j < 5; j++) {
                    vector<double> empVec;
                    for (int k = 0; k < 3; k++) {
                        empVec.push_back((double)(k*j));
                    }
                    objVec.push_back(empVec);
                }
                retVec.push_back(objVec);
            }
        return retVec;
    }
};
#endif