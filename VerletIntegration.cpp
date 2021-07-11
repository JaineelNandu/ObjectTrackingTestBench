/*
    Author: Jaineel Nandu 
    Email: nandujaineel@vt.edu  
*/
#ifndef VERLET_CPP
#define VERLET_CPP
#include <vector>
#include <math.h>
#include <random>
#include "CommonFunctions.cpp"
#include <iostream>

using namespace std; 

class Verlet {
    private:
    int baserate;
    double jerk_maximum_magnitude;
    double acceleration_maximum_magnitude;
    double velocity_maximum_magnitude;
    vector<int> modes;
    vector<double> jerks;
    vector<double> accelerations;
    vector<double> velocities;
    vector<double> position_coordinates;
    double ti; // Time at which acceleration must start decreasing (ref. Report 3.5 Acceleration vs. Velocity)
    double vp; // Velocity at which acceleration must start decreasing (ref. Report 3.5 Acceleration vs. Velocity)

    public:
    Verlet(int mdx, int mdy, int mdz, double jMax, double aMax, double vMax, int rate) {
        modes.push_back(mdx);
        modes.push_back(mdy);
        modes.push_back(mdz);
        jerk_maximum_magnitude = round_to<double>(jMax, 6);
        acceleration_maximum_magnitude = round_to<double>(aMax, 6);
        velocity_maximum_magnitude = round_to<double>(vMax, 6);
        baserate = rate;
        ti = round_to<double>((velocity_maximum_magnitude/acceleration_maximum_magnitude) - (acceleration_maximum_magnitude/(2*jerk_maximum_magnitude)), 6);
        vp = round_to<double>(acceleration_maximum_magnitude*ti, 6);
        vector<vector<double> > kinematic_parameters = getStartingKinematicParams();
        jerks = kinematic_parameters[0];
        accelerations = kinematic_parameters[1];
        velocities = kinematic_parameters[2];
        position_coordinates = kinematic_parameters[3];
    }

    vector<double> getJerk() {
        return jerks;
    }

    vector<double> getAcc() {
        return accelerations;
    }

    vector<double> getVel() {
        return velocities;
    }

    vector<double> getPos() {
        return position_coordinates; // Returns a 3 X 1 Vector of position coordinates.
    }

    vector<vector<double> > getStartingKinematicParams() {
        vector<vector<double> > retVec;
        random_device rd;  // Will be used to obtain a seed for the random number engine
        mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
        vector<double> start_pos_box_max = {30, 30, 10}; // Magnitude of position in that coordinate, only for starting value though.

        // Distributions to generate relevant data
        uniform_real_distribution<> dis_jerk(-jerk_maximum_magnitude, jerk_maximum_magnitude);
        uniform_real_distribution<> dis_acc(-acceleration_maximum_magnitude, acceleration_maximum_magnitude);
        uniform_real_distribution<> dis_vel(-velocity_maximum_magnitude, velocity_maximum_magnitude);
        uniform_real_distribution<> dis_pos_x(-start_pos_box_max.at(0), start_pos_box_max.at(0));
        uniform_real_distribution<> dis_pos_y(-start_pos_box_max.at(1), start_pos_box_max.at(1));
        uniform_real_distribution<> dis_pos_z(-start_pos_box_max.at(2), start_pos_box_max.at(2));
        vector<double> jer;
        vector<double> acc;
        vector<double> vel;
        vector<double> po;
        for(int i = 0; i < 3; i++) {
            // Generating random start values
            jer.push_back(round_to<double>(dis_jerk(gen)/2, 6));
            acc.push_back(round_to<double>(dis_acc(gen)/2, 6));
            vel.push_back(round_to<double>(dis_vel(gen)/2, 6));
        }
        retVec.push_back(jer);
        double aMax = getMaximumPossibleAcceleration(vectorMagnitude(vel));
        acc = checkAndClipMax(aMax, acc);
        retVec.push_back(acc);
        retVec.push_back(vel);
        // Fixing starting values according to their mode using fix_mode function.
        retVec = fix_mode(retVec);
        po.push_back(round_to<double>(dis_pos_x(gen), 6));
        po.push_back(round_to<double>(dis_pos_y(gen), 6));
        po.push_back(round_to<double>(dis_pos_z(gen), 6));
        retVec.push_back(po);
        return retVec;
    }

    /** 
     * Fixes the starting parameters according to the motion modes of the object
    **/
    vector<vector<double> > fix_mode(vector<vector<double> > params) {
        vector<double> limits;
        limits.push_back(jerk_maximum_magnitude);
        limits.push_back(acceleration_maximum_magnitude);
        limits.push_back(velocity_maximum_magnitude);
        for(int axis = 0; axis < 3; axis++) {
            /*
            x_mode, y_mode, z_mode - which mode the motion will be
            0 - random start point, random start velocity, zero acceleration and jerk.
            1 - random start point and velocity, random constant acceleration, no jerk.
            2 - random start point, velocity and acceleration, random constant jerk.
            3 - random start point, velocity and acceleration, varying jerk.
            */
            int mode = modes.at(axis);
            switch (mode) {
                case 0:
                    params[0][axis] = round_to<double>(0.0, 6);
                    params[1][axis] = round_to<double>(0.0, 6);
                    break;
                case 1:
                    params[0][axis] = round_to<double>(0.0, 6);
                    break;
                case 2:
                    break;
                case 3:
                    break;
            }
        }
        for(int i = 0; i < 3; i++) {
            params[i] = checkAndClipMax(limits.at(i), params.at(i));
        }
        return params;
    }

    double getMaximumPossibleAcceleration(double magnitude_of_old_velocity) {
        if (magnitude_of_old_velocity >= velocity_maximum_magnitude) {
            return round_to<double>(0.0, 6);
        }
        else if (magnitude_of_old_velocity > vp) {
            return round_to<double>(
                                    sqrt((2*jerk_maximum_magnitude*acceleration_maximum_magnitude*ti)
                                          +(acceleration_maximum_magnitude*acceleration_maximum_magnitude)
                                          -(2*jerk_maximum_magnitude*magnitude_of_old_velocity))
                                    , 6);
        }
        return acceleration_maximum_magnitude;
    }

    vector<double> updateJerk(vector<int> mode, vector<double> oldJerk) {
        vector<double> retJerk;
        for (int axis = 0; axis < 3; axis++) {
            if (mode[axis] == 3) {
                random_device rd;  // Will be used to obtain a seed for the random number engine
                mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
                uniform_real_distribution<> dis_jerk(-jerk_maximum_magnitude, jerk_maximum_magnitude);
                retJerk.push_back(round_to<double>(dis_jerk(gen), 6));
                continue;
            }
            retJerk.push_back(oldJerk[axis]);
        }
        retJerk = checkAndClipMax(jerk_maximum_magnitude, retJerk);
        return retJerk;
    }

    vector<double> updateAcceleration(vector<double> oldJerk, vector<double> oldAcceleration, vector<double> oldVelocity) {
        double accMax = getMaximumPossibleAcceleration(vectorMagnitude(oldVelocity));
        vector<double> retAcc;
        for (int axis = 0; axis < 3; axis++) {
            retAcc.push_back(
                                (oldAcceleration[axis]) 
                                + (oldJerk[axis])
                                * round_to<double>(((double)1/(double)baserate)
                                            ,9)
                            );
        }
        retAcc = checkAndClipMax(accMax, retAcc);
        return retAcc;
    }

    vector<double> updateVelocity(vector<double> oldAcceleration, vector<double> newAcceleration, vector<double> oldVelocity) {
        vector<double> retVel;
        for (int axis = 0; axis < 3; axis++) {
            retVel.push_back(
                                round_to<double>(oldVelocity[axis] 
                                                + (((oldAcceleration[axis]+newAcceleration[axis])/2)
                                                *(1/(double)baserate))
                                            ,6)
                            );
        }
        retVel = checkAndClipMax(velocity_maximum_magnitude, retVel);
        return retVel;
    }

    vector<double> updatePosition(vector<double> oldAcceleration, vector<double> oldVelocity, vector<double> oldPosition) {
        vector<double> retPos;
        for (int axis = 0; axis < 3; axis++) {
            retPos.push_back(
                                round_to<double>((oldAcceleration[axis]*(0.5/((double)(baserate*baserate)))) 
                                                + (oldVelocity[axis]/(double)baserate) 
                                                + oldPosition[axis]
                                            ,6)
                            );
        }
        return retPos;
    }

    void update() {
        position_coordinates = updatePosition(accelerations, velocities, position_coordinates);
        vector<double> newacc = updateAcceleration(jerks, accelerations, velocities);
        jerks = updateJerk(modes, jerks);
        velocities = updateVelocity(accelerations, newacc, velocities);
        accelerations = newacc;
    }
};
#endif