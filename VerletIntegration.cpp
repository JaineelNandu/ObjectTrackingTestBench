/**
 *      Author   :   Jaineel Nandu
 *      email    :   nandujaineel@vt.edu
 *                   jaineel.nandu02@gmail.com
 **/

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
    /**
     * Constructor
     **/
    Verlet(int mdx, int mdy, int mdz, double jMax, double aMax, double vMax, int rate) {
        modes.push_back(mdx);
        modes.push_back(mdy);
        modes.push_back(mdz);
        jerk_maximum_magnitude = roundThisTo<double>(jMax, 6);
        acceleration_maximum_magnitude = roundThisTo<double>(aMax, 6);
        velocity_maximum_magnitude = roundThisTo<double>(vMax, 6);
        baserate = rate;
        ti = roundThisTo<double>(
                                (velocity_maximum_magnitude/acceleration_maximum_magnitude) 
                                - (acceleration_maximum_magnitude/(2*jerk_maximum_magnitude))
                            , 6);
        vp = roundThisTo<double>(acceleration_maximum_magnitude*ti, 6);
        vector<vector<double> > kinematic_parameters = getStartingKinematicParams();
        jerks = kinematic_parameters[0];
        accelerations = kinematic_parameters[1];
        velocities = kinematic_parameters[2];
        position_coordinates = kinematic_parameters[3];
    }

    /**
     * Returns current jerk values [3 X 1] vector.
     **/
    vector<double> getJerk() {
        return jerks;
    }

    /**
     * Returns current accelration values [3 X 1] vector.
     **/
    vector<double> getAcc() {
        return accelerations;
    }

    /**
     * Returns current velocity values [3 X 1] vector.
     **/
    vector<double> getVel() {
        return velocities;
    }

    /**
     * Returns current position coordinates [3 X 1] vector.
     **/
    vector<double> getPos() {
        return position_coordinates; 
    }

    /**
     * Generates random kinematic parameters for the initial time.
     **/
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
            jer.push_back(roundThisTo<double>(dis_jerk(gen)/2, 6));
            acc.push_back(roundThisTo<double>(dis_acc(gen)/2, 6));
            vel.push_back(roundThisTo<double>(dis_vel(gen)/2, 6));
        }
        retVec.push_back(jer);
        double aMax = getMaximumPossibleAcceleration(vectorMagnitude(vel));
        acc = checkMagnitudeAndClipToMax(aMax, acc);
        retVec.push_back(acc);
        retVec.push_back(vel);
        // Fixing starting values according to their mode using fix_mode function.
        retVec = fix_mode(retVec);
        po.push_back(roundThisTo<double>(dis_pos_x(gen), 6));
        po.push_back(roundThisTo<double>(dis_pos_y(gen), 6));
        po.push_back(roundThisTo<double>(dis_pos_z(gen), 6));
        retVec.push_back(po);
        return retVec;
    }

    /** 
     * Fixes the starting parameters according to the motion modes of the object.
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
                    params[0][axis] = roundThisTo<double>(0.0, 6);
                    params[1][axis] = roundThisTo<double>(0.0, 6);
                    break;
                case 1:
                    params[0][axis] = roundThisTo<double>(0.0, 6);
                    break;
                case 2:
                    break;
                case 3:
                    break;
            }
        }
        for(int i = 0; i < 3; i++) {
            params[i] = checkMagnitudeAndClipToMax(limits.at(i), params.at(i));
        }
        return params;
    }

    /**
     * Calculates maximum possible accleration (Refer 3.5 of the report).
     **/
    double getMaximumPossibleAcceleration(double magnitude_of_old_velocity) {
        if (magnitude_of_old_velocity >= velocity_maximum_magnitude) {
            return roundThisTo<double>(0.0, 6);
        }
        else if (magnitude_of_old_velocity > vp) {
            return roundThisTo<double>(
                                    sqrt((2*jerk_maximum_magnitude*acceleration_maximum_magnitude*ti)
                                          +(acceleration_maximum_magnitude*acceleration_maximum_magnitude)
                                          -(2*jerk_maximum_magnitude*magnitude_of_old_velocity))
                                    , 6);
        }
        return acceleration_maximum_magnitude;
    }

    /**
     * Calculates new Jerks with respect to mode of motion.
     **/
    vector<double> getNewJerks(vector<int> mode, vector<double> oldJerks) {
        vector<double> new_jerks;
        for (int axis = 0; axis < 3; axis++) {
            if (mode[axis] == 3) {
                random_device rd;  // Will be used to obtain a seed for the random number engine
                mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
                uniform_real_distribution<> dis_jerk(-jerk_maximum_magnitude, jerk_maximum_magnitude);
                new_jerks.push_back(roundThisTo<double>(dis_jerk(gen), 6));
                continue;
            }
            new_jerks.push_back(oldJerks[axis]);
        }
        new_jerks = checkMagnitudeAndClipToMax(jerk_maximum_magnitude, new_jerks);
        return new_jerks;
    }
    
    /**
     * Calculates new Accelerations.
     **/
    vector<double> getNewAccelerations(vector<double> oldJerks, vector<double> oldAccelerations, vector<double> oldVelocities) {
        double accMax = getMaximumPossibleAcceleration(vectorMagnitude(oldVelocities));
        vector<double> new_accelerations;
        for (int axis = 0; axis < 3; axis++) {
            new_accelerations.push_back(
                                (oldAccelerations[axis]) 
                                + (oldJerks[axis])
                                * roundThisTo<double>(((double)1/(double)baserate)
                                            ,9)
                            );
        }
        new_accelerations = checkMagnitudeAndClipToMax(accMax, new_accelerations);
        return new_accelerations;
    }

    /**
     * Calculates new Velocities.
     **/
    vector<double> getNewVelocities(vector<double> oldAccelerations, vector<double> newAccelerations, vector<double> oldVelocities) {
        vector<double> new_velocities;
        for (int axis = 0; axis < 3; axis++) {
            new_velocities.push_back(
                                roundThisTo<double>(oldVelocities[axis] 
                                                + (((oldAccelerations[axis]+newAccelerations[axis])/2)
                                                *(1/(double)baserate))
                                            ,6)
                            );
        }
        new_velocities = checkMagnitudeAndClipToMax(velocity_maximum_magnitude, new_velocities);
        return new_velocities;
    }

    /**
     * Calculates new Position coordinates.
     **/
    vector<double> getNewPositions(vector<double> oldAccelerations, vector<double> oldVelocities, vector<double> oldPositions) {
        vector<double> new_positions;
        for (int axis = 0; axis < 3; axis++) {
            new_positions.push_back(
                                roundThisTo<double>((oldAccelerations[axis]*(0.5/((double)(baserate*baserate)))) 
                                                + (oldVelocities[axis]/(double)baserate) 
                                                + oldPositions[axis]
                                            ,6)
                            );
        }
        return new_positions;
    }

    /**
     * Updates the Kinematic Parameters with respect to the next timestep.
     **/
    void update() {
        position_coordinates = getNewPositions(accelerations, velocities, position_coordinates);
        vector<double> newacc = getNewAccelerations(jerks, accelerations, velocities);
        jerks = getNewJerks(modes, jerks);
        velocities = getNewVelocities(accelerations, newacc, velocities);
        accelerations = newacc;
    }
};
#endif