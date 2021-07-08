/*
    Author: Jaineel Nandu 
    Email: nandujaineel@vt.edu  
*/
#include <vector>
#include <math.h>
#include <random>
#include <iostream>

using namespace std; 

class Verlet {
    private:
    int baserate;
    double jerk_max;
    double acc_max;
    double vel_max;
    vector<int> modes;
    vector<double> jerk;
    vector<double> acc;
    vector<double> vel;
    vector<double> pos;
    double ti; // Time at which acceleration must start decreasing (ref. Report 3.5 Acceleration vs. Velocity)
    double vp; // Velocity at which acceleration must start decreasing (ref. Report 3.5 Acceleration vs. Velocity)

    public:
    Verlet(int mdx, int mdy, int mdz, double jMax, double aMax, double vMax, int rate) {
        modes.push_back(mdx);
        modes.push_back(mdy);
        modes.push_back(mdz);
        jerk_max = jMax;
        acc_max = aMax;
        vel_max = vMax;
        baserate = rate;
        vector<vector<double> > kinec = getStartingKinematicParams();
        jerk = kinec[0];
        acc = kinec[1];
        vel = kinec[2];
        pos = kinec[3];
        ti = round_to<double>((vel_max/acc_max) - (acc_max/(2*jerk_max)), 6);
        vp = round_to<double>(acc_max*ti, 6);
    }

    vector<double> getJerk() {
        return jerk;
    }

    vector<double> getAcc() {
        return acc;
    }

    vector<double> getVel() {
        return vel;
    }

    vector<double> getPos() {
        return pos;
    }

    template<typename T>
    static void print_vec(T vec)
    {
        std::cout << vec;
    }


    template<typename T>
    static void print_vec(std::vector<T> vec)
    {
        int size = vec.size();
        if (size <= 0) {
            std::cout << "invalid vector";
            return;
        }
        std::cout << '{';
        for (int l = 0; l < size - 1; l++) {
            print_vec(vec[l]);
            std::cout << ',';
        }
        print_vec(vec[size - 1]);
        std::cout << '}';
    }

    template<typename T>
    static T round_to(T x, int n){ 
	    int d = 0; 
	    if((x * pow(10, n + 1)) - (floor(x * pow(10, n))) > 4) d = 1; 
	    x = (floor(x * pow(10, n)) + d) / pow(10, n); 
	    return x; 
    }

    /**
     * Updates the magnitude to max if it is higher.
    **/
    template<typename T>
    vector<T> checkAndClipMax(T target, vector<T> quantity) {
        vector<T> retQuant;
        T magnitude = vectorMag(quantity);
        if (magnitude <= target) {
            retQuant = quantity;
        }
        else {
            for(int i = 0; i < quantity.size(); i++) {
                retQuant.push_back(round_to<T>(((target/magnitude)*quantity.at(i)), 6));
            }
        } 
        return retQuant;
    }

    /**
     * Calculates Magnitude i.e. l2norm of the vector passed. 
    **/
    template<typename T>
    T vectorMag(vector<T> vec) {
        double sum = 0;
        for(int i = 0; i < vec.size(); i++) {
            sum += vec.at(i)*vec.at(i);
        }
        return round_to<T>(sqrt(sum), 6);
    }

    vector<vector<double> > getStartingKinematicParams() {
        vector<vector<double> > retVec;
        random_device rd;  // Will be used to obtain a seed for the random number engine
        mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
        vector<double> start_pos_box_max = {30, 30, 10}; // Magnitude of position in that co-ordinate, only for starting value though.

        // Distributions to generate relevant data
        uniform_real_distribution<> dis_jerk(-jerk_max, jerk_max);
        uniform_real_distribution<> dis_acc(-acc_max, acc_max);
        uniform_real_distribution<> dis_vel(-vel_max, vel_max);
        uniform_real_distribution<> dis_pos_x(-start_pos_box_max.at(0), start_pos_box_max.at(0));
        uniform_real_distribution<> dis_pos_y(-start_pos_box_max.at(1), start_pos_box_max.at(1));
        uniform_real_distribution<> dis_pos_z(-start_pos_box_max.at(2), start_pos_box_max.at(2));
        vector<double> jer;
        vector<double> acc;
        vector<double> vel;
        vector<double> po;
        for(int i = 0; i < 3; i++) {
            // Generating random start values
            jer.push_back(dis_jerk(gen)/2);
            acc.push_back(dis_acc(gen)/2);
            vel.push_back(dis_vel(gen)/2);
        }
        retVec.push_back(jer);
        double aMax = getAccMax(vectorMag(vel));
        acc = checkAndClipMax(aMax, acc);
        retVec.push_back(acc);
        retVec.push_back(vel);
        // Fixing starting values according to their mode using fix_mode function.
        retVec = fix_mode(retVec);
        po.push_back(dis_pos_x(gen));
        po.push_back(dis_pos_y(gen));
        po.push_back(dis_pos_z(gen));
        retVec.push_back(po);
        return retVec;
    }

    /** 
     * Fixes the starting parameters according to the motion modes of the object
    **/
    vector<vector<double> > fix_mode(vector<vector<double> > vals) {
        vector<double> lims;
        lims.push_back(jerk_max);
        lims.push_back(acc_max);
        lims.push_back(vel_max);
        for(int i = 0; i < 3; i++) {
            /*
            x_mode, y_mode, z_mode - which mode the motion will be
            0 - random start point, random start velocity, zero acceleration and jerk.
            1 - random start point and velocity, random constant acceleration, no jerk.
            2 - random start point, velocity and acceleration, random constant jerk.
            3 - random start point, velocity and acceleration, varying jerk.
            */
            int mode = modes.at(i);
            switch (mode) {
                case 0:
                    vals[0][i] = 0.0;
                    vals[1][i] = 0.0;
                    break;
                case 1:
                    vals[0][i] = 0.0;
                    break;
                case 2:
                    break;
                case 3:
                    break;
            }
        }
        for(int i = 0; i < 3; i++) {
            vals[i] = checkAndClipMax(lims.at(i), vals.at(i));
        }
        return vals;
    }

    double getAccMax(double v_t) {
        if (v_t >= vel_max) {
            return 0.0;
        }
        else if (v_t > vp) {
            return round_to<double>(sqrt((2*jerk_max*acc_max*ti)+(acc_max*acc_max)-(2*jerk_max*v_t)), 6);
        }
        return acc_max; //round_to<double>(acc_max, 6);
    }
};