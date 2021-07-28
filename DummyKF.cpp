#ifndef DUMMYKF_CPP
#define DUMMYKF_CPP

#include <vector>
#include "Obstacle.cpp"
#include "CommonFunctions.cpp"

using namespace std;

class DummyKF {
    private:
    double position_estimate = 1e10;
    double previous_position = 1e10; // Only for starting since this is an impossible value.
    double velocity_estimate = 1e10;
    double previous_time = -1;
    bool enable = false;

    void updateVelocity(double time) {
        if (previous_position == 1e10) return;
        velocity_estimate = roundThisTo<double>((position_estimate - previous_position)/(time-previous_time), 6);
    }

    public: 
    bool isEnabled() {
        return enable;
    }

    void makeEnable() {
        enable = true;
    }

    void disable() {
        position_estimate = 1e10;
        previous_position = 1e10;
        velocity_estimate = 1e10;
        previous_time = -1;
        enable = false;
    }

    void estimate(double noisy, double truth, double time) {
        position_estimate = truth;
        updateVelocity(time);
        previous_position = position_estimate;
        previous_time = time;
    }

    double getPosition() {
        return position_estimate;
    }

    double getVelocity() {
        return velocity_estimate;
    }

    double getTime() {
        return previous_time;
    }

};

#endif