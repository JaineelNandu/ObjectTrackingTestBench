#ifndef ASSOCIATOR_CPP
#define ASSOCIATOR_CPP

#include <vector>
#include "Sensor.cpp"
#include "CommonFunctions.cpp"

using namespace std;


class Associator
{
private:
    int new_ID = 1;
    vector<vector<double> > cov_sensor;
    vector<vector<double> > cov_inertial;
    vector<vector<double> > inv_cov_inertial;
public:
    Associator(vector<double> noise_std, vector<vector<double> > chctm) {
        vector<vector<double> > chctm_rot;
        for (int row = 0; row < 3; row++) {
            vector<double> row_vector;
            for (int col = 0; col < 3; col++) row_vector.push_back(chctm[row][col]);
            chctm_rot.push_back(row_vector);
        }
        cov_sensor.push_back({noise_std[0], 0, 0});
        cov_sensor.push_back({0, noise_std[1], 0});
        cov_sensor.push_back({0, 0, noise_std[2]});

    }

    double calculateProbability();
};
#endif