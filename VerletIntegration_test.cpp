#include "VerletIntegration.cpp"
#include "ObsGenerator.cpp"
#include <bits/stdc++.h>
#include <gtest/gtest.h>
#include <vector>

using namespace std;

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

TEST(MotionTests, startValsTest) {
    int i = 5;
    ObsGenerator g1(i, 0.1, 2.2, 300, 1, 3, 11);
    vector<vector<int> > m_modes = g1.getMotionModes();
    ASSERT_EQ(m_modes.size(), 3);
    ASSERT_EQ(m_modes.at(0).size(), i);
    for(int n_ob = 0; n_ob < i; n_ob++) {
        Verlet v1(m_modes.at(0).at(n_ob),
        m_modes.at(1).at(n_ob),
        m_modes.at(2).at(n_ob),
        1, 3, 11, 270); 
        vector<vector<double> > params = v1.getStartingKinematicParams();
        ASSERT_EQ(params.size(), 4);
        ASSERT_LE(v1.vectorMag(params.at(0)), 1);
        ASSERT_LE(v1.vectorMag(params.at(1)), 3);
        ASSERT_LE(v1.vectorMag(params.at(2)), 11);
        ASSERT_EQ(params.at(0).size(), 3);
        for(int axis = 0; axis < 3; axis++) {
            int mode = m_modes.at(axis).at(n_ob);
            switch (mode) {
                case 0:
                    ASSERT_DOUBLE_EQ(params.at(0).at(axis), 0.0);
                    ASSERT_DOUBLE_EQ(params.at(1).at(axis), 0.0);
                    break;
                case 1:
                    ASSERT_DOUBLE_EQ(params.at(0).at(axis), 0.0);
                    EXPECT_NE(params.at(1).at(axis), 0.0);
                    break;
                default:
                    ASSERT_TRUE(mode > 1);
                    break;
            }
        }
    }
}

TEST(QuantityClipper, ClipperTest) {
    Verlet g1(0, 1, 2, 1, 2, 10, 270);
    double max = 4;
    vector<double> subMax_3 = {1, 1, 2};
    vector<double> superMax_3 = {3, 3, 3};
    vector<double> subMax_4 = {2, 2, 1, 1};
    vector<double> superMax_4 = {3, 2, 2, 1};
    ASSERT_GE(max, g1.vectorMag(g1.checkAndClipMax(max, subMax_3)));
    ASSERT_GE(max, g1.vectorMag(g1.checkAndClipMax(max, subMax_4)));
    ASSERT_DOUBLE_EQ(max, g1.vectorMag(g1.checkAndClipMax(max, superMax_3)));
    ASSERT_DOUBLE_EQ(max, g1.vectorMag(g1.checkAndClipMax(max, superMax_4)));
}

TEST(JerkUpdateTests, JerkNoChangeTest) {
    vector<int> mode = {0, 1, 2};
    Verlet v1(0, 1, 2, 1, 3, 11, 270);
    vector<double> jold = v1.getJerk();
    vector<double> jcheck;
    jcheck.push_back(0.0); // Mode 0 along x axis;
    jcheck.push_back(0.0); // Mode 1 along y axis;
    jcheck.push_back(jold[2]); // Random constant jerk along z axis.
    for (int axis = 0; axis < 3; axis++) {
        ASSERT_DOUBLE_EQ(jold[axis], jcheck[axis]);
    }
    vector<double> jnew = v1.updateJerk(mode, jold);
    for (int axis = 0; axis < 3; axis++) {
        ASSERT_DOUBLE_EQ(jcheck[axis], jnew[axis]);
    }
}

TEST(JerkUpdateTests, JerkChangeTest) {
    vector<int> mode = {2, 3, 3};
    Verlet v1(2, 3, 3, 1, 3, 11, 270);
    vector<double> jold = v1.getJerk();
    vector<double> jnew = v1.updateJerk(mode, jold);
    ASSERT_GE(jold[0], jnew[0]);
    EXPECT_NE(jold[1], jnew[1]);
    EXPECT_NE(jold[2], jnew[2]);
}

TEST(AccelerationUpdateTests, GetCorrectAccMax) {
    double vmax = 11;
    double amax = 3;
    double jmax = 1;
    Verlet v1(1, 2, 3, jmax, amax, vmax, 270);
    double ti = round_to<double>((vmax/amax) - (amax/(2*jmax)), 6);
    double vp = round_to<double>(amax*ti, 6);
    for(float v_t = 0; v_t < vp ; v_t+= 0.5) {
        ASSERT_DOUBLE_EQ(3, v1.getAccMax(v_t));
    }
    for (float v_t = vp+0.0001; v_t <= vmax; v_t += 0.5) {
        double amaxv = round_to<double>(sqrt((2*jmax*amax*ti)+(amax*amax)-(2*jmax*v_t)), 6); 
        ASSERT_DOUBLE_EQ(amaxv, v1.getAccMax(v_t));
    }
    
}

TEST(AccelerationUpdateTests, AccelMode0Test) {
    Verlet v1(0, 0, 0, 1, 3, 11, 270);
    vector<double> oldAcc = v1.getAcc();
    vector<double> oldJerk = v1.getJerk();
    double vmax = 11;
    double amax = 3;
    double jmax = 1;
    double ti = round_to<double>((vmax/amax) - (amax/(2*jmax)), 6);
    double vp = round_to<double>(amax*ti, 6);
    // Velocity is 0, Maximum possible acceleration is amax; Acceleration is 0.
    vector<double> velocity = v1.getVel();
    velocity = v1.checkAndClipMax(0.0, velocity);
    vector<double> newAccel = v1.updateAcceleration(oldJerk, oldAcc, velocity);
    for (int axis = 0; axis < 3; axis++) {
        ASSERT_DOUBLE_EQ(0.0, newAccel[axis]);
    }
    // Velocity > vp, Maximum is dependent on vp, Acceleration is 0.
    vector<double> n1(3,1);
    velocity = v1.checkAndClipMax(vp+0.1, n1);
    newAccel = v1.updateAcceleration(oldJerk, oldAcc, velocity);
    for (int axis = 0; axis < 3; axis++) {
        ASSERT_DOUBLE_EQ(0.0, newAccel[axis]);
    }
    // Velocity = vmax, Maximum acceleration s 0, Acceleration is 0.
    velocity = v1.checkAndClipMax(vmax, n1);
    newAccel = v1.updateAcceleration(oldJerk, oldAcc, velocity);
    for (int axis = 0; axis < 3; axis++) {
        ASSERT_DOUBLE_EQ(0.0, newAccel[axis]);
    }
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}