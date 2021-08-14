#include <bits/stdc++.h>
#include <gtest/gtest.h>
#include "CommonFunctions.cpp"
#include <vector>
#include "Associator.cpp"
#include "Obstacle.cpp"
#include "Sensor.cpp"

TEST(MahalanobisTests, CalculateMahalanobisTest) {
    vector<vector<double> > chctm =     {{0.2114,    0.9156,    0.3420,         0},
   {-0.9744,    0.2250,         0,         0},
   {-0.0769,   -0.3333,    0.9397,         0},
        { 0,         0,         0,    1.0000}};
    Associator a1 ({0.1, 0.3, 0.5}, chctm);
    vector<vector<double> > points = {{20.5, 11, 12}, {4, 5, 1}, {4, 5, 10}, {1, 2, 36}, {143, 2, 1}, {-39, -60, 1}};
    vector<double> mahaDist;
    for (int i = 0; i < points.size(); i++) {
        mahaDist.push_back(a1.calculateMahalanobis({0, 0, 0}, points[i], 0.1));
    }
    vector<double> maha_truth = {9980,    1930,    2790,    7960,    280300,    279300};
    ASSERT_TRUE(areEqualVectors(mahaDist, maha_truth, 1e4));
}

TEST(AssociationTests, DISABLED_BothAreZeroSizedTest) {
    vector<double> sensor_noise_std = {0.01, 0.05, 0.5};
    Sensor s1({270, 40, 3, 150}, sensor_noise_std);
    vector<int> truth_active = {2, 5, 6};
    vector<int> list_active = {1, 2, 3, 4, 5, 6};
    vector<vector<double> > truth_passed = {{4, 5, 1}, {143, 2, 1}, {-39, -60, 1}};
    vector<vector<double> > points = {{20.5, 11, 12}, {4, 5, 1}, {4, 5, 10}, {1, 2, 36}, {143, 2, 1}, {-39, -60, 1}};
    s1.passThrough(list_active, points);
    vector<int> passed_active = s1.getActivePassed();
    vector<vector<double> > passed_points = s1.getPassed();
    ASSERT_TRUE(areEqualVectors(passed_active, truth_active));
    EXPECT_TRUE(areEqualVectors(passed_points, truth_passed, 1.5)); // Tolerance is 3 std of maximum std i.e. 0.5

    Associator a1(sensor_noise_std, invertCHCTM(s1.getCHCTM())); // Mobile to fixed.
    vector<Obstacle> tracked; // Empty
    vector<Obstacle> passed; // Empty
    ASSERT_EQ(0, a1.getAssociated().size());
    a1.associate(tracked, passed, truth_passed, 0.01);
    ASSERT_EQ(0, a1.getAssociated().size());
}

TEST(AssociationTests, DISABLED_TrackedZeroPassesFiniteSizedTest) {
    vector<double> sensor_noise_std = {0.01, 0.05, 0.5};
    Sensor s1({270, 40, 3, 150}, sensor_noise_std);
    vector<int> truth_active = {2, 5, 6};
    vector<int> list_active = {1, 2, 3, 4, 5, 6};
    vector<vector<double> > truth_passed = {{4, 5, 1}, {143, 2, 1}, {-39, -60, 1}};
    vector<vector<double> > points = {{20.5, 11, 12}, {4, 5, 1}, {4, 5, 10}, {1, 2, 36}, {143, 2, 1}, {-39, -60, 1}};
    s1.passThrough(list_active, points);
    vector<int> passed_active = s1.getActivePassed();
    vector<vector<double> > passed_points = s1.getPassed();
    ASSERT_TRUE(areEqualVectors(passed_active, truth_active));
    EXPECT_TRUE(areEqualVectors(passed_points, truth_passed, 1.5)); // Tolerance is 3 std of maximum std i.e. 0.5
    vector<Obstacle> passed_obstacles = s1.pointToObstacle(0.1);

    Associator a1(sensor_noise_std, invertCHCTM(s1.getCHCTM())); // Mobile to fixed.
    vector<Obstacle> tracked; // Empty
    vector<Obstacle> passed; // Empty
    ASSERT_EQ(0, a1.getAssociated().size());
    passed = passed_obstacles;
    cout << "Before passing:\n";
    for (int i = 0 ; i < 3; i++) {
        passed[i].printObstacle();
    }
    cout << "\n";
    a1.associate(tracked, passed, truth_passed, 0.1);
    ASSERT_EQ(3, a1.getAssociated().size());
    vector<Obstacle> a = a1.getAssociated();
    cout << "Assigned Data:\n";
    for (int i = 0; i < 3; i++) {
        a[i].printObstacle();
    }
}

TEST(AssociationTests, DISABLED_TrackedFinitePassedZeroSizedTest) {
    vector<double> sensor_noise_std = {0.01, 0.05, 0.5};
    Sensor s1({270, 40, 3, 150}, sensor_noise_std);
    vector<int> truth_active = {2, 5, 6};
    vector<int> list_active = {1, 2, 3, 4, 5, 6};
    vector<vector<double> > truth_passed = {{4, 5, 1}, {143, 2, 1}, {-39, -60, 1}};
    vector<vector<double> > points = {{20.5, 11, 12}, {4, 5, 1}, {4, 5, 10}, {1, 2, 36}, {143, 2, 1}, {-39, -60, 1}};
    s1.passThrough(list_active, points);
    vector<int> passed_active = s1.getActivePassed();
    vector<vector<double> > passed_points = s1.getPassed();
    ASSERT_TRUE(areEqualVectors(passed_active, truth_active));
    EXPECT_TRUE(areEqualVectors(passed_points, truth_passed, 1.5)); // Tolerance is 3 std of maximum std i.e. 0.5
    vector<Obstacle> passed_obstacles = s1.pointToObstacle(0.1);

    Associator a1(sensor_noise_std, invertCHCTM(s1.getCHCTM())); // Mobile to fixed.
    vector<Obstacle> tracked; // Empty
    vector<Obstacle> passed; // Empty
    ASSERT_EQ(0, a1.getAssociated().size());
    passed = passed_obstacles;
    cout << "Before passing:\n";
    for (int i = 0 ; i < 3; i++) {
        passed[i].printObstacle();
    }
    cout << "\n";
    a1.associate(passed, tracked, truth_passed, 0.1);
    ASSERT_EQ(3, a1.getAssociated().size());
    vector<Obstacle> a = a1.getAssociated();
    cout << "Assigned Data:\n";
    for (int i = 0; i < 3; i++) {
        cout << a[i].getOutFOVCounter() << " "; a[i].printObstacle();
        ASSERT_EQ(19, a[i].getOutFOVCounter());
    }
}


TEST(AssociationTests, Series_of_Associations_Test) {
    vector<double> sensor_noise_std = {1, 0.3, 0.15};
    Sensor s1({270, 40, 3, 100}, sensor_noise_std);
    Associator a1(sensor_noise_std, invertCHCTM(s1.getCHCTM()));
    vector<Obstacle> tracked; // Empty
    vector<Obstacle> passed; // Empty
    vector<vector<double> > truth;
    vector<vector<double> > noisy;
    vector<vector<double> > tr_truth;
    a1.associate(tracked, passed, truth, 2.5);
    ASSERT_EQ(a1.getAssociated().size(), 0);
    ASSERT_EQ(a1.getAssociatedTruth().size(), 0);

    noisy = {{19.6783,-2.90718,3.55893}, {-9.83458,21.848,-4.53554}};
    truth = {{19.5821,-2.7759,3.53062}, {-10.3861,22.009,-4.66707}};
    for (int i = 0; i < noisy.size(); i++) {
        Obstacle o1;
        o1.setPosition(noisy[i]);
        o1.setTimeStamp(2.6);
        passed.push_back(o1);
    }
    a1.associate(tracked, passed, truth, 2.6);
    ASSERT_EQ(2, a1.getAssociated().size());
    vector<Obstacle> a = a1.getAssociated();
    cout << "Assigned Data:\n";
    for (int i = 0; i < a.size(); i++) {
        cout << a[i].getOutFOVCounter() << " "; a[i].printObstacle();
    }
    tracked = a;
    tr_truth = a1.getAssociatedTruth();
    cout<< "\nTracked: \n";
    for (int i = 0; i < a.size(); i++) {
        tracked[i].setPosition(tr_truth[i]);
        cout << tracked[i].getOutFOVCounter() << " "; tracked[i].printObstacle();
    }

    noisy = {{18.5019,-2.26727,3.52221}, {-10.7579,21.3141,-4.54381}};
    truth = {{19.6498,-2.25787,3.38762}, {-10.8041,21.738,-4.49221}};
    passed = {};
    for (int i = 0; i < noisy.size(); i++) {
        Obstacle o1;
        o1.setPosition(noisy[i]);
        o1.setTimeStamp(2.7);
        passed.push_back(o1);
    }
    a1.associate(tracked, passed, truth, 2.7);
    
    tracked = a1.getAssociated();
    tr_truth = a1.getAssociatedTruth();
    cout<< "\nTracked: \n";
    for (int i = 0; i < tracked.size(); i++) {
        tracked[i].setPosition(tr_truth[i]);
        cout << tracked[i].getOutFOVCounter() << " "; tracked[i].printObstacle();
    }


    noisy = {{19.6124,-1.75142,3.30646}};
    truth = {{19.7206,-1.74686,3.23078}};
    passed = {};
    for (int i = 0; i < noisy.size(); i++) {
        Obstacle o1;
        o1.setPosition(noisy[i]);
        o1.setTimeStamp(2.8);
        passed.push_back(o1);
    }
    a1.associate(tracked, passed, truth, 2.8);
    
    tracked = a1.getAssociated();
    tr_truth = a1.getAssociatedTruth();
    cout<< "\nTracked: \n";
    for (int i = 0; i < tracked.size(); i++) {
        tracked[i].setPosition(tr_truth[i]);
        cout << tracked[i].getOutFOVCounter() << " "; tracked[i].printObstacle();
    }

    noisy = {{27.8726,5.07572,-7.39295}, {19.2378,-1.63349,3.00456}};
    truth = {{28.6242,5.63527,-7.20261}, {19.7948,-1.24267,3.06011}};
    passed = {};
    for (int i = 0; i < noisy.size(); i++) {
        Obstacle o1;
        o1.setPosition(noisy[i]);
        o1.setTimeStamp(2.9);
        passed.push_back(o1);
    }
    a1.associate(tracked, passed, truth, 2.9);
    
    tracked = a1.getAssociated();
    tr_truth = a1.getAssociatedTruth();
    cout<< "\nTracked: \n";
    for (int i = 0; i < tracked.size(); i++) {
        tracked[i].setPosition(tr_truth[i]);
        cout << tracked[i].getOutFOVCounter() << " "; tracked[i].printObstacle();
    }
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}