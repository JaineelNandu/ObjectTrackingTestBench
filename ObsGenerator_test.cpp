#include "ObsGenerator.cpp"
#include<bits/stdc++.h>
#include <gtest/gtest.h>
#include <vector>
#include "CommonFunctions.cpp"

using namespace std;    

TEST(TimingTests, startSampleTest) {
    ObsGenerator g1(3, 0.1, 1.1, 270, 1, 3, 11);
    ASSERT_EQ(3, g1.getStartSamples().size());
}

TEST(TimingTests, endSampleTest) {
    ObsGenerator g1(7, 0.1, 1.1, 270, 1, 3, 11);
    ASSERT_EQ(7, g1.getEndSamples().size());
}

TEST(TimingTests, startEndTimeTest) {
    ObsGenerator g1(6, 0.6, 10, 270, 1, 3, 11);
    vector<int> starts = g1.getStartSamples();
    vector<int> ends = g1.getEndSamples();
    for(int i = 0; i<starts.size(); i++) {
        ASSERT_LE(starts.at(i), ends.at(i));
    }
}

TEST(TimingTests, totalTimeStepsTest) {
    ObsGenerator g1(3,0.1, 2.2, 300, 1, 3, 11);
    ASSERT_EQ((int)(2.2*300), g1.getTimeStamps().size());
    vector<double> t = g1.getTimeStamps();
    ASSERT_NEAR(roundThisTo<double>((double)1/((double)300), 9), t[1], 1e-9);
    ASSERT_NEAR(roundThisTo<double>((double)((int)(2.2*300)-1)*(double)(1/((double)300)), 9), t[t.size()-1], 1e-9);
}

TEST(MotionStartTests, motionModesTest) {
    ObsGenerator g1(3,0.1, 2.2, 300, 1, 3, 11);
    vector<vector<int> > m_modes = g1.getMotionModes();
    for(int i = 0; i < m_modes.size(); i++) {
        for(int j = 0; j < m_modes.at(i).size(); j++) {
            ASSERT_TRUE(m_modes.at(i).at(j) >= 0 && m_modes.at(i).at(j) <=3);
        }
    }
}


TEST(EmpVec, EmpVecTest) {
    ObsGenerator g1(3,0.1, 2.2, 300, 1, 3, 11);
    vector<vector<vector<double> > > testVec = g1.testingEmptyVectors();
    ASSERT_EQ(20, testVec.size());
    for (int i = 0; i < 20; i++) {
        ASSERT_EQ(5, testVec.at(i).size());
        if (i < 10) {
            for (int j = 0; j < 5; j++) {
                ASSERT_EQ(0, testVec.at(i).at(j).size());
            }
        }
        else {
            for (int j = 0; j < 5; j++) {
                ASSERT_EQ(3, testVec.at(i).at(j).size());
            }
        }
    }

}


TEST(GetDataTests, GetNextTest1) {
    ObsGenerator g1(10, 0.02, 10, 270, 1, 3, 11);
    vector<int> start_sam = g1.getStartSamples();
    vector<int> end_sam = g1.getEndSamples();
    int current = 0;
    int max = 0;
    double current_time = 0.0;
    int sample = 0;
    while(g1.hasNext()) {
        ASSERT_NEAR(current_time, g1.currentTime(), 1e-6);
        ASSERT_EQ(sample, g1.currentSample());
        sample++;
        current_time = roundThisTo<double>(current_time + roundThisTo<double>((1.0/(double)270),10), 10);
        vector<vector<double> > data = g1.getNext(); 
        int count = 0;
        for (int i = 0; i < 10; i++) {
            if (current < end_sam[i] && current >= start_sam[i]) count++;
        }
        if (count>=max) max = count;
        current++;
        ASSERT_EQ(count, data.size());
    }
}

TEST(GetDataTests, GetNextTest2) {
    ObsGenerator g1(50, 2, 50, 270, 1, 3, 11);
    vector<int> start_sam = g1.getStartSamples();
    vector<int> end_sam = g1.getEndSamples();
    int current = 0;
    int max = 0;
    double current_time = 0.0;
    int sample = 0;
    while(g1.hasNext()) {
        ASSERT_NEAR(current_time, g1.currentTime(), 1e-6);
        ASSERT_EQ(sample, g1.currentSample());
        sample++;
        current_time = roundThisTo<double>(current_time + roundThisTo<double>((1.0/(double)270),10), 10);
        vector<vector<double> > data = g1.getNext(); 
        int count = 0;
        for (int i = 0; i < 50; i++) {
            if (current < end_sam[i] && current >= start_sam[i]) count++;
        }
        if (count>=max) max = count;
        current++;
        ASSERT_EQ(count, data.size());
    }
}

TEST(GetDataTests, GetNextTest3) {
    ObsGenerator g1(10, 0.02, 10, 300, 1, 3, 11);
    vector<int> start_sam = g1.getStartSamples();
    vector<int> end_sam = g1.getEndSamples();
    int current = 0;
    int max = 0;
    double current_time = 0.0;
    int sample = 0;
    while(g1.hasNext()) {
        ASSERT_NEAR(current_time, g1.currentTime(), 1e-6);
        ASSERT_EQ(sample, g1.currentSample());
        sample++;
        current_time = roundThisTo<double>(current_time + roundThisTo<double>((1.0/(double)300),10), 10);
        vector<vector<double> > data = g1.getNext(); 
        int count = 0;
        for (int i = 0; i < 10; i++) {
            if (current < end_sam[i] && current >= start_sam[i]) count++;
        }
        if (count>=max) max = count;
        current++;
        ASSERT_EQ(count, data.size());
    }
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}