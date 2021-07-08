#include "ObsGenerator.cpp"
#include<bits/stdc++.h>
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
    ASSERT_DOUBLE_EQ(round_to((double)1/((double)300), 9), t[1]);
    ASSERT_DOUBLE_EQ(round_to((double)((int)(2.2*300)-1)*(double)(1/((double)300)), 9), t[t.size()-1]);
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

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}