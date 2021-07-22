#include <bits/stdc++.h>
#include <gtest/gtest.h>
#include "CommonFunctions.cpp"
#include <vector>

using namespace std;

TEST(VectorEqualTest, NonEqualDimTest) {
    vector<vector<int> > v1 = {{1}, {1, 2}};
    vector<int> v2 = {1, 2};
    ASSERT_FALSE(areEqualVectors(v1,v2));
}

TEST(VectorEqualTest, NonEqualDimTest2) {
    vector<int> v1 = {1, 2, 3, 4};
    vector<int> v2 = {1, 2};
    ASSERT_FALSE(areEqualVectors(v1,v2));
}

TEST(VectorEqualTest, EqualDim1ElementNotEqualTest) {
    vector<int> v1 = {1, 2, 3, 4};
    vector<int> v2 = {1, 2, 5, 4};
    ASSERT_FALSE(areEqualVectors(v1,v2));
}

TEST(VectorEqualTest, EqualDimAllEqualTest) {
    vector<int> v1 = {1, 2, 3, 4};
    vector<int> v2 = {1, 2, 3, 4};
    ASSERT_TRUE(areEqualVectors(v1,v2));
}

TEST(VectorEqualTest, 2DAllEqualTest) {
    vector<vector<int> > v1 = {{1, 2, 3, 4}, {6, 7, 8}};
    vector<vector<int> > v2 = {{1, 2, 3, 4}, {6, 7, 8}};
    ASSERT_TRUE(areEqualVectors(v1,v2));
}

TEST(VectorEqualTest, 2DNotEqualTest) {
    vector<vector<int> > v1 = {{1, 2, 3, 4}, {6, 7, 8}};
    vector<vector<int> > v2 = {{1, 2, 3, 9}, {6, 7, 8}};
    ASSERT_FALSE(areEqualVectors(v1,v2));
}

TEST(VectorEqualTest, 3DAllEqualTest) {
    vector<vector<vector<int> > > v1 = {{{3, 4, 5, 6}, {1, 2, 3, 4}}, {{1, 2, 3, 4}, {6, 7, 8}}};
    vector<vector<vector<int> > > v2 = {{{3, 4, 5, 6}, {1, 2, 3, 4}}, {{1, 2, 3, 4}, {6, 7, 8}}};
    ASSERT_TRUE(areEqualVectors(v1,v2));
}

TEST(VectorEqualTest, 3DNotEqualTest) {
    vector<vector<vector<int> > > v1 = {{{3, 4, 5, 6}, {1, 2, 3, 4}}, {{1, 2, 3, 4}, {6, 7, 8}}};
    vector<vector<vector<int> > > v2 = {{{3, 4, 5, 6}, {1, 2, 3, 9}}, {{1, 2, 3, 4}, {6, 7, 8}}};
    ASSERT_FALSE(areEqualVectors(v1,v2));
}

TEST(MatrixOperationsTest, DISABLED_MatrixMultiplyTest) {

}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}