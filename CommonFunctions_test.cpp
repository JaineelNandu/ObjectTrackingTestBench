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

TEST(MatrixOperationsTest, MatrixMultiplyTest1) {
    vector<vector<int> > A = {{1, 2, 3}, {4, 5, 6}};
    vector<vector<int> > B = {{1}, {2}, {3}};
    vector<vector<int> > product = matrixMultiply(A, B);
    vector<vector<int> > truth = {{14}, {32}};
    ASSERT_TRUE(areEqualVectors(product, truth));
}

TEST(MatrixOperationsTest, MatrixMultiplyTest2) {
    vector<vector<int> > A = {{1, 2, 3}, {4, 5, 6}};
    vector<vector<int> > B = {{1, 2}, {2, 3}, {3, 4}};
    vector<vector<int> > product = matrixMultiply(A, B);
    vector<vector<int> > truth = {{14, 20}, {32, 47}};
    ASSERT_TRUE(areEqualVectors(product, truth));
}

TEST(MatrixOperationsTest, MatrixMultiplyTest3) {
    vector<vector<int> > A = {{1, 2, 3, 4}, {-1, -5, 10, 0}, {1, 5, 9, 20}, {9, 8, 5, 6}};
    vector<vector<int> > B = {{1}, {2}, {3}, {1}};
    vector<vector<int> > product = matrixMultiply(A, B);
    vector<vector<int> > truth = {{18}, {19}, {58}, {46}};
    ASSERT_TRUE(areEqualVectors(product, truth));
}

TEST(MatrixOperationsTest, matrixTransposeTest1) {
    vector<vector<int> > A = {{1, 2, 3, 4}, {-1, -5, 10, 0}, {1, 5, 9, 20}, {9, 8, 5, 6}};
    vector<vector<int> > transposed = matrixTranspose(A);
    vector<vector<int> > truth = {{1, -1, 1, 9}, {2, -5, 5, 8}, {3, 10, 9, 5}, {4, 0, 20, 6}};
    ASSERT_TRUE(areEqualVectors(transposed, truth));
} 

TEST(MatrixOperationsTest, matrixTransposeTest2) {
    vector<vector<int> > A = {{1}, {2}, {3}, {1}};
    vector<vector<int> > transposed = matrixTranspose(A);
    vector<vector<int> > truth = {{1, 2, 3, 1}};
    ASSERT_TRUE(areEqualVectors(transposed, truth));
} 

TEST(MatrixOperationsTest, matrixTransposeTest3) {
    vector<vector<int> > A = {{1, 2, 3, 1}};
    vector<vector<int> > transposed = matrixTranspose(A);
    vector<vector<int> > truth = {{1}, {2}, {3}, {1}};
    ASSERT_TRUE(areEqualVectors(transposed, truth));
}

TEST(CHCTMOperationsTest, InvertCHCTMTest1) {
    // Rotated 45 degrees about X axis
    vector<vector<double> > chctm = { { 1, 0, 0, 0 }, 
                                    { 0, 0.7071, -0.7071, 0 },
                                    { 0, 0.7071, 0.7071, 0 },
                                    { 0, 0, 0, 1 } };
    vector<vector<double> > inv = invertCHCTM(chctm);
    vector<vector<double> > truth = { { 1, 0, 0, 0 }, 
                                    { 0, 0.7071, 0.7071, 0 },
                                    { 0, -0.7071, 0.7071, 0 },
                                    { 0, 0, 0, 1 } };
    ASSERT_TRUE(areEqualVectors(inv, truth));
    vector<vector<double> > pf = matrixMultiply(inv, {{1},{1},{0},{1}});
    vector<vector<double> > true_pf = {{1}, {0.7071}, {-0.7071}, {1}};
    ASSERT_TRUE(areEqualVectors(pf, true_pf));
}

TEST(TransformationMatricesTest, GetTranslationMatrix) {
    vector<vector<double> > trans = getTranslation<double>({1.0, 2.0, 3.0});
    vector<vector<double> > truth = {{1, 0, 0, 1}, {0, 1, 0, 2}, {0, 0, 1, 3}, {0, 0, 0, 1}};
    ASSERT_TRUE(areEqualVectors(trans, truth));
}

TEST(TransformationMatricesTest, GetRotationX) {
    vector<vector<double> > truth = { { 1.0000, 0, 0, 0 }, 
                                        { 0, -0.5000, -0.8660, 0 },
                                        { 0, 0.8660, -0.5000, 0 },
                                        { 0, 0, 0, 1.0000 } };
    vector<vector<double> > rotX = getRotationX<double>(120);
    ASSERT_TRUE(areEqualVectors(rotX, truth));
}

TEST(TransformationMatricesTest, GetRotationY) {
    vector<vector<double> > truth = { { 0.3420, 0, -0.9397, 0 },
                                        { 0, 1.0000, 0, 0 },
                                        { 0.9397, 0, 0.3420, 0 },
                                        { 0, 0, 0, 1.0000 } };
    vector<vector<double> > rotY = getRotationY<double>(-70);
    ASSERT_TRUE(areEqualVectors(rotY, truth));
}

TEST(TransformationMatricesTest, GetRotationZ) {
    vector<vector<double> > truth = { { 0.8660, -0.5000, 0, 0 },
                                        { 0.5000, 0.8660, 0, 0 },
                                        { 0, 0, 1.0000, 0 },
                                        { 0, 0, 0, 1.0000 } };
    vector<vector<double> > rotZ = getRotationZ<double>(30);
    ASSERT_TRUE(areEqualVectors(rotZ, truth));
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}