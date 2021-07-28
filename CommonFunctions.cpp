/**
 *      Author   :   Jaineel Nandu
 *      email    :   nandujaineel@vt.edu
 *                   jaineel.nandu02@gmail.com
 **/

#ifndef COMMON_CPP
#define COMMON_CPP
#include <math.h>
#include <vector>
#define PI_6 3.141592


/**
 * Prints a single element of the vector.
 **/
template <typename T>
static void printVector(T vec)
{
    std::cout << vec;
}

/**
 * Prints vector.
 **/
template <typename T>
static void printVector(std::vector<T> vec)
{
    int size = vec.size();
    if (size <= 0)
    {
        std::cout << "invalid vector";
        return;
    }
    std::cout << '{';
    for (int l = 0; l < size - 1; l++)
    {
        printVector(vec[l]);
        std::cout << ',';
    }
    printVector(vec[size - 1]);
    std::cout << '}';
}

/**
 * Rounds This number To decimal places.
 **/
template <typename T>
static T roundThisTo(T number, int decimal_places)
{
    int d = 0;
    if ((number * pow(10, decimal_places + 2)) - (floor(number * pow(10, decimal_places + 1))) > 4)
        d = 1;
    number = (floor(number * pow(10, decimal_places + 1)) + d) / pow(10, decimal_places + 1);
    return number;
}

template <typename T>
inline T sind(T x) {
    return roundThisTo<T>(std::sin(x*PI_6/180), 6);
}

template <typename T>
inline T cosd(T x) {
    return roundThisTo<T>(std::cos(x*PI_6/180), 6);
}

/**
 * Calculates Magnitude i.e. l2norm of the vector passed. 
 **/
template <typename T>
T vectorMagnitude(std::vector<T> vec)
{
    double sum = 0;
    for (int i = 0; i < vec.size(); i++)
    {
        sum += vec.at(i) * vec.at(i);
    }
    return roundThisTo<T>(sqrt(sum), 6);
}

/**
 * Updates the magnitude to max if it is higher.
 **/
template <typename T>
std::vector<T> checkMagnitudeAndClipToMax(T target, std::vector<T> quantity)
{
    std::vector<T> retQuant;
    T magnitude = vectorMagnitude(quantity);
    if (magnitude <= target)
    {
        retQuant = quantity;
    }
    else
    {
        for (int i = 0; i < quantity.size(); i++)
        {
            retQuant.push_back(roundThisTo<T>(((target / magnitude) * quantity.at(i)), 6));
        }
    }
    return retQuant;
}

/**
 * Calculates Matrix multiplication of two matrices. (Assumes multiplication is valid)
 **/
template <typename T>
std::vector<std::vector<T>> matrixMultiply(std::vector<std::vector<T>> matrix1, std::vector<std::vector<T>> matrix2)
{
    std::vector<std::vector<T>> product;
    for (int i = 0; i < matrix1.size(); i++)
    {
        std::vector<T> product_row;
        for (int j = 0; j < matrix2[0].size(); j++)
        {
            T sum = 0;
            for (int k = 0; k < matrix2.size(); k++)
            {
                sum += matrix1[i][k] * matrix2[k][j];
            }
            product_row.push_back(roundThisTo<T>(sum, 6));
        }
        product.push_back(product_row);
    }
    return product;
}

template <typename T>
std::vector<std::vector<T>> matrixTranspose(std::vector<std::vector<T>> matrix)
{
    std::vector<std::vector<T>> transposed;
    for (int col = 0; col < matrix[0].size(); col++)
    {
        std::vector<T> tr_row;
        for (int row = 0; row < matrix.size(); row++)
        {
            tr_row.push_back(matrix[row][col]);
        }
        transposed.push_back(tr_row);
    }
    return transposed;
}

template <typename T>
bool areEqualVectors(T v1, T v2, double tol = 1e-4)
{
    return v1 == v2 || ((v1 - v2 < tol) && (v1 - v2 > -tol));
}

template <typename T>
bool areEqualVectors(std::vector<T> v1, T element, double tol = 1e-4)
{
    return false;
}

template <typename T>
bool areEqualVectors(T element, std::vector<T> v2, double tol = 1e-4)
{
    return false;
}

template <typename T>
bool areEqualVectors(std::vector<T> vec1, std::vector<T> vec2, double tol = 1e-4)
{
    bool res = true;
    if (vec1.size() != vec2.size())
        return false;
    for (int i = 0; i < vec1.size(); i++)
    {
        res = res && areEqualVectors(vec1[i], vec2[i], tol);
    }
    return res;
}

template<typename T>
std::vector<std::vector<T> > invertCHCTM(std::vector<std::vector<T> > chctm) {
    std::vector<std::vector<T> > inverted;
    std::vector<std::vector<T> > R;
    std::vector<std::vector<T> > d;
    for (int row = 0; row < 3; row++) {
        std::vector<T> R_row;
        std::vector<T> d_row;
        for (int col = 0; col < 3; col++) {
            R_row.push_back(chctm[row][col]);
        }
        d_row.push_back(chctm[row][3]);
        R.push_back(R_row);
        d.push_back(d_row);
    }
    std::vector<std::vector<T> > RT = matrixTranspose(R);
    std::vector<std::vector<T> > RTd = matrixMultiply(RT, d);
    for (int row = 0; row < 3; row++) {
        std::vector<T> inv_row;
        for (int col = 0; col < 3; col++) {
            inv_row.push_back(RT[row][col]);
        }
        inv_row.push_back(-RTd[row][0]);
        inverted.push_back(inv_row);
    }
    inverted.push_back({0, 0, 0, 1});
    return inverted;
}

template <typename T>
std::vector<std::vector<T> > getTranslation(std::vector<T> translate) {
    std::vector<std::vector<T> > translation;
    translation.push_back({1, 0, 0, translate[0]});
    translation.push_back({0, 1, 0, translate[1]});
    translation.push_back({0, 0, 1, translate[2]});
    translation.push_back({0, 0, 0, 1});
    return translation;
}

template <typename T>
std::vector<std::vector<T> > getRotationX(T degrees) {
    T cd = cosd(degrees);
    T sd = sind(degrees);
    std::vector<std::vector<T> > rotation;
    rotation.push_back({1, 0, 0, 0});
    rotation.push_back({0, cd, -sd, 0});
    rotation.push_back({0, sd, cd, 0});
    rotation.push_back({0, 0, 0, 1});
    return rotation;
}

template <typename T>
std::vector<std::vector<T> > getRotationY(T degrees) {
    T cd = cosd(degrees);
    T sd = sind(degrees);
    std::vector<std::vector<T> > rotation;
    rotation.push_back({cd, 0, sd, 0});
    rotation.push_back({0, 1, 0, 0});
    rotation.push_back({-sd, 0, cd, 0});
    rotation.push_back({0, 0, 0, 1});
    return rotation;
}

template <typename T>
std::vector<std::vector<T> > getRotationZ(T degrees) {
    T cd = cosd(degrees);
    T sd = sind(degrees);
    std::vector<std::vector<T> > rotation;
    rotation.push_back({cd, -sd, 0, 0});
    rotation.push_back({sd, cd, 0, 0});
    rotation.push_back({0, 0, 1, 0});
    rotation.push_back({0, 0, 0, 1});
    return rotation;
}

template <typename T>
T vectorAdd(T v1, T v2) {
    return v1 + v2;
}

template <typename T>
std::vector<T> vectorAdd(std::vector<T> v1, std::vector<T> v2) {
    std::vector<T> addition;
    for (int i = 0; i < v1.size(); i++) {
        addition.push_back(vectorAdd(v1[i], v2[i]));
    }
    return addition;
}

template <typename T, typename E>
void sortWithRespectToFirst(std::vector<T> &f, std::vector<E> &s) {
    std::vector<std::pair<T, E> > pairVector;
    for (int i = 0; i < f.size(); i++) {
        pairVector.push_back(make_pair(f[i], s[i]));
    }
    sort(pairVector.begin(), pairVector.end());
    for(int i = 0; i < f.size(); i++) {
        f[i] = pairVector[i].first;
        s[i] = pairVector[i].second;
    }
}

;
#endif