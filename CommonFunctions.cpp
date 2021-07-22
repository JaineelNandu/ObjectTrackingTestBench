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
            product_row.push_back(sum);
        }
        product.push_back(product_row);
    }
    return product;
}

template <typename T>
std::vector<std::vector<T> >  matrixTranspose(std::vector<std::vector<T> > matrix) {
    std::vector<std::vector<T> > transposed;
    for (int col = 0; col < matrix[0].size(); col++) {
        std::vector<T> tr_row;
        for (int row = 0; row < matrix.size(); row++) {
            tr_row.push_back(matrix[row][col]);
        }
        transposed.push_back(tr_row);
    }
    return transposed;
}

template <typename T>
bool areEqualVectors(T v1, T v2) {
    return v1 == v2;
}

template <typename T>
bool areEqualVectors(std::vector<T> v1, T element) {
    return false;
}

template <typename T>
bool areEqualVectors(T element, std::vector<T> v2) {
    return false;
}

template <typename T>
bool areEqualVectors(std::vector<T> vec1, std::vector<T> vec2) {
    bool res = true;
    if (vec1.size() != vec2.size()) return false;
    for (int i = 0; i < vec1.size(); i++) {
        res = res && areEqualVectors(vec1[i], vec2[i]);
    }
    return res;
}

;
#endif