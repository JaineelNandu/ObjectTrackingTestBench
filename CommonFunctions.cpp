#ifndef COMMON_CPP
#define COMMON_CPP
#include <math.h>
#include <vector>

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
	if((x * pow(10, n + 2)) - (floor(x * pow(10, n+1))) > 4) d = 1; 
	x = (floor(x * pow(10, n+1)) + d) / pow(10, n+1); 
	return x; 
}

/**
 * Calculates Magnitude i.e. l2norm of the vector passed. 
**/
template<typename T>
T vectorMagnitude(std::vector<T> vec) {
    double sum = 0;
    for(int i = 0; i < vec.size(); i++) {
        sum += vec.at(i)*vec.at(i);
    }
    return round_to<T>(sqrt(sum), 6);
}

/**
 * Updates the magnitude to max if it is higher.
**/
template<typename T>
std::vector<T> checkAndClipMax(T target, std::vector<T> quantity) {
    std::vector<T> retQuant;
    T magnitude = vectorMagnitude(quantity);
    if (magnitude <= target) {
        retQuant = quantity;
    }
    else {
        for(int i = 0; i < quantity.size(); i++) {
            retQuant.push_back(round_to<T>(((target/magnitude)*quantity.at(i)), 6));
        }
    } 
    return retQuant;
}

;
#endif