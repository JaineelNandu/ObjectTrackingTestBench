#ifndef COMMON_CPP
#define COMMON_CPP
#include <math.h>
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
;
#endif