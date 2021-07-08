#include <iostream>
#include <vector>
#include "ObsGenerator.cpp"
#include <random>

using namespace std;

// A recursive function able to print a vector
// of an arbitrary amount of dimensions.
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

int main() {
    int num_obs = 5;            // 5 Total Objects
    double start_time = 2;       // in seconds
    double end_time = 10;        // in seconds
    int base_rate = 270;        // Hz (Since it is the LCM of 10, 30 & 27) (13.5*2 = 27)
    double max_jerk = 1.0;        // (m/s^3) Maximum Magnitude Jerk the object may experience.
    double max_acc = 3.0;         // (m/s^2) Maximum Magnitude of Acceleration the object may experience.
    double max_vel = 11;          // (m/s)   Maximum Magnitude of Velocity the object may experience.
    // To choose if we want to accept default values, if not limitations apply
    double sim_choice;
    cout << "\nDefault number objects (N): " << num_obs << ", default start time (ts): " << start_time << " (s), default end time (tf): " << end_time << "(s)";
    cout << "\nChoose 0 for default and 1 otherwise: ";
    cin >> sim_choice;
    if (sim_choice != 0)
    {
        do {
            cout << "\nNumber of obstacles must be greater than 0.\nEnter number of obstacles: ";
            cin >> num_obs;
        }
        while (num_obs <=0);
        do {
            cout << "\nStart time must be greater than 0.\nEnter start time (s): ";
            cin >> start_time;
        }
        while (start_time <= 0);
        do {
            cout << "\nEnd time must be greater than start time.\nEnter end time (s): ";
            cin >> end_time;
        }
        while (end_time < start_time);
        }
    cout << "N = " << num_obs << "\nts = " << start_time << "\ntf = " << end_time << endl;
    return 0;
}
