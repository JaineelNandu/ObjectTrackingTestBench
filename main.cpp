#include <iostream>
#include <vector>
#include "ObsGenerator.cpp"
#include <random>
#include "CommonFunctions.cpp"
#include <fstream>

using namespace std;


int main() {
    int num_obs = 5;            // 5 Total Objects
    double start_time = 2;       // in seconds
    double end_time = 10;        // in seconds
    int base_rate = 270;        // Hz (Since it is the LCM of 10, 30 & 27) (13.5*2 = 27)
    double max_jerk = 1.0;        // (m/s^3) Maximum Magnitude Jerk the object may experience.
    double max_acc = 3.0;         // (m/s^2) Maximum Magnitude of Acceleration the object may experience.
    double max_vel = 11;          // (m/s)   Maximum Magnitude of Velocity the object may experience.
    ofstream obsfile("ObstacleData.csv");
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
    cout << endl;
    ObsGenerator obstacle_generator(num_obs, start_time, end_time, base_rate, max_jerk, max_acc, max_vel);
    while (obstacle_generator.hasNext()) {
      obsfile << obstacle_generator.currentSample() << "," << obstacle_generator.currentTime() << ",";
      vector<int> active = obstacle_generator.listOfActiveObstacles();
      vector<vector<double> > generated_data = obstacle_generator.getNext();
      int num_active = active.size();
      obsfile << num_active << ",";
      for (int act = 0; act < num_active; act++) {
        obsfile << active[act] << ",";
      }
      for (int act = 0; act < num_active; act++) {
        for (int axis = 0; axis < 3; axis++) {
          obsfile << generated_data[act][axis] << ",";
        }
      }
      obsfile << endl;
    }
    /*
    while(obstacle_generator.hasNext()) {
      cout << "\nTime : " << obstacle_generator.currentTime()<< "\tSample : " <<obstacle_generator.currentSample() <<"\t";
      vector<int> active = obstacle_generator.activeObjects();
      vector<vector<double> > generated_data = obstacle_generator.getNext();
      cout << "Num Active : " << active.size() << "\t";
      if (generated_data.size() > 0) {
        print_vec(active);
        cout << "\t\t";
        //print_vec(generated_data);
      }
      else cout << "No Object" ;
    }
    cout << "\nStart Samples : ";
    print_vec(obstacle_generator.getStartSamples());
    cout << "\nEnd Samples : ";
    print_vec(obstacle_generator.getEndSamples());
    */
    obsfile.close();
    return 0;
}
