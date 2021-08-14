#include <iostream>
#include <vector>
#include "ObsGenerator.cpp"
#include <random>
#include "CommonFunctions.cpp"
#include <fstream>
#include "SensorType.hpp"
#include "Sensor.cpp"
#include "Obstacle.cpp"
#include <bits/stdc++.h>
#include "KFHandler.cpp"
#include "Associator.cpp"

using namespace std;

int main()
{
  int num_obs = 5;       // 5 Total Objects
  double start_time = 2; // in seconds
  double end_time = 10;  // in seconds
  int base_rate = 270;   // Hz (Since it is the LCM of 10, 30 & 27) (13.5*2 = 27)
  double max_jerk = 1.0; // (m/s^3) Maximum Magnitude Jerk the object may experience.
  double max_acc = 3.0;  // (m/s^2) Maximum Magnitude of Acceleration the object may experience.
  double max_vel = 11;   // (m/s)   Maximum Magnitude of Velocity the object may experience.
  ofstream obsfile("ObstacleData_1000_0P1_100.csv");
  ofstream sensorObstaclePointfile("SensorObstaclePointData_1000_0P1_100.csv");
  ofstream sensorObstacleParamfile("SensorObstacleParamData_1000_0P1_100.csv");
  ofstream kfHandlerFile("KFHandlerData_1000_0P1_100.csv");
  ofstream associatorFile("AssociatorFile_1000_0P1_100.csv");
  // To choose if we want to accept default values, if not limitations apply
  double sim_choice;
  cout << "\nDefault number objects (N): " << num_obs << ", default start time (ts): " << start_time << " (s), default end time (tf): " << end_time << "(s)";
  cout << "\nChoose 0 for default and 1 otherwise: ";
  cin >> sim_choice;
  if (sim_choice != 0)
  {
    do
    {
      cout << "\nNumber of obstacles must be greater than 0.\nEnter number of obstacles: ";
      cin >> num_obs;
    } while (num_obs <= 0);
    do
    {
      cout << "\nStart time must be greater than 0.\nEnter start time (s): ";
      cin >> start_time;
    } while (start_time <= 0);
    do
    {
      cout << "\nEnd time must be greater than start time.\nEnter end time (s): ";
      cin >> end_time;
    } while (end_time < start_time);
  }
  cout << "N = " << num_obs << "\nts = " << start_time << "\ntf = " << end_time << endl;
  cout << endl;
  ObsGenerator obstacle_generator(num_obs, start_time, end_time, base_rate, max_jerk, max_acc, max_vel);
  double sensor_sigmaX = 1;       // meters
  double sensor_sigmaY = 0.3;     // meters
  double sensor_sigmaZ = 0.15;    // meters
  double sensor_azimuth = 270;    // degrees
  double sensor_elevation = 40;   // degrees
  double sensor_min_r = 3;        // meters
  double sensor_max_r = 100;      // meters
  vector<vector<double> > chctm = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
  vector<double> fov_params = {sensor_azimuth, sensor_elevation, sensor_min_r, sensor_max_r};
  //vector<vector<double> > chctm = invertCHCTM(getRotationY<double>(-90));
  //vector<vector<double> > chctm = invertCHCTM(matrixMultiply(matrixMultiply(getTranslation<double>({1, 0, 2}), getRotationX<double>(90)), getRotationY<double>(30)));
  //Sensor lidar2({270, 40, 3, 150}, {sensor_sigmaX, sensor_sigmaY, sensor_sigmaZ}, SensorType::Lidar2, 10, chctm); 
  Sensor lidar2(fov_params, {sensor_sigmaX, sensor_sigmaY, sensor_sigmaZ}, SensorType::Lidar2); // Default rate = 10 Hz and CHCTM = eye(4)
  obsfile << num_obs << endl;
  sensorObstaclePointfile << num_obs << "," << sensor_sigmaX << "," << sensor_sigmaY << "," << sensor_sigmaZ << endl;
  sensorObstacleParamfile << num_obs << "," << sensor_azimuth << "," << sensor_elevation << "," << sensor_min_r << "," << sensor_max_r << ",";
  for (int i = 0; i < chctm.size(); i++) {
    for (int j = 0; j < chctm[i].size(); j++) {
      sensorObstacleParamfile << chctm[i][j] << ",";
    }
  }
  double filter_capacity_to_obstacle_ratio = 0.5; // Num of obstacles that can be tracked divided by num of total obstacles that are generated.
  sensorObstacleParamfile << endl;
  int handler_size = (int)(filter_capacity_to_obstacle_ratio*(double)num_obs);
  KFHandler kf_handler(handler_size);
  KFHandler for_associator(handler_size);
  Associator associator({sensor_sigmaX, sensor_sigmaY, sensor_sigmaZ}, invertCHCTM(chctm));
  kfHandlerFile << num_obs << "," <<  handler_size << endl;
  while (obstacle_generator.hasNext())
  { 
    int current_sample = obstacle_generator.currentSample();
    double current_time = obstacle_generator.currentTime();
    if (current_sample >= base_rate && current_sample%base_rate == 0) cout << "Time: " << current_time << endl;
    obsfile << current_sample << "," << current_time << ",";
    vector<int> active = obstacle_generator.listOfActiveObstacles();
    vector<vector<double>> generated_data = obstacle_generator.getNext();
    int num_active = active.size();
    obsfile << num_active << ",";
    for (int act = 0; act < num_active; act++)
    {
      obsfile << active[act] << ",";
    }
    for (int act = 0; act < num_active; act++)
    {
      for (int axis = 0; axis < 3; axis++)
      {
        obsfile << generated_data[act][axis] << ",";
      }
    }
    obsfile << endl;
    if (lidar2.isUpdateSample(base_rate, current_sample)) {
      lidar2.passThrough(active, generated_data);
      vector<vector<double> > passed_truth = lidar2.getPassedTruth();
      vector<vector<double> > passed_noisy = lidar2.getPassed();
      vector<int> passed_active = lidar2.getActivePassed();
      int num_passed = passed_active.size();
      sensorObstaclePointfile << current_sample << "," <<current_time << ",";
      sensorObstacleParamfile << current_sample << "," <<current_time << ",";
      sensorObstaclePointfile << num_passed << ",";
      for (int ps = 0; ps < num_passed; ps++) {
        sensorObstaclePointfile << passed_active[ps] <<",";
      }
      for (int ps = 0; ps < num_passed; ps++) {
        for (int axis = 0; axis < 3; axis ++) {
          sensorObstaclePointfile << passed_noisy[ps][axis] << ",";
        }
        for (int axis = 0; axis < 3; axis++) {
          sensorObstaclePointfile << passed_truth[ps][axis] << ",";
        }
      }
      sensorObstacleParamfile << num_active << ",";
      for (int act = 0; act < num_active; act++)
      {
        sensorObstacleParamfile << active[act] << ",";
      }
      for (int n = 0; n < num_active; n++) {
        vector<vector<double> > pt_inertial = {generated_data[n]};
        pt_inertial[0].push_back(1);
        pt_inertial = matrixTranspose(pt_inertial);
        vector<vector<double> > pt_sensor = matrixMultiply(lidar2.getCHCTM(), pt_inertial);
        pt_sensor = matrixTranspose(pt_sensor);
        vector<double> obs_params = lidar2.getObstacleParametes(pt_sensor[0]);
        for (int pr = 0; pr < 3; pr++) {
          sensorObstacleParamfile << obs_params[pr] << ",";
        }
      }
      // Temporary to test KF Handler plots
      vector<Obstacle> passed_obstacles = lidar2.pointToObstacle(current_time);
      for (int obs = 0; obs < passed_obstacles.size(); obs++) {
        passed_obstacles[obs].setObstacleID(passed_active[obs]);
      }
      sortWithRespectToFirst(passed_obstacles, passed_truth);
      kf_handler.track(passed_obstacles, passed_truth);
      kfHandlerFile << current_sample << "," << current_time << ","; 
      vector<Obstacle> tracked_obstacles = kf_handler.getCurrentlyTracked();
      kfHandlerFile << passed_obstacles.size() << ",";
      kfHandlerFile << tracked_obstacles.size() << ",";
      for (int tr = 0; tr < tracked_obstacles.size(); tr++) {
        kfHandlerFile << tracked_obstacles[tr].getID() << ",";
      }
      for (int tr = 0; tr < tracked_obstacles.size(); tr++) {
        vector<double> tr_position = tracked_obstacles[tr].getPosition();
        vector<int> instances = tracked_obstacles[tr].getKFInstances();
        for (int axis = 0; axis < 3; axis++) {
          kfHandlerFile << tr_position[axis] << ",";
        }
        for (int axis = 0; axis < 3; axis++) {
          kfHandlerFile << instances[axis] << ",";
        }
      }
      kfHandlerFile << endl;
      // end of temporary
      sensorObstaclePointfile << endl;
      sensorObstacleParamfile << endl;
      //Testing KF Handler with associator.
      passed_obstacles = lidar2.pointToObstacle(current_time);
      passed_truth = lidar2.getPassedTruth();
      vector<Obstacle> currently_tracked = for_associator.getCurrentlyTracked();
      associator.associate(currently_tracked, passed_obstacles, passed_truth, current_time);
      vector<Obstacle> associated = associator.getAssociated();
      vector<vector<double> > associated_truth = associator.getAssociatedTruth();
      for_associator.track(associated, associated_truth);
      associatorFile << current_sample << "," << current_time << ",";
      currently_tracked = for_associator.getCurrentlyTracked();
      associatorFile << passed_obstacles.size() << ",";
      associatorFile << currently_tracked.size() << ",";
      for (int tr = 0; tr <  currently_tracked.size(); tr++) {
        associatorFile << currently_tracked[tr].getID() << ",";
      } 
      for (int tr = 0; tr < currently_tracked.size(); tr++) {
        vector<double> tr_position = currently_tracked[tr].getPosition();
        vector<int> instances = currently_tracked[tr].getKFInstances();
        for (int axis = 0; axis < 3; axis++) {
          associatorFile << tr_position[axis] << ",";
        }
        for (int axis = 0; axis < 3; axis++) {
          associatorFile << instances[axis] << ",";
        }
      }
      associatorFile << endl;
    }
  }
  associatorFile << associator.getNewID() -1 << "," << handler_size <<endl;
  obsfile.close();
  sensorObstaclePointfile.close();
  sensorObstacleParamfile.close();
  kfHandlerFile.close();
  associatorFile.close();
  return 0;
}
