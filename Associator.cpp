#ifndef ASSOCIATOR_CPP
#define ASSOCIATOR_CPP

#include <vector>
#include "Sensor.cpp"
#include "CommonFunctions.cpp"
#include <bits/stdc++.h>
#include <math.h>

using namespace std;


class Associator
{
private:
    int new_ID = 1;
    vector<vector<double> > cov_sensor;
    vector<vector<double> > cov_inertial;
    vector<vector<double> > inv_cov_inertial;
    vector<Obstacle> associated;
    vector<vector<double> > associated_truth;
    double axis_vel_var = 0;
    double axis_acc_var = 0;

public:
    Associator(vector<double> noise_std, vector<vector<double> > chctm_fixed_mobile, double max_vel = 11, double max_acc = 3) {
        vector<vector<double> > chctm_rot;
        for (int row = 0; row < 3; row++) {
            vector<double> row_vector;
            for (int col = 0; col < 3; col++) row_vector.push_back(chctm_fixed_mobile[row][col]);
            chctm_rot.push_back(row_vector);
        }
        double vel_var = roundThisTo(pow(2*max_vel, 2)/12, 6);
        double acc_var = roundThisTo(pow(2*max_acc, 2)/12, 6);
        axis_vel_var = roundThisTo(vel_var/3, 6);
        axis_acc_var = roundThisTo(acc_var/3, 6);
        cov_sensor.push_back({pow(noise_std[0],2), 0, 0});
        cov_sensor.push_back({0, pow(noise_std[1],2), 0});
        cov_sensor.push_back({0, 0, pow(noise_std[2],2)});
        cov_inertial = matrixMultiply(matrixMultiply(chctm_rot, cov_sensor), matrixTranspose(chctm_rot));
        inv_cov_inertial = matrix3Inverse(cov_inertial);
    }

    vector<Obstacle> getAssociated() {
        return associated;
    }

    vector<vector<double> > getAssociatedTruth() {
        return associated_truth;
    }

    void associate(vector<Obstacle> tracked, vector<Obstacle> passed_obstacles, vector<vector<double> > passed_truth,const double &time) {
        if (tracked.size() == 0 && passed_obstacles.size() == 0) {
            associated = passed_obstacles;
            associated_truth = passed_truth;
            return;
        }
        else if (tracked.size() == 0 && passed_obstacles.size() != 0) {
            sortWithRespectToFirst(passed_obstacles, passed_truth);
            for (int i = 0; i < passed_obstacles.size(); i++) {
                assignNewID(passed_obstacles[i]);
            }
            associated = passed_obstacles;
            associated_truth = passed_truth;
            return;
        }
        else if (tracked.size() != 0 && passed_obstacles.size() == 0) {
            double old_time = tracked[0].getTimeStamp();
            predict(tracked, time);
            sort(tracked.begin(), tracked.end());
            vector<Obstacle> follow_out_FOV;
            vector<vector<double> > follow_truth;
            for (int i = 0; i < tracked.size(); i++) {
                tracked[i].decrementFOVCounter();
                if (tracked[i].getOutFOVCounter() != 0) {
                    follow_out_FOV.push_back(tracked[i]);
                    follow_truth.push_back(tracked[i].getPosition());
                }
                associated = follow_out_FOV;
                associated_truth = follow_truth;
            }
        }
        else {
            double old_time = tracked[0].getTimeStamp();
            double time_difference = time - old_time;
            predict(tracked, time);
            sort(tracked.begin(), tracked.end());
            sortWithRespectToFirst(passed_obstacles, passed_truth);
            vector<vector<int> > associationList = getAssociationLists(tracked, passed_obstacles, time_difference);
            vector<Obstacle> ret;
            vector<vector<double> > ret_truth;
            for (int tr = 0; tr < associationList[0].size(); tr++) {
                if (associationList[0][tr] == -1) {     // -1 is unassigned
                    tracked[tr].decrementFOVCounter();
                    if(tracked[tr].getOutFOVCounter() != 0) {
                        ret.push_back(tracked[tr]);
                        ret_truth.push_back(tracked[tr].getPosition());
                    }
                }
                else {
                    tracked[tr].setPosition(passed_obstacles[associationList[0][tr]].getPosition());
                    tracked[tr].resetFOVCounter();
                    ret.push_back(tracked[tr]);
                    ret_truth.push_back(passed_truth[associationList[0][tr]]);
                }
            }
            for (int po = 0; po < associationList[1].size(); po++) {
                if (associationList[1][po] == -1) {     // -1 is unassigned
                    assignNewID(passed_obstacles[po]);
                    ret.push_back(passed_obstacles[po]);
                    ret_truth.push_back(passed_truth[po]);
                }
            }
            sortWithRespectToFirst(ret, ret_truth);
            //sort(ret.begin(), ret.end());
            associated = ret;
            associated_truth = ret_truth;
        }
    }

    vector<vector<int> > getAssociationLists(vector<Obstacle> a1,vector<Obstacle> a2, double td) {
        vector<vector<int> > retMat;
        vector<int> a1_assign(a1.size(), -1);
        vector<int> a2_assign(a2.size(), -1);
        vector<vector<double> > mahalanobisMatrix = getMahalanobisMatrix(a1, a2, td);
        for (int row = 0; row < a1_assign.size(); row++) {
            vector<int> inGate;
            for (int col = 0; col < a2_assign.size(); col++) {
                if (mahalanobisMatrix[row][col] < 10) inGate.push_back(col); // 3 is the maximum mahalanobis distance to be tolerated.
            }
            if (inGate.size() != 0) {
                int min_idx = findMinIndex(inGate);
                if (a2_assign[inGate[min_idx]] == -1) {
                    a1_assign[row] = inGate[min_idx];
                    a2_assign[inGate[min_idx]] = row;
                }  
            }
        }
        /**
         * for row
         *  vec inGate 
         *  for col 
         *      inGate push col if < 3
         *  if inGate.size != 0 {
         *          find min index
         *          if a2[inGate[min_idx] == -1] 
         *              a1[row] = inGate[min_idx]
         *              a2[inGate[min_idx]] = row  
         *  }
         *      
         * 
         * */
        retMat.push_back(a1_assign);
        retMat.push_back(a2_assign);
        return retMat;
    }

    void assignNewID(Obstacle &ob) {
        ob.setObstacleID(new_ID);
        new_ID++;
    }

    void predict(vector<Obstacle> &obstacles, const double &t) {
        for (int i = 0; i < obstacles.size(); i++) {
            obstacles[i].predict(t);
        }
    }

    vector<vector<double> > getMahalanobisMatrix(vector<Obstacle> r, vector<Obstacle> c, double td) {
        vector<vector<double> > mahalanobis;
        for (int row = 0; row < r.size(); row++) {
            vector<double> rowMahalanobis;
            for (int col = 0; col < c.size(); col++) {
                rowMahalanobis.push_back(calculateMahalanobis(r[row].getPosition(), c[col].getPosition(), td));
            }
            mahalanobis.push_back(rowMahalanobis);
        }
        return mahalanobis;
    }

    double calculateMahalanobis (vector<double> p1, vector<double> p2, double td) {
        vector<vector<double> > pT;
        pT.push_back(vectorSubtract(p1,p2));
        vector<vector<double> > cov_V = vectorAdd(cov_inertial, {{(axis_vel_var+axis_acc_var*td*td)*td*td, 0, 0}, {0, (axis_vel_var+axis_acc_var*td*td)*td*td, 0}, {0, 0, (axis_vel_var+axis_acc_var*td*td)*td*td}});
        vector<vector<double> > inv_cov_V = matrix3Inverse(cov_V);
        vector<vector<double> > maha = matrixMultiply(matrixMultiply(pT,inv_cov_V), matrixTranspose(pT));
        return roundThisTo(maha[0][0], 6);
    }
    int getNewID() {
        return new_ID;
    }
};
#endif