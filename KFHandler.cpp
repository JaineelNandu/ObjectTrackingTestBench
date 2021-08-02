#ifndef KFHANDLER_CPP
#define KFHANDLER_CPP

#include <vector>
#include "DummyKF.cpp"
#include "Obstacle.cpp"
#include <iostream>

using namespace std;

class KFHandler {
    private:
    vector<int> free_list;
    int num_free;
    vector<DummyKF> KFInstances;
    vector<Obstacle> currently_tracked;

    public:
    KFHandler(const int &num_simultaneous_track) {
        num_free = 3*num_simultaneous_track;
        if (num_free == 0) num_free = 1;
        for (int n = 0; n < num_free; n++) {
            DummyKF k1;
            KFInstances.push_back(k1);
            free_list.push_back(n);
        }
    }

    int getNumObstacleAvailable() {
        return num_free/3;
    }

    vector<Obstacle> getCurrentlyTracked() {
        return currently_tracked;
    }

    vector<bool> getOccupiedInstances() {
        vector<bool> occupied;
        for (int ins = 0; ins < KFInstances.size(); ins++) {
            occupied.push_back(KFInstances[ins].isEnabled());
        }
        return occupied;
    }

    vector<double> getTrackedCoordinates() {
        vector<double> coordinates;
        for (int i = 0; i < KFInstances.size(); i++) {
            coordinates.push_back(KFInstances[i].getPosition());
        }
        return coordinates;
    }

    // Truth is for the DummyKF
    void track(vector<Obstacle> obstacles, vector<vector<double> > truth) {
        if (obstacles.size() == 0) {
            currently_tracked = obstacles;
            resetHandler();
            return;
        }
        obstacles = associateByID(obstacles);
        obstacles = assignKF(obstacles);
        vector<Obstacle> tracked;
        vector<bool> is_assigned = findAssignedAndFree(obstacles);
        int obs = 0;
        while(obs != is_assigned.size() && is_assigned[obs]) {
            vector<double> obs_pos_noisy = obstacles[obs].getPosition();
            vector<double> obs_pos_truth = truth[obs];
            vector<int> instances = obstacles[obs].getKFInstances();
            vector<double> obs_pos_new;
            vector<double> obs_vel_new;
            for (int axis = 0; axis < 3; axis++) {
                KFInstances[instances[axis]].estimate(obs_pos_noisy[axis], obs_pos_truth[axis], obstacles[obs].getTimeStamp());
                obs_pos_new.push_back(KFInstances[instances[axis]].getPosition());
                obs_vel_new.push_back(KFInstances[instances[axis]].getVelocity());
            }
            obstacles[obs].setPosition(obs_pos_new);
            obstacles[obs].setVelocity(obs_vel_new);
            tracked.push_back(obstacles[obs]);
            obs++;
        }
        currently_tracked = tracked;
    }

    vector<Obstacle> associateByID(vector<Obstacle> obstacles) {
        if (currently_tracked.size() <= obstacles.size()) {
            for (int ct = 0; ct < currently_tracked.size(); ct++) {
                for (int ob = 0; ob < obstacles.size(); ob++) {
                    if (currently_tracked[ct].getID() == obstacles[ob].getID()) {
                        obstacles[ob].setKFInstances(currently_tracked[ct].getKFInstances());
                    }
                }
            } 
        }
        else {
            for (int ob = 0; ob < obstacles.size(); ob++) {
                for (int ct = 0; ct < currently_tracked.size(); ct++) {
                    if (currently_tracked[ct].getID() == obstacles[ob].getID()) {
                        obstacles[ob].setKFInstances(currently_tracked[ct].getKFInstances());
                    }
                }
            }
        }
        return obstacles;
    }

    vector<Obstacle> assignKF(vector<Obstacle> obstacle_list) {
        if (obstacle_list.size() == 0) {
            resetHandler();
        }
        else {

            vector<bool> is_assigned = findAssignedAndFree(obstacle_list);
            for (int obs = 0; obs < obstacle_list.size(); obs++) {
                if (is_assigned[obs]) { continue; }
                else {
                    if (num_free >= 3) {
                        vector<int> instances;
                        for (int axis = 0; axis < 3; axis++) {
                            instances.push_back(free_list[0]);
                            KFInstances[free_list[0]].makeEnable();
                            free_list.erase(free_list.begin());
                            num_free--;
                        }
                        obstacle_list[obs].setKFInstances(instances);
                        is_assigned[obs] = true;
                    }
                    else {
                        int position_of_last_assigned = getLowestPriorityAssigned(is_assigned);
                        if (position_of_last_assigned < obs) { break; }
                        else {
                            vector<int> instances = obstacle_list[position_of_last_assigned].getKFInstances();
                            for(int axis = 0; axis < 3; axis++) {
                                KFInstances[instances[axis]].disable();
                                KFInstances[instances[axis]].makeEnable();
                            }
                            obstacle_list[position_of_last_assigned].setKFInstances({-1, -1, -1});
                            obstacle_list[obs].setKFInstances(instances);
                        }
                    }
                }
            }
            is_assigned = findAssignedAndFree(obstacle_list);
        }
        return obstacle_list;
    }

    int getLowestPriorityAssigned(vector<bool> assigned) {
        int lp = assigned.size();
        for (int last = assigned.size()-1; last >= 0; last--) {
            if (assigned[last]) {
                lp = last;
                break;
            }
        }
        return lp;
    }

    vector<bool> findAssignedAndFree(vector<Obstacle> obstacle_list) {
        vector<bool> is_assigned;
        vector<bool> occupied(KFInstances.size(), false);
        for (int obs = 0; obs < obstacle_list.size(); obs++) {
            vector<int> obstacle_instances = obstacle_list[obs].getKFInstances();
            is_assigned.push_back(!areEqualVectors({-1, -1, -1}, obstacle_instances));
            for (int axis = 0; axis < 3; axis++) {
                if (obstacle_instances[axis] != -1) {
                    occupied[obstacle_instances[axis]] = true;
                }
            }
        }
        freeUnOccupied(occupied);
        return is_assigned;
    }

    void freeUnOccupied(vector<bool> isOccupied) {
        if (isOccupied.size() == 0) return;
        for (int i = 0; i < isOccupied.size(); i++) {
            if(!isOccupied[i] && KFInstances[i].isEnabled()) {
                KFInstances[i].disable();
                free_list.push_back(i);
                num_free += 1;
            }
        }
    } 

    void resetHandler() {
        vector<int> new_free;
        for (int i = 0; i < KFInstances.size(); i++) {
            KFInstances[i].disable();
            new_free.push_back(i);
        }
        free_list = new_free;
        num_free = KFInstances.size();
    }
};

#endif