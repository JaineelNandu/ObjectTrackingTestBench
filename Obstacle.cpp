#include "ObstacleType.hpp"
#include "SensorType.cpp"

class Obstacle {
    private:
    double p_x, p_y, p_z, v_x, v_y, v_z, a_x, a_y, a_z;
    obstacleType type;
    SensorType s;
    double time_stamp;
    int KF_Instances[3] = {-1, -1, -1};
    int obstacle_ID;
    int out_FOV_count;

    public:
    Obstacle() {
        p_x, p_y, p_z, v_x, v_y, v_z, a_x, a_y, a_z = 0.0;
        type = obstacleType::unclassified;
        s = SensorType::global;
        time_stamp = -1.0;
        obstacle_ID = -1;
        out_FOV_count = 20;
    } 
};