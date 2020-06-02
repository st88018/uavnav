#ifndef GENERALSTEP_H
#define GENERALSTEP_H

#include <stdio.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include "../utils/kinetic_math.h"
#include "trj_config.h"


class generalStep
{
public:
    generalStep();
    generalStep(double time,
                double end_x, double end_y, double end_z, double end_yaw,
                double duration);
    void getPose(double time,
                 geometry_msgs::PoseStamped& pose);
    int  finished(void);
private:
    double endx, endy, endz, endyaw;
    double start_time;
    double curr_time;
    double est_t;
};



#endif // GENERALMOVE_H
