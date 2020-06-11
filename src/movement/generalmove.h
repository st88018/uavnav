#ifndef GENERALMOVE_H
#define GENERALMOVE_H

#include <stdio.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include "../utils/kinetic_math.h"
#include "trj_config.h"


class generalMove
{
public:
    generalMove();
    generalMove(double time,
                Vec3 uav_lp_p, Vec4 uav_lp_q,
                double end_x,   double end_y,   double end_z,   double end_yaw_rad,
                double velocity, double angular_velocity);
    void getPose(double time, Vec3 uav_lp_p, Vec4 uav_lp_q,
                 geometry_msgs::PoseStamped& pose);
    int  finished(void);
private:
    double endx, endy, endz, endyaw;
    double startx, starty, startz, startyaw;
    double dist, turnrad;

    double start_time;
    double curr_time;
    double dt;

    double v,av;
    double vx,vy,vz,vyaw;
    double desx,desy,desz,desyaw;

    double est_t,dist_t,yaw_t;
    bool init_flag;
    int gmcounter;
};



#endif // GENERALMOVE_H
