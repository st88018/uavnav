#ifndef HEADTOWARDS_H
#define HEADTOWARDS_H

#include <stdio.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include "../utils/kinetic_math.h"
#include "trj_config.h"


class headtowards
{
public:
    headtowards();
    headtowards(double time,
                double start_x, double start_y, double start_z, double start_yaw_rad,
                double end_x, double end_y, double end_z, double end_yaw,
                double speed);
    void getPose(double time,
                 geometry_msgs::PoseStamped& pose);
    void setVerticalVelocityLimit(double v);
    void setHorizonVelocityLimit(double v);
    void setAngularSpeedRadLIMIT(double w);
    void setGeoFence(double xMinLim, double xMaxLim, double yMinLim, double yMaxLim, double zMinLim, double zMaxLim);
    int  finished(void);
private:
    double endx, endy, endz, endyaw;
    double startx, starty, startz, startyaw;
    double distance,speed, head_yaw, speed_yaw;
    double hv_limit,vv_limit,av_limit;
    double gfxmin,gfxmax,gfymin,gfymax,gfzmin,gfzmax;
    double start_time;
    double curr_time;
    double vx,vy,vz,v_yaw1,v_yaw2;
    double h_est_t,yaw_est_t1,yaw_est_t2;
    int est_flag;
};



#endif // GENERALMOVE_H
