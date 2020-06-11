#include "generalvelocity.h"
#include "stdio.h"

using namespace std;

generalVelocity::generalVelocity(double time,
                    Vec3 uav_lp_p, Vec4 uav_lp_q,
                    double end_x,   double end_y,   double end_z,   double end_yaw_rad,
                    double velocity, double angular_velocity)
{
    this->start_time = time;
    this->startx = uav_lp_p[0];
    this->starty = uav_lp_p[1];
    this->startz = uav_lp_p[2];
    Quaterniond q(uav_lp_q(0),uav_lp_q(1),uav_lp_q(2),uav_lp_q(3));
    Vec3 rpy = Q2rpy(q);
    this->startyaw = rpy[2];
    this->endx = end_x;
    this->endy = end_y;
    this->endz = end_z;
    this->endyaw = end_yaw_rad;
    this->v = velocity;
    this->av = angular_velocity;
    init_flag = true;
}

void generalVelocity::getPose(double time, Vec3 uav_lp_p, Vec4 uav_lp_q,
                              geometry_msgs::PoseStamped& pose, geometry_msgs::Twist& twist)
{
    curr_time = time;
    if(init_flag == true)
    {
        double dist = (sqrt(pow((endx-startx),2)+pow((endy-starty),2)+pow((endz-startz),2)));
        est_t = dist/v;
        vx = ((endx-startx)/dist)*v;
        vy = ((endy-starty)/dist)*v;
        vz = ((endz-startz)/dist)*v;
        if (startyaw>=M_PI)  startyaw-=2*M_PI;
        if (startyaw<=-M_PI) startyaw+=2*M_PI;
        if (endyaw>=M_PI)    endyaw-=2*M_PI;
        if (endyaw<=-M_PI)   endyaw+=2*M_PI;
        double d_yaw = endyaw - startyaw;
        if (d_yaw>=M_PI)  d_yaw-=2*M_PI;
        if (d_yaw<=-M_PI) d_yaw+=2*M_PI;
        yaw_t = d_yaw/av;

        if(yaw_t>=est_t){est_t = yaw_t;}
        
    }
    if(init_flag == false)
    {
        double dt=curr_time-start_time;
        desx = startx+dt*vx;
        desy = starty+dt*vy;
        desz = starty+dt*vz;
        if(dt<=yaw_t)
        {
            desyaw = startyaw+dt*av;
        }else{
            desyaw = endyaw;
        }
        
        
        pose.pose.position.x = desx;
        pose.pose.position.y = desy;
        pose.pose.position.z = desz;
        Quaterniond q = rpy2Q(Vec3(0,0,desyaw));
        pose.pose.orientation.w = q.w();
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();

        twist.linear.x = vx;
        twist.linear.y = vy;
        twist.linear.z = vz;
    }
}

int generalVelocity::finished()
{
    if((curr_time-start_time)>=est_t)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
