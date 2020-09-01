#include "generalmove.h"
#include "stdio.h"

using namespace std;

generalMove::generalMove(double time,
                         Vec3 uav_lp_p, Vec4 uav_lp_q,
                         double end_x, double end_y, double end_z, double end_yaw_rad,
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

void generalMove::getPose(double time, Vec3 uav_lp_p, Vec4 uav_lp_q,
                              geometry_msgs::PoseStamped& pose)
{
    curr_time = time;
    if(init_flag == true)
    {
        dist = sqrt(pow((endx-startx),2)+pow((endy-starty),2)+pow((endz-startz),2));
        dist_t = dist/v;
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
        yaw_t = sqrt(pow(d_yaw/av,2));
        if(yaw_t>=dist_t){est_t = yaw_t;}else{est_t = dist_t;}

        init_flag = false;
    }
    if(init_flag == false)
    {
        dt=curr_time-start_time;
        
        if(dt<=yaw_t)
        {
            desyaw = startyaw+dt*av;
        }else{
            desyaw = endyaw;
        }
        if(dt<=dist_t)
        {
            desx = startx+dt*vx;
            desy = starty+dt*vy;
            desz = startz+dt*vz;
        }else{
            desx = endx;
            desy = endy;
            desz = endz;
        }
        pose.pose.position.x = desx;
        pose.pose.position.y = desy;
        pose.pose.position.z = desz;
        Quaterniond q = rpy2Q(Vec3(0,0,desyaw));
        pose.pose.orientation.w = q.w();
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
    }
    
    // if(gmcounter > 10){
    //   cout << "------------------------------------------------------------------------------" << endl;
    //   cout << "currentpos_x: " << uav_lp_p[0] << " y: " << uav_lp_p[1] << " z: "<< uav_lp_p[2] << endl;
    //   cout << "desiredpos_x: " << desx << " y: " << desy << " z: "<< desz << endl;
    //   cout << "est_t: "<< est_t << " yaw_t: " << yaw_t << " dist_t: " << dist_t <<" dist: "<< dist << endl;
    //   cout << "startx: "<< startx << " starty: "<< starty <<" startz: "<< startz << endl;
    //   cout << "endx: "<< endx << " endy: "<< endy <<" endz: "<< endz << endl;
    //   cout << "vx: " << vx << " vy: " << vy << " vz: " << vz << " dt: "<< dt << endl;
    //   gmcounter = 0;
    //   cout << "------------------------------------------------------------------------------" << endl;
    // }else{gmcounter++;}
}

int generalMove::finished()
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
