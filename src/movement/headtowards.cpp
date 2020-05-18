#include "headtowards.h"
#include "stdio.h"

using namespace std;

headtowards::headtowards()
{
    vv_limit = DEFALUT_VV_LIMIT;
    hv_limit = DEFAULT_HV_LIMIT;
    av_limit = DEFAULT_AV_LIMIT;
    est_flag = 0;
}
headtowards::headtowards(double time,
                double start_x, double start_y, double start_z, double start_yaw_rad,
                double end_x,   double end_y,   double end_z,   double end_yaw,
                double speed, double speed_yaw);
{
    this->start_time = time;
    this->startx = start_x;
    this->starty = start_y;
    this->startz = start_z;
    this->startyaw = start_yaw_rad;
    this->endx = end_x;
    this->endy = end_y;
    this->endz = end_z;
    this->endyaw = end_yaw_rad;
    this->speed = speed;
    this->speed_yaw = speed_yaw;
    vv_limit = DEFALUT_VV_LIMIT;
    hv_limit = DEFAULT_HV_LIMIT;
    av_limit = DEFAULT_AV_LIMIT;
    est_flag = 0;
}

void headtowards::getPose(double time,
                          geometry_msgs::PoseStamped& pose)
{
    curr_time = time;
    if(est_flag == 0) //calc properties
    {
        if (startyaw>=M_PI)  startyaw-=2*M_PI;
        if (startyaw<=-M_PI) startyaw+=2*M_PI;
        if (endyaw>=M_PI)    endyaw-=2*M_PI;
        if (endyaw<=-M_PI)   endyaw+=2*M_PI;
        
        head_yaw = atan((start_y-end_y)/(start_x-end_x));
        if (head_yaw>=M_PI)     head_yaw-=2*M_PI;
        if (head_yaw<=-M_PI)    head_yaw+=2*M_PI;
        double d_yaw1 = head_yaw - startyaw;
        if (d_yaw1>=M_PI)       d_yaw1-=2*M_PI;
        if (d_yaw1<=-M_PI)      d_yaw1+=2*M_PI;
        yaw_est_t1 = fabs(d_yaw1)/speed_yaw;
        v_yaw1 = ((head_yaw - startyaw)/fabs(d_yaw1))*speed_yaw;

        distance = sqrt(pow((start_x-end_x),2)+pow((start_y-end_y),2)+pow((start_z-end_z),2));
        h_est_t = distance/speed;
        vx = ((start_x-end_x)/distance)*speed;
        vy = ((start_y-end_y)/distance)*speed;
        vz = ((start_z-end_z)/distance)*speed;
        
        double d_yaw2 = end_yaw - head_yaw;
        if (d_yaw2>=M_PI)       d_yaw2-=2*M_PI;
        if (d_yaw2<=-M_PI)      d_yaw2+=2*M_PI;
        yaw_est_t2 = fabs(d_yaw2)/speed_yaw;
        v_yaw2 = ((end_yaw - head_yaw)/fabs(d_yaw2))*speed_yaw;
        
        est_flag = 1;
        
        if(vx > hv_limit || vy > hv_limit){std::cout << "check horizen speed limit" << std::endl; est_flag = 4;}
        if(vz > vv_limit){std::cout << "check vertical speed limit" << std::endl; est_flag = 4;}
        if(speed_yaw > av_limit){std::cout << "check angular speed limit" << std::endl; est_flag = 4;}

        // if(est_t>0.0 && est_t<1000.0)
        // {
        //     double hv = (sqrt(pow((endx-startx),2)+pow((endy-starty),2)))/est_t;
        //     double vv = (sqrt(pow((endz-startz),2)))/est_t;
        //     if (startyaw>=M_PI)  startyaw-=2*M_PI;
        //     if (startyaw<=-M_PI) startyaw+=2*M_PI;
        //     if (endyaw>=M_PI)    endyaw-=2*M_PI;
        //     if (endyaw<=-M_PI)   endyaw+=2*M_PI;
        //     double d_yaw = endyaw - startyaw;
        //     if (d_yaw>=M_PI)  d_yaw-=2*M_PI;
        //     if (d_yaw<=-M_PI) d_yaw+=2*M_PI;
        //     double av = fabs(d_yaw/est_t);
        //     if(hv > hv_limit){std::cout << "check horizen speed limit" << std::endl;}
        //     if(vv > vv_limit){std::cout << "check vertical speed limit" << std::endl;}
        //     if(av > av_limit){std::cout << "check angular speed limit" << std::endl;}
        //     if(hv < hv_limit && vv < vv_limit && av < av_limit)
        //     {
        //         vx = (endx-startx)/est_t;
        //         vy = (endy-starty)/est_t;
        //         vz = (endz-startz)/est_t;
        //         v_yaw = (d_yaw)/est_t;
        //         est_flag = 1;
        //     }
        // }
    }
    if(est_flag == 1) //Turn head toward end
    {
        double dt=curr_time-start_time;
        pose.pose.position.x=startx;
        pose.pose.position.y=starty;
        pose.pose.position.z=startz;
        Quaterniond q = rpy2Q(Vec3(0,0,startyaw+dt*v_yaw1));
        pose.pose.orientation.w = q.w();
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
    }
    if(est_flag == 2) //Move toward end
    {
        double dt=curr_time-start_time-yaw_est_t1;
        pose.pose.position.x=startx+dt*vx;
        pose.pose.position.y=starty+dt*vy;
        pose.pose.position.z=startz+dt*vz;
        Quaterniond q = rpy2Q(Vec3(0,0,head_yaw));
        pose.pose.orientation.w = q.w();
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
    }
    if(est_flag == 3) //Turn head back to setting
    {
        double dt=curr_time-start_time-yaw_est_t1-h_est_t;
        pose.pose.position.x= endx;
        pose.pose.position.y= endy;
        pose.pose.position.z= endz;
        Quaterniond q = rpy2Q(Vec3(0,0,head_yaw+dt*v_yaw2));
        pose.pose.orientation.w = q.w();
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
    }
    if(est_flag == 4) //Stop
    {
        pose.pose.position.x= endx;
        pose.pose.position.y= endy;
        pose.pose.position.z= endz;
        Quaterniond q = rpy2Q(Vec3(0,0,endyaw));
        pose.pose.orientation.w = q.w();
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
    }
}

int generalMove::finished()
{
    if( est_flag ==4 )
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

void generalMove::setVerticalVelocityLimit(double vv)
{
    vv_limit = vv;
}

void generalMove::setHorizonVelocityLimit(double hv)
{
    hv_limit = hv;
}

void generalMove::setAngularSpeedRadLIMIT(double w)
{
    av_limit = w;
}
