#include "generalstep.h"
#include "stdio.h"

using namespace std;

generalStep::generalStep(double time,
                         double end_x, double end_y, double end_z, double end_yaw_rad,
                         double duration)
{
    this->start_time = time;
    this->endx = end_x;
    this->endy = end_y;
    this->endz = end_z;
    this->endyaw = end_yaw_rad;
    this->est_t = duration;
}

void generalStep::getPose(double time,
                          geometry_msgs::PoseStamped& pose)
{
    curr_time = time;
    pose.pose.position.x = endx;
    pose.pose.position.y = endy;
    pose.pose.position.z = endz;
    Quaterniond q = rpy2Q(Vec3(0,0,endyaw));
    pose.pose.orientation.w = q.w();
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    // cout << "Start time: "<< start_time << endl;
}
int generalStep::finished()
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