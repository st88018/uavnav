#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <math.h>
#include "std_msgs/String.h"
#include "utils/yamlRead.h"
#include "utils/eigen_typedef.h"
#include "movement/generalmove.h"
#include "movement/circletrj.h"

Vec4 takeoff;
double takeoff_x,takeoff_y,takeoff_z,takeoff_yaw;
int    half_circle;
bool   force_start;
bool   Initialfromtakeoffpos;
bool   Initilized;

#define PI (3.1415926)

using namespace std;

mavros_msgs::State current_state;
double uavposition_x,uavposition_y,uavposition_z,uavposition_yaw;
// double tmpposition_x,tmpposition_y,tmpposition_z,tmporientation_yaw;
double uav_lp_x,uav_lp_y,uav_lp_z;
double uav_lp_qx,uav_lp_qy,uav_lp_qz,uav_lp_qw;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
  current_state = *msg;
}

void uav_lp_cb(const geometry_msgs::PoseStamped::ConstPtr& pose){
  uav_lp_x = pose->pose.position.x;
  uav_lp_y = pose->pose.position.y;
  uav_lp_z = pose->pose.position.z;
  uav_lp_qx = pose->pose.orientation.x;
  uav_lp_qy = pose->pose.orientation.y;
  uav_lp_qz = pose->pose.orientation.z;
  uav_lp_qw = pose->pose.orientation.w;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "offboard_node");
  ros::NodeHandle nh("~");

  string configFilePath;

  nh.getParam("force_start", force_start);
  nh.getParam("Initialfromtakeoffpos", Initialfromtakeoffpos);
  // cout << force_start << endl;

  takeoff_x = 0.0;
  takeoff_y = 0.0;
  takeoff_z = 1.2;
  takeoff_yaw = 0.0;
  takeoff = Vec4(takeoff_x,takeoff_y,takeoff_z,takeoff_yaw);

  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
      ("/mavros/state", 10, state_cb);
  ros::Subscriber uavposlp_sub = nh.subscribe<geometry_msgs::PoseStamped>
      ("/mavros/local_position/pose", 10, uav_lp_cb);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
      ("/mavros/setpoint_position/local", 10);
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
      ("/mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
      ("/mavros/set_mode");

  //the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);
  //wait for FCU connection
  if(force_start)
  {
    cout << "force start " << endl;
  }
  else
  {
    cout << "Waiting for FCU connection " << endl;
    while(ros::ok() && !current_state.connected){
      ros::spinOnce();
      rate.sleep();
      cout << "Waiting for FCU connection " << endl;
    }
  }
  if(Initialfromtakeoffpos)
  {
    cout << "Update takeoff pos " << endl;
  }
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "world";
  pose.pose.position.x = 0.0;
  pose.pose.position.y = 0.0;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.x=0.0;
  pose.pose.orientation.y=0.0;
  pose.pose.orientation.z=0.0;
  pose.pose.orientation.w=1.0;

  //send a few setpoints before starting
  if(force_start)
  {
    cout << "force start " << endl;
  }
  else
  {
    for(int i = 100; ros::ok() && i > 0; --i){
      local_pos_pub.publish(pose);
      ros::spinOnce();
      rate.sleep();
    }
  }

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;
  cout << "change last_request A" << endl;
  ros::Time last_request = ros::Time::now();

  while(ros::ok()){
    //update the uav position
    //from motion capture system
    //local postion estimator(in simulation platform)
    uavposition_x = uav_lp_x;
    uavposition_y = uav_lp_y;
    uavposition_z = uav_lp_z;

    static bool Missionstart = false;
    /*Update takeoff Position**********************************************/
    if(Initialfromtakeoffpos)
    {
      static bool takeoffinit=true;
      if(takeoffinit){
        cout << "Update takeoff pos " << endl;
        Quaterniond q;
        q.w() = uav_lp_qw;
        q.x() = uav_lp_qx;
        q.y() = uav_lp_qy;
        q.z() = uav_lp_qz;
        Vec3 rpy = Q2rpy(q);
        takeoff = Vec4(uav_lp_x,uav_lp_y,uav_lp_z,rpy[2]);
        Initialfromtakeoffpos = false;
        takeoffinit = false;

        cout << "takeoff_x:" << takeoff[0] << endl;
        cout << "takeoff_y:" << takeoff[1] << endl;
        cout << "takeoff_z:" << takeoff[2] << endl;
        cout << "takeoff_yaw:" << takeoff[3] << endl;
      }
    }
    /*offboard and arm*****************************************************/
    if(force_start){
      static bool once=true;
      if(once){
        last_request = ros::Time::now();
        cout << "force start the mission " << endl;
        once = false;
        Missionstart = true;
      }
    }
    else{
      if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(1.0))){
        if( set_mode_client.call(offb_set_mode) &&
            offb_set_mode.response.mode_sent){
          ROS_INFO("Offboard enabled");
        }
        last_request = ros::Time::now();
      }else{
        if( !current_state.armed && 
            (ros::Time::now() - last_request > ros::Duration(1.0))){
          if( arming_client.call(arm_cmd) && arm_cmd.response.success){
            ROS_INFO("Vehicle armed");
            Missionstart = true;
          }
          last_request = ros::Time::now();
        }
      }
    }
    /*Mission Starts here***************************************************/
    if(Missionstart){
      last_request = ros::Time::now();




    }
  }
  return 0;
}
