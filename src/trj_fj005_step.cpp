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
#include "movement/generalstep.h"
#include "movement/generalmove.h"
#include "movement/circletrj.h"
#define PI (3.1415926)
using namespace std;

enum Mission_STATE {
  IDLE,
  TAKEOFFP1,
  TAKEOFFP2,
  HOVER1,
  RECT1,
  HOVER2,
  RECT2,
  HOVER3,
  RECT3,
  HOVER4,
  RECT4,
  HOVER5,
  RECT5,
  HOVER6,
  RECT6,
  HOVER7,
  RECT7,
  HOVER8,
  RECT8,
  HOVER9,
  RETURN,
  LANDING,
  END,
} mission_state=IDLE;

mavros_msgs::State current_state;
double takeoff_x,takeoff_y,takeoff_z,takeoff_yaw;
int    half_circle;
bool   force_start; 
bool   Initialfromtakeoffpos;
double tmpposition_x,tmpposition_y,tmpposition_z,tmporientation_yaw;
double uav_lp_x,uav_lp_y,uav_lp_z;
double uav_lp_qx,uav_lp_qy,uav_lp_qz,uav_lp_qw;
double velocity_takeoff,velocity_angular, velocity_mission, altitude_mission;
Vec3 uav_lp_p;
Vec4 uav_lp_q;
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
  uav_lp_p = Vec3(uav_lp_x,uav_lp_y,uav_lp_z);
  uav_lp_q = Vec4(uav_lp_qw,uav_lp_qx,uav_lp_qy,uav_lp_qz);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "offboard_node");
  ros::NodeHandle nh("~");

  string configFilePath;

  takeoff_x = 0.0;
  takeoff_y = 0.0;
  takeoff_z = 0.0;
  takeoff_yaw = 0; //in rad
  velocity_mission = 0.5;
  velocity_takeoff = 0.5;
  altitude_mission = 1;

  nh.getParam("force_start", force_start);
  nh.getParam("Initialfromtakeoffpos", Initialfromtakeoffpos);
  nh.getParam("Velocity_mission", velocity_mission);
  nh.getParam("Velocity_takeoff", velocity_takeoff);
  nh.getParam("Velocity_angular", velocity_angular);
  nh.getParam("Altitude_mission", altitude_mission);
  cout << force_start << endl;

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
  if(force_start){cout << "force start " << endl;}
  else{
    cout << "Waiting for FCU connection " << endl;
    while(ros::ok() && !current_state.connected){
      ros::spinOnce();
      rate.sleep();
      cout << "Waiting for FCU connection " << endl;
    }}

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
   /*Update takeoff Position**********************************************/
    if(Initialfromtakeoffpos)
    {
      static bool takeoffinit=true;
      if(takeoffinit){
        Quaterniond q(uav_lp_qw,uav_lp_qx,uav_lp_qy,uav_lp_qz);
        Vec3 rpy = Q2rpy(q);
        takeoff_x = uav_lp_x;
        takeoff_y = uav_lp_y;
        takeoff_z = uav_lp_z;
        takeoff_yaw = rpy[2];
        Initialfromtakeoffpos = false;
        takeoffinit = false;
        cout << "====================================================" <<endl;
        cout << "====================================================" <<endl;
        cout << "Mission Params Initialized" << endl;
        cout << " " << endl;
        cout << "takeoff_x: " << takeoff_x << endl;
        cout << "takeoff_y: " << takeoff_y << endl;
        cout << "takeoff_z: " << takeoff_z << endl;
        cout << "takeoff_yaw: " << takeoff_yaw << endl;
        cout << "Mission Altitude: " << altitude_mission << endl;
        cout << "Mission Velocity: " << velocity_mission << endl;
        cout << "Angular Velocity: " << velocity_angular << endl;
        cout << "Takeoff Velocity: " << velocity_takeoff << endl;
        cout << "====================================================" <<endl;
        cout << "====================================================" <<endl;
      }
    }
        /*offboard and arm*****************************************************/
    if(force_start)
    {
      static bool once=true;
      if(once)
      {
        mission_state = TAKEOFFP1;
        last_request = ros::Time::now();
        cout << "force start the mission " << endl;
        once = false;
      }
    }
    else
    {
      if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(1.0)))
      {
        if( set_mode_client.call(offb_set_mode) &&
            offb_set_mode.response.mode_sent){
          ROS_INFO("Offboard enabled");
        }
        last_request = ros::Time::now();
      } else {
        if( !current_state.armed &&
            (ros::Time::now() - last_request > ros::Duration(1.0))){
          if( arming_client.call(arm_cmd) &&
              arm_cmd.response.success){
            ROS_INFO("Vehicle armed");
            mission_state = TAKEOFFP1;
          }
          last_request = ros::Time::now();
        }
      }
    }

    /*takeoff*****************************************************/
    //PLEASE DEFINE THE LANDING PARAMETER HERE
    if(mission_state==TAKEOFFP1)
    {
      static generalMove takeoff1(ros::Time::now().toSec(),
                                  uav_lp_p, uav_lp_q,
                                  takeoff_x,takeoff_y,altitude_mission-0.2,takeoff_yaw,
                                  velocity_takeoff,velocity_angular);
      takeoff1.getPose(ros::Time::now().toSec(), uav_lp_p, uav_lp_q, pose);
      if(takeoff1.finished())
      {
        cout << "Takeoff P1 finished" << endl;
        mission_state = TAKEOFFP2;
        last_request = ros::Time::now();
      }
    }
    if(mission_state==TAKEOFFP2)
    {
      static generalMove takeoff2(ros::Time::now().toSec(),
                                  uav_lp_p, uav_lp_q,
                                  takeoff_x,takeoff_y,altitude_mission,takeoff_yaw,
                                  velocity_takeoff,velocity_angular);
      takeoff2.getPose(ros::Time::now().toSec(), uav_lp_p, uav_lp_q, pose);
      if(takeoff2.finished())
      {
        mission_state = HOVER1;
        cout << "Takeoff P2 finished" << endl;
        last_request = ros::Time::now();
      }
    }

    if(mission_state==HOVER1)
    {
      if(ros::Time::now()-last_request > ros::Duration(1.0))
      {
        mission_state = RECT1;
        cout << "Hover1 finished" << endl;
        last_request = ros::Time::now();
      }
    }

    if(mission_state==RECT1)
    {
      static generalMove gm(ros::Time::now().toSec(),
                            uav_lp_p, uav_lp_q,
                            takeoff_x+0.75, takeoff_y+0.75, altitude_mission, takeoff_yaw,
                            velocity_mission,velocity_angular);
      gm.getPose(ros::Time::now().toSec(),uav_lp_p, uav_lp_q, pose);
      if(gm.finished())
      {
        mission_state = HOVER2;
        last_request = ros::Time::now();
      }
    }

    if(mission_state==HOVER2)
    {
      if(ros::Time::now()-last_request > ros::Duration(5.0))
      {
        mission_state = RECT2;
        cout << "Hover2 finished" << endl;
        last_request = ros::Time::now();
      }
    }

    if(mission_state==RECT2)
    {
      static generalStep gm(ros::Time::now().toSec(),
                            takeoff_x-0.75, takeoff_y+0.75, altitude_mission, takeoff_yaw
                            ,5);
      gm.getPose(ros::Time::now().toSec(),pose);
      if(gm.finished())
      {
        mission_state = HOVER3;
        last_request = ros::Time::now();
      }
    }

    if(mission_state==HOVER3)
    {
      if(ros::Time::now()-last_request > ros::Duration(1.0))
      {
        mission_state = RECT3;
        cout << "Hover3 finished" << endl;
        last_request = ros::Time::now();
      }
    }

    if(mission_state==RECT3)
    {
      static generalStep gm(ros::Time::now().toSec(),
                            takeoff_x-0.75, takeoff_y-0.75, altitude_mission,  takeoff_yaw,
                            5);
      gm.getPose(ros::Time::now().toSec(),pose);
      if(gm.finished())
      {
        mission_state = HOVER4;
        last_request = ros::Time::now();
      }
    }

    if(mission_state==HOVER4)
    {
      if(ros::Time::now()-last_request > ros::Duration(1.0))
      {
        mission_state = RECT4;
        cout << "Hover4 finished" << endl;
        last_request = ros::Time::now();
      }
    }

    if(mission_state==RECT4)
    {
      static generalStep gm(ros::Time::now().toSec(),
                            takeoff_x+0.75, takeoff_y-0.75, altitude_mission,  takeoff_yaw,
                            5);
      gm.getPose(ros::Time::now().toSec(),pose);
      if(gm.finished())
      {
        mission_state = HOVER5;
        last_request = ros::Time::now();
      }
    }

    if(mission_state==HOVER5)
    {
      if(ros::Time::now()-last_request > ros::Duration(1.0))
      {
        mission_state = RECT5;
        cout << "Hover5 finished" << endl;
        last_request = ros::Time::now();
      }
    }

    if(mission_state==RECT5)
    {
      static generalStep gm(ros::Time::now().toSec(),
                            takeoff_x+0.75, takeoff_y+0.75, altitude_mission,  takeoff_yaw,
                            5);
      gm.getPose(ros::Time::now().toSec(),pose);
      if(gm.finished())
      {
        mission_state = HOVER6;
        last_request = ros::Time::now();
      }
    }

    if(mission_state==HOVER6)
    {
      if(ros::Time::now()-last_request > ros::Duration(1.0))
      {
        mission_state = RECT6;
        cout << "Hover6 finished" << endl;
        last_request = ros::Time::now();
      }
    }

    if(mission_state==RECT6)
    {
      static generalStep gm(ros::Time::now().toSec(),
                            takeoff_x-0.75, takeoff_y+0.75, altitude_mission, takeoff_yaw,
                            5);
      gm.getPose(ros::Time::now().toSec(),pose);
      if(gm.finished())
      {
        mission_state = HOVER7;
        last_request = ros::Time::now();
      }
    }

    if(mission_state==HOVER7)
    {
      if(ros::Time::now()-last_request > ros::Duration(1.0))
      {
        mission_state = RECT7;
        cout << "Hover7 finished" << endl;
        last_request = ros::Time::now();
      }
    }

    if(mission_state==RECT7)
    {
      static generalStep gm(ros::Time::now().toSec(),
                            takeoff_x-0.75, takeoff_y-0.75, altitude_mission,  takeoff_yaw,
                            5);
      gm.getPose(ros::Time::now().toSec(),pose);
      if(gm.finished())
      {
        mission_state = HOVER8;
        last_request = ros::Time::now();
      }
    }

    if(mission_state==HOVER8)
    {
      if(ros::Time::now()-last_request > ros::Duration(1.0))
      {
        mission_state = RECT8;
        cout << "Hover8 finished" << endl;
        last_request = ros::Time::now();
      }
    }

    if(mission_state==RECT8)
    {
      static generalStep gm(ros::Time::now().toSec(),
                            takeoff_x+0.75, takeoff_y-0.75, altitude_mission, takeoff_yaw,
                            5);
      gm.getPose(ros::Time::now().toSec(),pose);
      if(gm.finished())
      {
        mission_state = HOVER9;
        last_request = ros::Time::now();
      }
    }

    if(mission_state==HOVER9)
    {
      if(ros::Time::now()-last_request > ros::Duration(1.0))
      {
        mission_state = RETURN;
        cout << "Hover9 finished" << endl;
        last_request = ros::Time::now();
      }
    }

    if(mission_state==RETURN)
    {
      static generalMove gotolanding(ros::Time::now().toSec(),
                                     uav_lp_p, uav_lp_q,
                                     takeoff_x,takeoff_y,altitude_mission, takeoff_yaw,
                                     velocity_mission,velocity_angular);
      gotolanding.getPose(ros::Time::now().toSec(),uav_lp_p, uav_lp_q, pose);
      if(gotolanding.finished())
      {
        mission_state = LANDING;
        cout << "reached landing place" << endl;
        last_request = ros::Time::now();
      }
    }

    //PLEASE DEFINE THE LANDING PARAMETER HERE
    if(mission_state==LANDING)
    {
      double secs = (ros::Time::now() - last_request).toSec();
      //cout << secs << endl;
      pose.pose.position.x = takeoff_x;
      pose.pose.position.z = takeoff_y;
      pose.pose.position.z = takeoff_z-(secs)*0.1;
      if(pose.pose.position.z < -0.3)
      {
        pose.pose.position.z=-0.3;
        arm_cmd.request.value = false;
        if( arming_client.call(arm_cmd) &&
            arm_cmd.response.success)
        {
          mission_state=END;
          cout << "Landing P2 finished" << endl;
          return 0;//break the control UAV will land automatically
        }
      }
    }

    if(mission_state==END)
    {
      pose.pose.position.x = 0;
      pose.pose.position.y = 0;
      pose.pose.position.z = -0.3;
      return 0;
    }
    //cout  << pose.pose.position.x << "," << pose.pose.position.y << "," << pose.pose.position.z << endl;

    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
    
    int coutcounter;
    if(coutcounter > 30){
      cout << "currentpos_x: " << uav_lp_x << " y: " << uav_lp_y << " z: "<< uav_lp_z << endl;
      cout << "desiredpos_x: " << pose.pose.position.x << " y: " << pose.pose.position.y << " z: "<< pose.pose.position.z << endl;
      coutcounter = 0;
    }else{coutcounter++;}
    
  }

  return 0;
}
