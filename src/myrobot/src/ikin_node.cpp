#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include "hebiros/EntryListSrv.h"
#include "hebiros/AddGroupFromNamesSrv.h"
#include "hebiros/SizeSrv.h"
#include "opencv_apps/FaceArrayStamped.h"
#include <cstdlib> 
#include <iostream> 
#include <array>
#include <stdio.h>
#include <math.h>

using namespace hebiros;


/*
**   Global Variables.  So the callbacks can pass information.
*/
sensor_msgs::JointState feedback;       // The actuator feedback struccture
volatile int            feedbackvalid = 0;
volatile double         tip_pos;        // The goal position
volatile float * final_dest;       // The goal position
//volatile float[]        joint_commands;   //joint commands


/*
**   Goal Subscriber Callback
*/
void tipCallback(const geometry_msgs::PoseStamped& msg)
{
  final_dest[0] = msg.pose.position.x;
  final_dest[1] = msg.pose.position.y; 
  final_dest[2] = msg.pose.position.z; 
  // msg.pose.quaternion.x, 
  // msg.pose.quaternion.y, 
  // msg.pose.quaternion.z, 
  // msg.pose.quaternion.w];
  // ROS_INFO("I heard: [%f]", (double[]) final_dest);
}


float * fKin(float q1, float q2, float q3, float q4, float q5, float a, float b, float c)
{
  static float pos[5];
  pos[0] = cos(q1) * (a*cos(q2) + b*cos(q3+q2) + c*cos(q2+q3+q4));
  pos[1] = sin(q1) * (a*cos(q2) + b*cos(q2+q3) + c*cos(q2+q3+q4));
  pos[2] = a*sin(q2) + b*sin(q2+q3) + c*sin(q2+q3+q4);
  pos[3] = q2 + q3 + q4;
  pos[4] = q5;

  return pos;
}



float * iKin_tri(float x, float y, float z, float a, float b) {
  static float qs[2];
  float q2;
  float q3;
  float delta_y = z;
  float delta_x = sqrt(x*x + y*y);
  q2 = ((delta_x*delta_x + delta_y*delta_y + a*a - b*b) / (2 * a * sqrt(delta_x*delta_x + delta_y*delta_y)));
  if (abs(q2) < 1) {
    q2 = acos(q2);
  }
  else {
    q2 = 0;
  }
  q2 += atan2(delta_y, delta_x);
  //printf("q2 is %f \n", q2);
  q3 = (a*a + b*b - delta_x*delta_x - delta_y*delta_y) / (2 * a * b);
  if (abs(q3) < 1) {
    q3 = acos(q3);
  }
  else {
    q3 = 0;
  }
  q3 = -(3.14159 - q3);
  //printf("q3 is %f \n", q3);
  qs[0] = q2;
  qs[1] = q3;
  return qs;
}



float * iKin(float x, float y, float z, float pitch, float roll, float a, float b, float c) {

  static float q[5];
  q[0] = atan2(y , x);
  // get x,y,z for joint 3 and then solve triangle for q1,q2,q3
  float z3 = z - c * sin(pitch);
  float length = sqrt(x*x + y*y);
  float x3 = sqrt( pow(length - c*cos(pitch), 2) / (1 + y/x * y/x));
  //printf("x3 is %f \n", x3);
  float y3 = y / x * x3;
  //printf("y3 is %f \n", y3);
  float * res = iKin_tri(x3, y3, z3, a, b);
  q[1] = res[0];
  q[2] = res[1];

  q[3] = pitch - q[1] - q[2];
  q[4] = roll;
  

  return q;
}

float *test_out(float x, float y, float z){
  static float output[5];
  output[0] = x;
  output[1] = y;
  output[2] = z;
  return output;
}

/*
**   Main Code
*/
int main(int argc, char **argv)
{
  // Initialize the basic ROS node, run at 200Hz.
  ros::init(argc, argv, "ikin_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(200);


  // Create a subscriber to listen for a goal.
  ros::Subscriber tipSubscriber = n.subscribe("tipposition", 100, tipCallback);

  // Compute joint states from tip position -> final destination [x,y,z, a, b, c, d] [position quaternion]
  
  

  // Create a publisher to send commands to the actuator group.
  ros::Publisher JointState_publisher = n.advertise<sensor_msgs::JointState>("joint_states", 100);

  sensor_msgs::JointState joint_state;
  // command_msg.name.push_back("cont1");
  // command_msg.header.stamp = ros::Time::now();
  // command_msg.position.resize(1);
  // command_msg.velocity.resize(1);
  // command_msg.effort.resize(1);

  while(ros::ok())
    {
      //joint_commands = iKin(float x, float y, float z, float pitch, float roll, float a, float b, float c)
      //float * joint_commands = test_out(1,2,3);

      // command_msg.position[0] = final_dest[0];
      // command_msg.velocity[0] = final_dest[1];
      // command_msg.effort[0] = final_dest[2];
      // command_msg.position[0] = 1;
      // command_msg.velocity[0] = 1;
      // command_msg.effort[0] = 1;
      joint_state.header.stamp = ros::Time::now();
      joint_state.name.resize(3);
      joint_state.position.resize(3);
      joint_state.name[0] ="cont1";
      joint_state.position[0] = 1;
      joint_state.name[1] ="cont2";
      joint_state.position[1] = 1;
      joint_state.name[2] ="cont3";
      joint_state.position[2] = 1;


      JointState_publisher.publish(joint_state);



      // Wait for next turn.
      ros::spinOnce();
      //loop_rate.sleep();
      //ros::Duration(0.1).sleep();
    }

  return 0;
}
