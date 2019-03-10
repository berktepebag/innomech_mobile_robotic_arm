#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <rosgraph_msgs/Clock.h>
#include <iostream>
#include <math.h>

using namespace std;

class TeleopInnomechRobot
{
public:
  TeleopInnomechRobot();
  void publishSpeed();
  double pidContoller(double current, double pid_goal, double* previous_pid_error, double* previous_time, double KP, double KD);

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);
  void clockCallback(const rosgraph_msgs::Clock::ConstPtr& clock_msg);

  ros::NodeHandle nh_;
  geometry_msgs::Twist twist;

  int linear_, angular_,linear_arm,angular_arm;
  double l_scale_, a_scale_;
  double pos_link0,pos_link1,pos_link2,pos_link3;
  double link0_mult,link1_mult,link2_mult,link3_mult;
  double arm0_goal,arm1_goal,arm2_goal,arm3_goal;
  std_msgs::Float64 arm0_msg,arm1_msg,arm2_msg,arm3_msg;
  double linear_axe,angular_axe;

  double mobilebase_pid_error, mobilebase_last_pid_time;
  double arm0_pid_error, arm0_last_pid_time;
  double arm1_pid_error, arm1_last_pid_time;
  double arm2_pid_error, arm2_last_pid_time;
  double arm3_pid_error, arm3_last_pid_time;
  bool move_arm;

  int current_joint;

  //Timing
  double ros_clock_sec;
  

  //Speed limits
  double min_linear_speed,max_linear_speed;
  //Speed goal values
  double linear_speed_goal,angular_speed_goal;  
  //Speed values from odometer
  double linear_odom_x, angular_odom_z;

  //double previous_pid_error;
  //double previous_time;
  
  ros::Publisher vel_pub_;
  ros::Publisher arm_pub_;

  ros::Publisher arm0_pub_;
  ros::Publisher arm1_pub_;
  ros::Publisher arm2_pub_;
  ros::Publisher arm3_pub_;

  ros::Subscriber joy_sub_;
  ros::Subscriber joint_states_sub;
  ros::Subscriber odom_sub_;
  ros::Subscriber clock_sub_;

};


TeleopInnomechRobot::TeleopInnomechRobot():  
angular_(0),
linear_(1),
angular_arm(3),
linear_arm(4)
{

  linear_speed_goal=0;
  current_joint = -1;
  mobilebase_pid_error, mobilebase_last_pid_time,
  arm0_pid_error, arm0_last_pid_time,
  arm1_pid_error, arm1_last_pid_time,
  arm2_pid_error, arm2_last_pid_time,
  arm3_pid_error, arm3_last_pid_time = 0;
  move_arm = false;

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("arm_axis_linear", linear_arm, linear_arm);

  nh_.param("axis_angular", angular_, angular_);
  nh_.param("arm_axis_angular", angular_arm, angular_arm);

  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);


  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 10);

  //arm_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/mra_armLinks_controller/command", 1);
  arm0_pub_ = nh_.advertise<std_msgs::Float64>("/joint0_position_controller/command", 10);
  arm1_pub_ = nh_.advertise<std_msgs::Float64>("/joint1_position_controller/command", 10);
  arm2_pub_ = nh_.advertise<std_msgs::Float64>("/joint2_position_controller/command", 10);
  arm3_pub_ = nh_.advertise<std_msgs::Float64>("/joint3_position_controller/command", 10);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopInnomechRobot::joyCallback, this);
  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/mobile_base_controller/odom",10, &TeleopInnomechRobot::odomCallback, this);
  joint_states_sub = nh_.subscribe<sensor_msgs::JointState>("joint_states", 10, &TeleopInnomechRobot::jointStateCallback, this);
  clock_sub_ = nh_.subscribe<rosgraph_msgs::Clock>("clock", 10, &TeleopInnomechRobot::clockCallback, this);

}

void TeleopInnomechRobot::clockCallback(const rosgraph_msgs::Clock::ConstPtr& clock_msg){
  ros_clock_sec = clock_msg->clock.toSec();
  //cout << "clock_msg: " << ros_clock_sec << endl; 
}

void TeleopInnomechRobot::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg){
  linear_odom_x = odom_msg->twist.twist.linear.x;
  angular_odom_z = odom_msg->twist.twist.angular.z;
  //cout << "odom_msg: " << linear_odom_x << endl; 
}

double TeleopInnomechRobot::pidContoller(double current, double pid_goal, double* previous_pid_error, double* previous_time, double KP, double KD){

  if (*previous_time == 0)
  {
    *previous_time = ros_clock_sec;
  }

  double current_time = ros_clock_sec;
  double dt = current_time - *previous_time;

  double Kp = KP;
  double Kd = KD;
  double pid_error = (pid_goal-current); 
  
  double pid_command = 0;

  if (abs(pid_error)>0.3 )
  {
    double de = *previous_pid_error - pid_error;
    pid_command = Kp * pid_error + Kd * (de/dt);
    *previous_pid_error = pid_error;         
    *previous_time = ros_clock_sec;
  }  
  return pid_command;  
}

void TeleopInnomechRobot::jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_msg){

  pos_link0 = joint_msg->position[3];
  pos_link1 = joint_msg->position[0];
  pos_link2 = joint_msg->position[1];
  pos_link3 = joint_msg->position[2];
  //cout << "pos0: "<<pos_link0 << endl;
}

void TeleopInnomechRobot::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  //Arm Movement
  double link_mult[]={0.5,M_PI/4,M_PI/4,M_PI/4};  

  linear_axe = joy->axes[linear_arm];
  angular_axe = joy->axes[angular_arm];
  int btn_a = joy->buttons[0]; //A
  int btn_b = joy->buttons[1]; //B

  if (btn_a)
  {
    move_arm = !move_arm;
    cout << "move_arm: " << move_arm << endl;
  }

  if (joy->buttons[4] && current_joint > 0)
  {
    current_joint--;
    cout << "Current Joint Changed: " << current_joint <<endl;
  }
  else if (joy->buttons[5] & current_joint < 3)
  {
    current_joint++;
    cout << "Current Joint Changed: " << current_joint <<endl;
  }


  if (joy->axes[angular_arm] || joy->axes[linear_arm])
  { 
    if (!move_arm) cout << "Arm locked, press A to unlock the arm!" << endl;

    else{

      cout << "Current Joint: " << current_joint <<endl;
      if (current_joint==0)
      {
        arm0_goal = pos_link0 + angular_axe*link_mult[0];      
      }
      else if (current_joint==1 && (arm1_goal<=M_PI/4 || arm1_goal>=-M_PI/4)){
        arm1_goal = pos_link1 + linear_axe *link_mult[1];      
      }
      else if (current_joint==2){    
        arm2_goal = pos_link2 + linear_axe *link_mult[2];      
      }
      else if (current_joint==3){
        arm3_goal = pos_link3 + linear_axe *link_mult[3];   
      }
    //Reset the arm to original position
      if (joy->buttons[10]){
       arm0_goal = 0;
       arm1_goal = 0;
       arm2_goal = 0;
       arm3_goal = 0;
     }
     arm0_msg.data = arm0_goal;
     arm1_msg.data = arm1_goal;
     arm2_msg.data = arm2_goal;
     arm3_msg.data = arm3_goal;
   }
 }
 //Mobile Base Movement
 min_linear_speed= -0.2;
 max_linear_speed= 0.5;

 //Stop the robot
 if (joy->buttons[9])
 {
   linear_speed_goal=0;
 } 

 if (joy->axes[2]<1 & linear_speed_goal>min_linear_speed)
 {
   linear_speed_goal -= 0.001;
 }
 else if(joy->axes[5]<1 & linear_speed_goal<=max_linear_speed)
 {
  linear_speed_goal += 0.001;
}
angular_speed_goal = joy->axes[angular_];
}

void TeleopInnomechRobot::publishSpeed(){
  //double pidContoller(double current, double pid_goal, double previous_pid_error, double previous_time);

  twist.linear.x = pidContoller(linear_odom_x, linear_speed_goal,&mobilebase_pid_error,&mobilebase_last_pid_time,100,10);

  twist.angular.z = angular_speed_goal;
  vel_pub_.publish(twist);

  arm0_pub_.publish(arm0_msg);
  arm1_pub_.publish(arm1_msg);
  arm2_pub_.publish(arm2_msg);
  arm3_pub_.publish(arm3_msg);
  
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_innomech_robot");
  TeleopInnomechRobot teleop_innomech_robot;

  ros::Rate rate(50);
  while(ros::ok())
  {
    teleop_innomech_robot.publishSpeed();

    ros::spinOnce();
    rate.sleep();
  }
}
