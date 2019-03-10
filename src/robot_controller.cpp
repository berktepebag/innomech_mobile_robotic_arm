#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <rosgraph_msgs/Clock.h>
#include <iostream>

using namespace std;

class TeleopInnomechRobot
{
public:
  TeleopInnomechRobot();
  void publishSpeed();
  double pidContoller();

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
  double arm0,arm1,arm2,arm3;
  std_msgs::Float64 arm0_msg,arm1_msg,arm2_msg,arm3_msg;
  double linear_axe,angular_axe;

  int current_joint;

  //Timing
  double ros_clock_sec;
  double previous_time;

  //Speed limits
  double min_linear_speed,max_linear_speed;
  //Speed goal values
  double linear_speed_goal,angular_speed_goal;  
  //Speed values from odometer
  double linear_odom_x, angular_odom_z;

  double previous_linear_error;
  
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

double TeleopInnomechRobot::pidContoller(){

  double current_time = ros_clock_sec;
  double dt = current_time - previous_time;

  double Kp = 100;
  double Kd = 10;
  double linear_error = (linear_speed_goal-linear_odom_x);

  /*
  cout << "***---------***" <<endl;
  cout << "linear_speed_goal: " << linear_speed_goal << endl;
  cout << "angular_speed_goal: " << angular_speed_goal << endl;
  cout << "linear_odom_x: " << linear_odom_x << endl;
  cout << "linear_error: " << linear_error << endl;
  */
  double linear_command = 0;

  if (abs(linear_error)>0.01 )
  {
    double de = previous_linear_error- linear_error;
    linear_command = Kp * linear_error + Kd * (de/dt);
    previous_linear_error = linear_error; 
  //cout << "time difference: "<< dt << endl;
    previous_time = ros_clock_sec;
  //cout << "linear_command: " <<linear_command << endl;

  }

  return linear_command;
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
  double link_mult[]={0.5,0.25,0.25,0.25};  

  linear_axe = joy->axes[linear_arm];
  angular_axe = joy->axes[angular_arm];

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
    cout << "Current Joint: " << current_joint <<endl;
    if (current_joint==0)
    {
      arm0 = pos_link0 + angular_axe*link_mult[0];
      /*cout << "joy->axes[angular_arm]*link_mult[0]: " << angular_axe*link_mult[0] <<endl;
      cout << "pos_link0: " << pos_link0 << endl;
      cout << "arm0: " << arm0_msg.data << endl;*/
    }
    else if (current_joint==1){
      arm1 = pos_link1 +joy->axes[linear_arm] *link_mult[1];
      /*cout << "joy->axes[linear_arm]*link_mult[1]: " << joy->axes[linear_arm]*link_mult[1] <<endl;
      cout << "pos_link1: " << pos_link1 << endl;
      cout << "arm1: " << arm1_msg.data << endl;*/
    }
    else if (current_joint==2){    
      arm2 = pos_link2 + linear_axe *link_mult[2];
      /*cout << "linear_axe*link_mult[2]: " << linear_axe*link_mult[2] <<endl;
      cout << "pos_link2: " << pos_link2 << endl;
      cout << "arm2: " << arm2_msg.data << endl;*/
    }
    else if (current_joint==3){
      arm3 = pos_link3 + linear_axe *link_mult[3];
      /*cout << "linear_axe*link_mult[3]: " << linear_axe*link_mult[3] <<endl;
      cout << "pos_link3: " << pos_link3 << endl;
      cout << "arm3: " << arm3_msg.data << endl;*/
    }


    if (joy->buttons[10]){
     arm0 = 0;
     arm1 = 0;
     arm2 = 0;
     arm3 = 0; 
   }

   arm0_msg.data = arm0;
   arm1_msg.data = arm1;
   arm2_msg.data = arm2;
   arm3_msg.data = arm3;


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
//cout << "joy->axes[angular_]: " << joy->axes[angular_] << endl;


}

void TeleopInnomechRobot::publishSpeed(){
  twist.linear.x = pidContoller();
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
