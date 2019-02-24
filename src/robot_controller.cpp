#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <iostream>

using namespace std;

class TeleopInnomechRobot
{
public:
  TeleopInnomechRobot();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint);

  ros::NodeHandle nh_;

  int linear_, angular_,linear_arm,angular_arm;
  double l_scale_, a_scale_;
  double pos_link0,pos_link1,pos_link2,pos_link3;
  double link0_mult,link1_mult,link2_mult,link3_mult;

  ros::Publisher vel_pub_;
  ros::Publisher arm_pub_;
  ros::Subscriber joy_sub_;
  ros::Subscriber joint_states_sub;
};


TeleopInnomechRobot::TeleopInnomechRobot():  
angular_(0),
linear_(1),
angular_arm(3),
linear_arm(4)
{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("arm_axis_linear", linear_arm, linear_arm);

  nh_.param("axis_angular", angular_, angular_);
  nh_.param("arm_axis_angular", angular_arm, angular_arm);

  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);


  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 1);

  arm_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/mra_armLinks_controller/command", 1);
  

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopInnomechRobot::joyCallback, this);

  joint_states_sub = nh_.subscribe<sensor_msgs::JointState>("joint_states", 10, &TeleopInnomechRobot::jointStateCallback, this);

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
  geometry_msgs::Twist twist;

  double link_mult[]={1.0,0.95,0.90,0.85};

  double arm0,arm1,arm2,arm3;
  arm0 = joy->axes[angular_arm] + pos_link0*link_mult[0];
  arm1 = joy->axes[linear_arm] + pos_link1*link_mult[1];
  arm2 = joy->axes[linear_arm] + pos_link2*link_mult[2];
  arm3 = joy->axes[linear_arm] + pos_link3*link_mult[3];

  if (joy->buttons[10])
  {
   arm0 = 0;
   arm1 = 0;
   arm2 = 0;
   arm3 = 0;
 }


  //cout << "arm0: " << arm0 << endl;


 std::vector<double> arm_vector;
 arm_vector.push_back(arm0);
 arm_vector.push_back(arm1);
 arm_vector.push_back(arm2);
 arm_vector.push_back(arm3);


 std_msgs::Float64MultiArray arm_msg;

 arm_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
 arm_msg.layout.dim[0].size = 4;
 arm_msg.layout.dim[0].stride = 1;
 arm_msg.layout.dim[0].label =' ';


 arm_msg.data.clear();
 arm_msg.data.insert(arm_msg.data.end(), arm_vector.begin(), arm_vector.end());

 arm_pub_.publish(arm_msg);


 if (joy->buttons[9])
 {
   twist.angular.z = 0;
   twist.linear.x = 0; 
 }

 twist.angular.z = joy->axes[angular_];
 twist.linear.x = joy->axes[linear_];
 vel_pub_.publish(twist);


}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_innomech_robot");
  TeleopInnomechRobot teleop_innomech_robot;

  ros::spin();
}
