/*
 * -> input speeds: /cmd_vel
 * -> observed velocities for each wheel: /joint_states ""
 * -> observed torques for each wheel: /joint_states
 * -> observed odometry: /odom "geometry_msgs/Twist"
 * -> ground truth pose: transform /map -> /base_link
 * -> 
 */


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <fstream>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <vector>

geometry_msgs::Twist cmd_base_vel;
bool cmd_base_vel_reveived = false;

sensor_msgs::JointState joint_states;
bool joint_states_received = false;

nav_msgs::Odometry odometry;
bool odometry_received = false;


void commandedBaseVelocityCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  cmd_base_vel = *msg; 
  cmd_base_vel_reveived = true;
}

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  joint_states = *msg; 
  joint_states_received = true;
}

void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  odometry = *msg; 
  odometry_received = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aiciss_data_logger");
  ros::NodeHandle nh("~");

  tf::TransformListener listener;

  ros::Subscriber sub_commanded_base_vel = nh.subscribe("/cmd_vel", 1, commandedBaseVelocityCallback);
  ros::Subscriber sub_joint_states = nh.subscribe("/joint_states", 1, jointStateCallback);
  ros::Subscriber sub_odometry = nh.subscribe("/odom", 1000, odometryCallback);

  std::vector<std::string> wheel_names;
  wheel_names.push_back("wheel_joint_fl");
  wheel_names.push_back("wheel_joint_fr");
  wheel_names.push_back("wheel_joint_bl");
  wheel_names.push_back("wheel_joint_br");

  // open file 
	std::ofstream log_file;
  log_file.open ("/home/aiciss/youbot.log");

  log_file << "timestamp; pos(rad) of wheel_joint_fl; vel(rad/s) of wheel_joint_fl; torq(?) of wheel_joint_fl; pos(rad) of wheel_joint_fr; vel(rad/s) of wheel_joint_fr; torq(?) of wheel_joint_fr; pos(rad) of wheel_joint_bl; vel(rad/s) of wheel_joint_bl; torq(?) of wheel_joint_bl; pos(rad) of wheel_joint_br; vel(rad/s) of wheel_joint_br; torq(?) of wheel_joint_br; odom x (m); odom y(m); odom yaw(rad); grd truth x(m); grd truth y(m); grd truth yaw(rad); input vel(m/s) lin. x; input vel(m/s) lin. y; input vel(m/s) ang. z;" << std::endl;

  ros::Duration diff;

  ros::Rate loop_rate(30);

  while (ros::ok())
  {
    ros::spinOnce();

    if(joint_states_received && odometry_received && (fabs(joint_states.header.stamp - odometry.header.stamp) < 0.1))
    {
      log_file << joint_states.header.stamp << "; ";

      // pos, vel, torque for all four wheels
      for(size_t j=0; j < wheel_names.size(); ++j)
      {
        for(size_t i=0; i < joint_states.name.size(); ++i)
	{
	  if(joint_states.name[i] == wheel_names[j])
	  {
	    log_file << joint_states.position[i] << "; ";
	    log_file << joint_states.velocity[i] << "; ";
	    log_file << joint_states.effort[i] << "; ";
	  }
  	}
      }

      tf::Quaternion q(odometry.pose.pose.orientation.x, odometry.pose.pose.orientation.y, odometry.pose.pose.orientation.z, odometry.pose.pose.orientation.w);
      double yaw, pitch, roll;
      tf::Matrix3x3(q).getEulerYPR(yaw,pitch,roll);

      // odom 
      log_file << odometry.pose.pose.position.x << "; " << odometry.pose.pose.position.y << "; " << yaw << "; "; 

      // ground truth
      /*
      tf::StampedTransform transform;
      try
      {
        listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
   	log_file << transform.getOrigin().x() << "; " << transform.getOrigin().y() << "; " << transform.getOrigin().y() << "; "; 
      } 
      catch (tf::TransformException ex)
      {
        ROS_ERROR("Could not lookup transform between /map and /base_link: %s",ex.what());
      }
      */

      // input velocities
      log_file << cmd_base_vel.linear.x << "; " << cmd_base_vel.linear.y << "; " << cmd_base_vel.angular.z << "; "; 

      joint_states_received = false;
      odometry_received = false;

      log_file << std::endl;
    }

    loop_rate.sleep();
  }

  log_file.close();

  return 0;
}
