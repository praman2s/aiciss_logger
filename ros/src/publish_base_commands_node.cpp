#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

std_msgs::String event_in;

void eventCallback(const std_msgs::String::ConstPtr& msg)
{
  event_in = *msg; 
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "base_command_publisher");
  ros::NodeHandle nh("~");

  ros::Subscriber sub_event_in = nh.subscribe("/mcr_navigation/relative_base_controller/event_out", 1000, eventCallback);
  ros::Publisher pub_base_cmd = nh.advertise<geometry_msgs::Twist>("/mcr_navigation/relative_base_controller/command", 1);
  ros::Publisher pub_trigger_cmd = nh.advertise<std_msgs::String>("/mcr_navigation/relative_base_controller/trigger", 1);

	sleep(1);

  unsigned int goal_counter = 0;
  const unsigned int max_no_of_goals = 4;

  // define relative goals  
  geometry_msgs::Twist base_cmd[max_no_of_goals];


  base_cmd[0].linear.x = 1.0;	// move 1.5 m forward
  base_cmd[1].linear.y = -1.0;	// shift 1.5 m to the right
  base_cmd[2].linear.x = -1.0;	// move 1.5 m backward
  base_cmd[3].linear.y = 1.0;	// shift 1.5 m to the left


  // publish first relative goal
  pub_base_cmd.publish(base_cmd[goal_counter]);
 
  std_msgs::String string_msg;
  string_msg.data = "start";
  pub_trigger_cmd.publish(string_msg);


  ++goal_counter;


  while (ros::ok())
  {
    ros::spinOnce();
 
    if(event_in.data == "e_goal_reached")
    {
      std::cout << "next goal: " << goal_counter << std::endl;

      pub_base_cmd.publish(base_cmd[goal_counter]);
      pub_trigger_cmd.publish(string_msg);

      ++goal_counter;
      event_in.data = "";


    }

    if(goal_counter >= max_no_of_goals)
    {
      std::cout << "all goals sent" << std::endl;
      break;
    }
  }

  return 0;
}
