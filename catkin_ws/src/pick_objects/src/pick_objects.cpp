#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/Float32MultiArray.h"
#include <string>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
static int calls = 0;
void sendGoal(const std_msgs::Float32MultiArray::ConstPtr& array)
{
  std::string location = "";
  if(calls < 1)
    location = "arrived_at_pickup";
  else
    location = "arrived_at_dropoff";
  calls += 1; 
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  ros::param::set("/pose", "initial_position");
  
  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  
  goal.target_pose.pose.position.x = array->data[0];
  goal.target_pose.pose.position.y = array->data[1];
  goal.target_pose.pose.orientation.w = 1.0;
  
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  ros::param::set("/pose", "heading_to_goal");
  
  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Hooray, the base moved 1 meter forward");
    ros::param::set("/pose", location);
  	sleep(5);
  }
  else{
   	ROS_INFO_STREAM("DATA:" << goal.target_pose.pose.position);
    ROS_INFO("The base failed to move forward 1 meter for some reason");
  	ros::param::set("/pose", "error");
  }
  
}

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle n;
  // Define a position and orientation for the robot to reach
  ros::Subscriber sub = n.subscribe("marker_location", 1000, sendGoal);
  ros::spin();
  return 0;
}