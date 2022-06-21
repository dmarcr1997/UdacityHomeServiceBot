#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/Float32MultiArray.h"

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Publisher marker_location_pub = n.advertise<std_msgs::Float32MultiArray>("marker_location", 1);

  uint32_t shape = visualization_msgs::Marker::CUBE;
  std_msgs::Float32MultiArray pickupLoc;
  pickupLoc.data.push_back(2.0);
  pickupLoc.data.push_back(2.5);
  
  std_msgs::Float32MultiArray dropOffLoc;
  dropOffLoc.data.push_back(-2.0);
  dropOffLoc.data.push_back(2.0);
  
  while (ros::ok())
  {
    visualization_msgs::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    marker.ns = "basic_shapes";
    marker.id = 0;

    marker.type = shape;

    marker.action = visualization_msgs::Marker::ADD;
	
    //Pickup location
    marker.pose.position.x = 2.0;
    marker.pose.position.y = 2.5;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
	
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    //Step 1 Show Marker at Pickup Zone
    	//subscribe to robot odom
    	//return whether or not robot pose is at pickup
    //Step 2 Hide the marker when robot reaches pickup zone
    //Step 3 Wait 5 seconds
    //Step 4 Show the marker at the drop off zone once robot reaches it
    	//return whether or not robot pose is at drop off
    	//spin
    marker_pub.publish(marker);
   
    marker_location_pub.publish(pickupLoc);
    std::string robot_pose;
    ros::Rate r(1);
    ROS_INFO("Pub marker to pickup LOC...");
   
    //Drop off location
    marker.pose.position.x = -2.0;
    marker.pose.position.y = 2.0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    while(ros::ok()){
      if (ros::param::get("/pose", robot_pose))
      {

        if (robot_pose=="arrived_at_pickup"){
            marker.action = visualization_msgs::Marker::DELETE;
           
            marker_pub.publish(marker);
           	marker_location_pub.publish(dropOffLoc);
            ROS_INFO("Object picked up by robot");

            sleep(5);
        }
        if (robot_pose=="arrived_at_dropoff"){

            // Set the drop-off location
            marker.pose.position.x = -2.0;
            marker.pose.position.y = 2.0;
            marker.pose.position.z = 0;
            marker.pose.orientation.w = 1.0;

            marker.action = visualization_msgs::Marker::ADD;
            marker_pub.publish(marker);
            ROS_INFO("Marker dropped at dropoff");

        }
        ros::spinOnce();
   	 }
     r.sleep();
  	}
  }
}
