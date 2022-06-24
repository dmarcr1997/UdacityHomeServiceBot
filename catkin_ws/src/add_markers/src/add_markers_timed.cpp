#include <ros/ros.h>
#include <visualization_msgs/Marker.h>


int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers_timed");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  uint32_t shape = visualization_msgs::Marker::CUBE;
  
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
    marker.pose.position.x = 2;
    marker.pose.position.y = -1.8;
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
    ROS_INFO("Publishing Marker to pickup...");
    
    marker_pub.publish(marker);
   
   	sleep(5);
   	
    ROS_INFO("Hiding Marker...");
    marker.action = visualization_msgs::Marker::DELETE;
     marker_pub.publish(marker);
    sleep(5);
   
    
    marker.action = visualization_msgs::Marker::ADD;
    
    //Drop off location
    marker.pose.position.x = 2.476;
    marker.pose.position.y = -3.53;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
	marker.lifetime = ros::Duration();
    
    ROS_INFO("Publishing Marker to dropoff...");
    marker_pub.publish(marker);
    marker.action = visualization_msgs::Marker::ADD;
    
    while(ros::ok()){
     ros::spinOnce();
  	}
    r.sleep();
  }
}
