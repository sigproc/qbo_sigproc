#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <iostream>

float robotx, roboty = 0.0; 
ros::Publisher marker_pub;
visualization_msgs::Marker marker;
int experience_index = 0;

void poseCallback(visualization_msgs::Marker startpose)
{
  
  if (experience_index == 0)
	{
		experience_index = startpose.points.size();
	}


  marker.pose.position.x = startpose.points[experience_index].x;
  marker.pose.position.y = startpose.points[experience_index].y;
  //std::cout << "I heard something" <<std::endl;
  marker_pub.publish(marker);

}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "qbo_place_marker");
  ros::NodeHandle n;
  int marker_no = 1;
  n.getParam("marker_no", marker_no);
  ros::Subscriber position_sub = n.subscribe("/test/ExperienceMap/MapMarker",1,poseCallback);
  std::string topic_name = "qbo_place_marker";
  //topic_name.append(std::string(marker_no));
  marker_pub = n.advertise<visualization_msgs::Marker>(topic_name, 1);
  //std::cout << "Main loop has started" <<std::endl;
  marker.header.frame_id = "1";
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.id = marker_no;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = robotx;
  marker.pose.position.y = roboty;
  marker.pose.position.z = 1.5;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.10;
  marker.scale.y = 0.10;
  marker.scale.z = 3.0;
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration();
  marker_pub.publish(marker);
  ros::spin();
}

