#include "ros/ros.h"

#include <tf/transform_listener.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>

class Point2PointNavigator
{
  public:
    Point2PointNavigator()
      : nh("~")
    {

      sub = nh.subscribe("/move_base_simple/goal", 1, &Point2PointNavigator::navigate, this);

      vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

      ROS_INFO("Point2PointNavigator initialized!");
    }

  	void navigate(const geometry_msgs::PoseStamped);

  private:
  	ros::Subscriber sub;
    ros::NodeHandle nh;
    ros::Publisher vel_pub;
    tf::TransformListener transform_listener;

};

void Point2PointNavigator::navigate(geometry_msgs::PoseStamped msg){
	ROS_INFO("Recieved Pose");
	try{
		tf::StampedTransform transform;
    	transform_listener.lookupTransform("/odom", msg.header.frame_id, ros::Time(0), transform);
    	ROS_INFO("Transform Calculated");
    	tf::Vector3 t;
    	t=transform.getOrigin();
    	std::cout<<t[0]<<" "<<t[1]<<" "<<t[2]<<std::endl;
    }
    catch (tf::TransformException ex){
      	ROS_ERROR("%s",ex.what());
    }
}

int main(int argc, char **argv)
{
	ros::init(argc,argv, "point2point_navigator");
	Point2PointNavigator nav;

	ros::spin();
}