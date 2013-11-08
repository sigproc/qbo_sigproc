#!/usr/bin/env python
import sys
import rospy
import roslib
import tf 
from geometry_msgs.msg import PoseStamped




if __name__ == '__main__':
	pub_pose = rospy.Publisher("/current_pose", PoseStamped)
	rospy.init_node('nav_pose_sub_pub', anonymous=True)
	listener = tf.TransformListener()
	current_pose = PoseStamped()
	while not rospy.is_shutdown():
		rate = rospy.Rate(5.0)
		try: 
			(position,orientation) = listener.lookupTransform('odom', 'base_footprint', rospy.Time(0))
			(current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z) = position;
			(current_pose.pose.orientation.x,current_pose.pose.orientation.y,current_pose.pose.orientation.z,current_pose.pose.orientation.w) = orientation
			current_pose.header.frame_id = "odom"
			current_pose.header.stamp = rospy.Time(0)
			pub_pose.publish(current_pose)
		except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
		pub_pose.publish(current_pose)
		rate.sleep()



