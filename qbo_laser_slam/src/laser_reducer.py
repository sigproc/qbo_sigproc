#!/usr/bin/env python

#A node to reduce laser data by fg295

#import ROS dependencies 
import roslib; roslib.load_manifest('qbo_laser_slam')
import rospy
 
#Import ROS laser message
from sensor_msgs.msg import LaserScan

#Import other libraries
import time
import sys

#global


def callback(data):
    n = int(rospy.get_param('~reduction', '2'))
    pub=rospy.Publisher('/scan_reduced', LaserScan)
    newScan=LaserScan()
    newScan=data
    newScan.angle_increment=n*data.angle_increment
    newScan.ranges=data.ranges[1::n]    
    pub.publish(newScan)


def listener():

    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The 
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node so that multiple talkers can
    # run simultaenously.
    rospy.init_node('laser_reducer', anonymous=True)

    rospy.Subscriber("/scan", LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
        
if __name__ == '__main__':
    listener()
