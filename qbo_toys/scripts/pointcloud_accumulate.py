#!/usr/bin/env python

import sys

import numpy as np
import roslib; roslib.load_manifest('sensor_msgs')
import rospy
import tf

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from point_cloud2 import read_points, create_cloud

class CloudStitcher(object):
    def __init__(self):
        self.tf = tf.TransformListener()
        self.seq = 0

        # subscribe to point clouds
        self.cloud_sub = rospy.Subscriber('/camera/depth/points', PointCloud2, self._cloud_cb)

        # output publisher
        self.cloud_pub = rospy.Publisher('/fused_points', PointCloud2)

        self.fused = None

    def _cloud_cb(self, cloud):
        points = np.array(list(read_points(cloud, skip_nans=True)))
        if points.shape[0] == 0:
            return

        # Get 4x4 matrix which transforms point cloud co-ords to odometry frame
        try:
            points_to_map = self.tf.asMatrix('/odom', cloud.header)
        except tf.ExtrapolationException:
            return

        transformed_points = points_to_map.dot(np.vstack((points.T, np.ones((1, points.shape[0])))))
        transformed_points = transformed_points[:3,:].T

        if self.fused is None:
            self.fused = transformed_points
        else:
            self.fused = np.vstack((self.fused, transformed_points))

        self.seq += 1
        header = Header()
        header.seq = self.seq
        header.stamp = rospy.Time.now()
        header.frame_id = '/odom'

        output_cloud = create_cloud(header, cloud.fields, self.fused)

        rospy.loginfo('Publishing new cloud')
        self.cloud_pub.publish(output_cloud)

def main():
    rospy.init_node('pointcloud_accumulate')
    stitcher = CloudStitcher()
    rospy.spin()

if __name__ == '__main__':
    main()
