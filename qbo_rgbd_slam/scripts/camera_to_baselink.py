#!/usr/bin/env python  

import roslib
import rospy
import tf

class TransformListenerAndBroadcaster():
    def __init__(self):
        rospy.init_node('camera_base_link_listener')
        self.listener = tf.TransformListener()
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                (trans,rot) = self.listener.lookupTransform('/camera_link_odom', '/base_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            print("Found Transform!")
            self.publish_tf(trans,rot)
            rate.sleep()


    def publish_tf(self, trans, rot):
        br = tf.TransformBroadcaster()
        br.sendTransform(trans,
                        rot,
                        rospy.Time.now(),
                        '/camera_link',
                        '/base_link_vo')

def main():
    ti=TransformListenerAndBroadcaster()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass