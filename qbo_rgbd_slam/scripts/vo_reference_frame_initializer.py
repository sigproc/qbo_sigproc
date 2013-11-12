#!/usr/bin/env python  

import roslib
import rospy
import tf

class TransformInitializer():
    def __init__(self):
        rospy.init_node('vo_initializer')
        self.listener = tf.TransformListener()
        self.transformFound = False
        self.listener = tf.TransformListener()
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            if(self.transformFound==False):
                try:
                    (trans,rot) = self.listener.lookupTransform('/world', '/camera_link_odom', rospy.Time(0))
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
                self.transformFound = True
                print("Found Transform!")
            else:
                self.publish_tf(trans,rot)
            rate.sleep()


    def publish_tf(self, trans, rot):
        br = tf.TransformBroadcaster()
        br.sendTransform(trans,
                        rot,
                        rospy.Time.now(),
                        '/odom_vo',
                        'world')

def main():
    ti=TransformInitializer()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass