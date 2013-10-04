#!/usr/bin/env python

import numpy as np
import roslib
import rospy
import tf

roslib.load_manifest('qbo_arduqbo')
from sensor_msgs.msg import JointState

# The motor state message type from qbo_arduqbo has annoying capitalisation.
from qbo_arduqbo.msg import motor_state as MotorState

class Mapping(object):
    """
    A mapping between an encoder value and an output angle. It is parameterised
    in terms of a scale and offset such that:

        output_angle = scale * (encoder_value + offset)

    The default scale is 1.0 and the default offset is 0.0.
    """

    def __init__(self, scale=None, offset=None):
        # The float() call here is to massage the input to a floating point
        # value even if we're passed a string or an integer. Mucky but it gets
        # the job done.
        self.scale = float(scale or 1.0)
        self.offset = float(offset or 0.0)

    def encoder_to_angle(self, encoder_value):
        return self.scale * (encoder_value + self.offset)

    def angle_to_encoder(self, angle):
        return (angle / self.scale) - self.offset

class JointOdomController(object):
    """
    Publish a tf from /base_footprint to /head when pan and tilt angles update.
    
    """

    def __init__(self, head_pan=None, head_tilt=None, base_frame_id=None, head_frame_id=None):
        # The default is to have an identity mapping
        self.head_pan = head_pan or Mapping()
        self.head_tilt = head_tilt or Mapping()

        # The default frame ids are 'base_footprint' and 'head'
        self.base_frame_id = base_frame_id or 'base_footprint'
        self.head_frame_id = head_frame_id or 'head'

        # The last pan and tilt as an angle
        self.last_pan_angle = 0
        self.last_tilt_angle = 0
        self.when = 0   # the last time stamp

    def on_head_pan_joint(self, motor_state):
        self.last_pan_angle = self.head_pan.encoder_to_angle(motor_state.position)
        self.when = motor_state.header.stamp
        self._publish_state()

    def on_head_tilt_joint(self, motor_state):
        self.last_tilt_angle = self.head_tilt.encoder_to_angle(motor_state.position)
        self.when = motor_state.header.stamp
        self._publish_state()

    def _publish_state(self):
        """Publish the current head state as a tf message."""
        # Create a broadcaster
        br = tf.TransformBroadcaster()

        # Working out the final pose of the head is subtle. It depends which
        # order we apply the transformations. We need to rotate in the tilt
        # axis (about y) first and then rotate in the pan axis (about z).
        
        # Calculate rotation matrices
        tilt_rot = tf.transformations.rotation_matrix(
                np.deg2rad(self.last_tilt_angle), (0, 1, 0))
        pan_rot = tf.transformations.rotation_matrix(
                np.deg2rad(self.last_pan_angle), (0, 0, 1))

        # Combine rotation matrix
        pose = pan_rot.dot(tilt_rot)
        
        # Calculate and send the tf
        br.sendTransform(
                (0, 0, 0.5),    # head is 0.5m above base_footprint
                tf.transformations.quaternion_from_matrix(pose),
                self.when, self.head_frame_id, self.base_frame_id)

def main():
    """
    Initialise the node, subscribe to topics and spin waiting for data.

    """
    # initialise our node
    rospy.init_node('qbo_joint_odom')

    # get the various mapping parameters
    head_pan_scale = float(rospy.get_param('~head_pan_joint/scale', 0.29))
    head_pan_offset = float(rospy.get_param('~head_pan_joint/offset', -512.0))

    head_tilt_scale = float(rospy.get_param('~head_tilt_joint/scale', 0.44))
    head_tilt_offset = float(rospy.get_param('~head_tilt_joint/offset', -512.0))

    # initialise the controller
    odom_controller = JointOdomController(
            head_pan = Mapping(head_pan_scale, head_pan_offset),
            head_tilt = Mapping(head_pan_scale, head_pan_offset))

    # which topics should we subscribe to?
    head_pan_joint_topic = str(rospy.get_param('~head_pan_joint/topic',
        '/qbo_arduqbo/head_pan_joint/state'))
    head_tilt_joint_topic = str(rospy.get_param('~head_tilt_joint/topic',
        '/qbo_arduqbo/head_tilt_joint/state'))

    # subscribe to the topics, wiring callbacks into the controller
    rospy.loginfo('Subscribing to {0} for pan state'.format(head_pan_joint_topic))
    head_pan_sub = rospy.Subscriber(head_pan_joint_topic, MotorState,
            odom_controller.on_head_pan_joint)

    rospy.loginfo('Subscribing to {0} for tilt state'.format(head_tilt_joint_topic))
    head_tilt_sub = rospy.Subscriber(head_tilt_joint_topic, MotorState,
            odom_controller.on_head_tilt_joint)

    # launch the ROS event loop
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
