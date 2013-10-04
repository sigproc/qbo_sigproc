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
    Publish a tf from base_frame_id to head_frame_id when pan and tilt angles update.
    
    """

    def __init__(self, head_pan=None, head_tilt=None, base_frame_id=None, head_frame_id=None, head_offset=None):
        # The default is to have an identity mapping
        self.head_pan = head_pan or Mapping()
        self.head_tilt = head_tilt or Mapping()
        self.head_offset = np.asarray(head_offset or [0, 0, 0.3])

        # The default frame ids are 'base_link' and 'head'
        self.base_frame_id = base_frame_id or 'base_link'
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
                self.head_offset,
                tf.transformations.quaternion_from_matrix(pose),
                self.when, self.head_frame_id, self.base_frame_id)

def mapping_from_servo(param_base):
    """
    Initialise and return a Mapping object by reading ROS parameters from
    param_base. If param_base is /foo/bar then the following parameters are
    used:

        /foo/bar/invert
        /foo/bar/neutral
        /foo/bar/range
        /foo/bar/ticks

    If any of the parameters are unavailable, a warning message is logged an
    an identity mapping is returned.

    """
    rospy.loginfo('Extracting servo information from {0}'.format(param_base))

    try:
        invert = rospy.get_param(param_base + '/invert')
        rospy.loginfo('invert: {0}'.format(invert))
        neutral = rospy.get_param(param_base + '/neutral')
        rospy.loginfo('neutral: {0}'.format(neutral))
        range_ = rospy.get_param(param_base + '/range')
        rospy.loginfo('range: {0}'.format(range))
        ticks = rospy.get_param(param_base + '/ticks')
        rospy.loginfo('ticks: {0}'.format(ticks))
    except KeyError as e:
        rospy.logwarn("unable to get servo parameters from {0}:{1}".format(param_base, e))
        return Mapping()

    # Calculate scale and offset
    scale = float(range_) / float(ticks)
    offset = -float(neutral)
    if invert:
        scale *= -1.0

    rospy.loginfo('Computed mapping: scale={0}, offset={1}'.format(scale, offset))

    return Mapping(scale, offset)

def main():
    """
    Initialise the node, subscribe to topics and spin waiting for data.

    """
    # initialise our node
    rospy.init_node('qbo_joint_odom')

    # get the head offset
    head_offset = rospy.get_param('~head_offset', None)

    # get the frame ids
    head_frame_id = rospy.get_param('~head_frame_id', None)
    base_frame_id = rospy.get_param('~base_frame_id', None)

    # which namespace can we find parameters for the servos in?
    head_pan_joint_servo = str(rospy.get_param('~head_pan_joint/servo_ns',
        '/qbo_arduqbo/dynamixelservo/head_pan_joint'))
    head_tilt_joint_servo = str(rospy.get_param('~head_tilt_joint/servo_ns',
        '/qbo_arduqbo/dynamixelservo/head_tilt_joint'))

    head_pan_mapping = mapping_from_servo(head_pan_joint_servo)
    head_tilt_mapping = mapping_from_servo(head_tilt_joint_servo)

    # initialise the controller
    odom_controller = JointOdomController(
            head_pan = head_pan_mapping, head_tilt = head_tilt_mapping,
            head_frame_id = head_frame_id, base_frame_id = base_frame_id,
            head_offset = head_offset)

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
