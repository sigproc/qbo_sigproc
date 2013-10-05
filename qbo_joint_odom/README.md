# Joint odometry

Subscribe to Q.bo joint sensor messages and publish tf frames for join pose and
position. Specifically, head pan and tilt messages are converted.

This requires your Q.bo be launched with the ``dynamixelservo`` support.

## The qbo\_joint\_odom.py node
Publish a tf transform for the head.

### Published transforms

*base_frame_id* → *head_frame_id*: a frame for the head. The x-axis points out
through the nose and the y-axis points out through the side of the head.

### Parameters

**~head\_frame\_id** (string, default: 'head'): frame id to publish head pose to.

**~base\_frame\_id** (string, default: 'base\_link'): parent frame id of head pose.

**~head\_offset** (float[3], default: [0,0,0.3]): the offset from
*base_frame_id* to the head pivot point.

**~head\_pan\_joint/topic** (string, default:
'/qbo_arduqbo/head\_pan\_joint/state'): topic to subscribe to to get updates on
head pan. Expects a qbo\_arduqbo::motor\_state message.

**~head\_pan\_joint/servo\_ns** (string, default:
'/qbo_arduqbo/dynamixelservo/head\_pan\_joint'): base namespace for parameters
specifying joint position encoding for the pan joint. (See servo encoding below.)

**~head\_tilt\_joint/topic** (string, default:
'/qbo_arduqbo/head\_tilt\_joint/state'): topic to subscribe to to get updates on
head tilt. Expects a qbo\_arduqbo::motor\_state message.

**~head\_tilt\_joint/servo\_ns** (string, default:
'/qbo_arduqbo/dynamixelservo/head\_tilt\_joint'): base namespace for parameters
specifying joint position encoding for the tilt joint. (See servo encoding below.)

### Servo encoding

The node will attempt to configure the mapping from servo encoding values to
actual degrees via the same ROS parameters which are used to configure the
``qbo_arduqbo`` node itself. Specifically for each servo, the following
parameters are queried:

- *servo\_ns*/invert: boolean indicating if the angle of the servo is inverted
  with respect to the encoder value.
- *servo\_ns*/neutral: the encoded value when the servo is at the neutral, or
  central, position.
- *servo\_ns*/ticks: the range of the servo in encoder values.
- *servo\_ns*/range: the range of the servo in degrees.

For the head pan and tilt servos, *servo\ns* is replaced as appropriate by the
values of the ~head\_pan\_joint/servo\_ns and ~head\_tilt\_joint/servo\_ns
parameters.

