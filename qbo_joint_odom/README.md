## Calibration methodology

Use ``/cmd_joints`` to command a particular joint position. For example,

```console
$ rostopic pub -1 /cmd_joints sensor_msgs/JointState "[0,0,'test']" "['head_pan_joint','head_tilt_joint']" "[-0.785,0]" "[]" "[]"
```

will move the head to -45 degrees (-0.785 radians). Use the following to show the joint state:

```console
$ rostopic echo /qbo_arduqbo/head_pan_joint/state
```

Record the ``goal`` value. We can now use these values to compute scales and
offsets for the joints.
