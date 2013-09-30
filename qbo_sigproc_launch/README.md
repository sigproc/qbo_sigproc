## Launch files for SigProC Q.bo

### Simple usage

To bring the Q.bo up with sensors and frames for the various cameras:

```console
(on your machine)
$ roslaunch qbo_sigproc_launch qbo.launch
```

### Advanced usage

The ``qbo.launch`` file is a simple wrapper which arranges for the nodes
included to be launched on the Q.bo. The meat of the work is done in
``launch/include/bringup_with_sensors.launch``.

## Setup notes for Q.bo

These launch files assume that you have a [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
set up in the home directory on the Q.bo under ``~/cued-masters/``. This could
be set as a parameter should anyone else need to use the Q.bo. If you are
installing packages locally on the Q.bo then please install into that
workspace and document on the wiki.
