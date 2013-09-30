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
