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

## Arguments for launch file

The main ``qbo.launch`` file accepts a number of arguments. Set arguments on
the command line when launching as follows:

```console
$ roslaunch qbo_sigproc_launch qbo.launch arg1:=value arg2:=value
```

### Q.bo networking

The ``qbo_address`` and ``qbo_user`` specify the hostname of the Q.bo and the username to use to log in:

**qbo\_address**: default ``sigproc-robot1``, usual values are ``sigproc-robot1`` or ``binky``.

**qbo\_username**: default ``qbo``

### Features

The following arguments can be set to ``true`` or ``false`` to enable or
disable certain features. The default is always ``false``.

**speech\_recognition**: if ``true``, launch the ``qbo_listen`` node

**speech\_synthesis**: if ``true``, launch the ``qbo_talk`` node

**cameras**: if ``true``, enable the eye cameras (default is monocular vision on the left eye)

**stereo**: if ``true``, enable both eye cameras (FIXME: currently broken)

**rqt_console**: if ``true``, bring up a graphical debug console

**rviz**: if ``true``, bring up rviz

**depth\_camera**: if ``true``, bring up the depth camera and associated processing nodes

**face**: (EXPERIMENTAL) if ``true``, bring up face tracking and following

**joypad**: if ``true``, enable wirless xbox 360 joypad control

### Camera parameters

**mono\_camera\_side**: default ``left``, possible values are ``left`` or
``right``. Which side should the monocular camera publish data to. I.e. should
the images appear as ``/stereo/left/image_raw`` or ``/stereo/right/image_raw``.

**mono\_camera\_device**: default ``/dev/video0``, usual values are
``/dev/video0`` or ``/dev/video1``. This specifies which video4linux device is
used to provide the camera feed. Note that the left camera and right camera
will have different devices.

### Extra includes

**extra\_include**: if this is set then the launch file
``path/to/qbo_sigproc_launch/launches/include/<value>.launch`` will also be
included with ``<value>`` replaced by the value of the ``extra_include``
argument. Note that you don't specify the trailing ``.launch``.

The ``face`` extra will launch the face tracking and following system. Let Q.bo
follow you around the room.

## Setup notes for Q.bo

These launch files assume that you have a [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
set up in the home directory on the Q.bo under ``~/cued-masters/``. This could
be set as a parameter should anyone else need to use the Q.bo. If you are
installing packages locally on the Q.bo then please install into that
workspace and document on the wiki.
