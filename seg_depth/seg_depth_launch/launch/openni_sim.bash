 #!/bin/bash
echo "Run bag file after this"
rosparam set /use_sim_time true
roslaunch seg_depth openni_sim.launch
