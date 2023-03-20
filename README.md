# Reeds-Shepp-ROS
This is a ROS adaptation of the Reeds-Shepp path planning algorithm. The original code can be found [here](https://github.com/liespace/pyReedsShepp).

# Installation
```
cd ~/catkin_ws/src
git clone https://github.com/SailorBrandon/reeds-shepp-ROS.git
cd .. && catkin_make
source devel/setup.bash
```

# Usage
```
roslaunch rs_planner try_rs_planner.launch
```