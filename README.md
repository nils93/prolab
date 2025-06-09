# PRO_Lab

## 0. Requirements
Make sure that the package **turtlebot3_description** is available in your global ROS-Path (*/opt/ros/noetic/share*):
```bash
ls /opt/ros/noetic/share/turtlebot3_description
```
Otherwise install it:
```bash
sudo apt update
sudo apt install ros-noetic-turtlebot3* xterm
```

### Other Packages:
```bash
sudo apt install xterm ros-noetic-slam-gmapping
```

## 1. Clone the repo
We start the proejct by cloning the repo from GitHub:
```bash
git clone https://github.com/nils93/prolab.git turtle_ws && cd turtle_ws
source devel/setup.bash
catkin build
```

## 2. Environmental Variables
Make sure the env variable **TURTLEBOT3_MODEL** is set:
```bash
echo $TURTLEBOT3_MODEL
```
When it's not equal to **burger**, change it:
```bash
export TURTLEBOT3_MODEL=burger
```

## 3. Start **roscore**
```bash
source devel/setup.bash
roscore
```

## 4. Record a map
Secondly, we launch the turtlebot from the **example_package**:
```bash
source devel/setup.bash
roslaunch example_package start.launch
```
In einem weiteren Terminal:
```bash
source devel/setup.bash
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
``` 
Note: Für den Fall, dass das nachstehende Warning kommt:
```bash
[WARN] [1748099231.200285610, 1328.300000000]: MessageFilter [target=odom ]: Dropped 100.00% of messages so far. Please turn the [ros.gmapping.message_filter] rosconsole logger to DEBUG for more information.
```
Einfach den Befehl killen und noch mal starten.

## Falls es mal hängt: Kill All!
```bash
killall -9 roscore rosmaster rosout gzserver gzclient
catkin clean -y
```

## 6. First Filter implemented: EKF
```bash
header: 
  seq: 12806
  stamp: 
    secs: 1712
    nsecs: 540000000
  frame_id: "map"
pose: 
  pose: 
    position: 
      x: 0.335299812381564
      y: -0.008349527280484794
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: -0.9943416899982722
      w: 0.10622901454583862
  covariance: [0.013686038362359095, -0.0024724910473428773, -0.00048270061444676644, 0.0, 0.0, 0.0, -0.0024724910473428725, 0.010599706078336685, 0.00014468562373250304, 0.0, 0.0, 0.0, -0.00048270061444676704, 0.00014468562373250271, 0.008091739755542917, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---
```

7. Comparison between `/ekf_node/ekf_pose` and `/gazebo/model_states`
```bash
cd ~/turtle_ws/src/ekf_node/scripts/ekf_analysis/
rosbag record /ekf_node/ekf_pose /gazebo/model_states
./main.py 
```