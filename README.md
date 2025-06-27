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
When it's not equal to **burger_for_autorace**, change it:
```bash
export TURTLEBOT3_MODEL=burger_for_autorace
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

## 6. Create the catkin
```bash
catkin build
```

## 7. Start the nodes
Terminal 1
```bash
roscore
```
Terminal 2
```bash
roslaunch example_package start.launch
```
Terminal 3
```bash
roslaunch filter_node filter.launch
```

## 8. Record the filter performance
Terminal 4
```bash
cd ~/turtle_ws/scripts/analyse/kf/
rosbag record /kf_node/kf_pose /gazebo/model_states
```
Terminal 5
```bash
cd ~/turtle_ws/scripts/analyse/ekf/
rosbag record /ekf_node/ekf_pose /gazebo/model_states
```
Terminal 6
```bash
cd ~/turtle_ws/scripts/analyse/pf/
rosbag record /pf_node/pf_particles /gazebo/model_states
```

## 9. Analyse the filter performance
Terminal 7
Start the script with the desired ARG (kf, ekf, pf)
```bash
cd ~/turtle_ws/scripts/analyse
./main.py --filter ARG
```