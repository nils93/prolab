# prolab

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
```

## 5. Ausgaben der Filter topics
```bash
focal@focal:~/turtle_ws$ rostopic echo /kf_filter/odom
header: 
  seq: 2196
  stamp: 
    secs: 1974
    nsecs: 520000000
  frame_id: "odom"
child_frame_id: "base_footprint"
pose: 
  pose: 
    position: 
      x: 0.31131548450910285
      y: 0.012277796424420282
      z: 0.049350379684697826
    orientation: 
      x: -0.001248069024348646
      y: 0.001907079160343123
      z: 0.9943613742529659
      w: 0.10602010548259229
  covariance: [1e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001]
twist: 
  twist: 
    linear: 
      x: -0.001987716961708169
      y: 0.0016246141608199747
      z: 0.0
    angular: 
      x: 0.0
      y: 0.0
      z: 0.030168834824700612
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---
```
```bash
focal@focal:~/turtle_ws$ rostopic echo /ekf_filter/odom
header: 
  seq: 5435
  stamp: 
    secs: 2082
    nsecs: 480000000
  frame_id: "odom"
child_frame_id: "base_footprint"
pose: 
  pose: 
    position: 
      x: 0.37245540280220174
      y: 0.014301139994589615
      z: 0.04958603089494515
    orientation: 
      x: 0.0018228338030330186
      y: 0.002904276685731321
      z: -0.8320165735834772
      w: 0.5547401767821303
  covariance: [1e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001]
twist: 
  twist: 
    linear: 
      x: 0.0003157882533193501
      y: 0.003306798832405872
      z: 0.0
    angular: 
      x: 0.0
      y: 0.0
      z: 0.07510671539852391
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---
```
```bash
focal@focal:~/turtle_ws$ rostopic echo /pf_filter/odom
header: 
  seq: 6851
  stamp: 
    secs: 2129
    nsecs: 680000000
  frame_id: "odom"
child_frame_id: "base_footprint"
pose: 
  pose: 
    position: 
      x: nan
      y: nan
      z: 0.04935611077519547
    orientation: 
      x: 0.0006394853024721342
      y: 0.00227606088465104
      z: 0.6078538939022572
      w: 0.7940453729310819
  covariance: [1e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001]
twist: 
  twist: 
    linear: 
      x: -0.0019784797731067206
      y: 0.004850601132048276
      z: 0.0
    angular: 
      x: 0.0
      y: 0.0
      z: 0.07855743069123798
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---
```