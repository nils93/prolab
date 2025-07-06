#!/bin/bash

# 1. Catkin build (wartet, bis fertig)
echo "Starte catkin build..."
cd ~/turtle_ws && catkin build
if [ $? -ne 0 ]; then
  echo "❌ catkin build fehlgeschlagen!"
  exit 1
fi

# 2. Starte roscore im Hintergrund
echo "Starte roscore..."
gnome-terminal -- bash -c "roscore; exec bash"
sleep 2  # Kurze Pause für roscore

# 3. Starte example_package
echo "Starte example_package..."
gnome-terminal -- bash -c "source ~/turtle_ws/devel/setup.bash && roslaunch example_package start_v1.launch; exec bash"
sleep 5

# 4. Starte filter_node
echo "Starte filter_node..."
gnome-terminal -- bash -c "source ~/turtle_ws/devel/setup.bash && roslaunch filter_node start.launch; exec bash"

# 5. Shutdown
echo "Warte auf ENTER zum Beenden aller ROS-Prozesse..."
read -p "Drücke [ENTER], um alles zu stoppen..."

echo "❌ Beende ROS-Prozesse..."
killall -q roscore
killall -q roslaunch
killall -q rosmaster
killall -q gzserver gzclient
