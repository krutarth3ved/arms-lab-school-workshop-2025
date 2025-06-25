#!/bin/bash

set -e

echo "=== Starting ROS Noetic + TurtleBot3 + Gazebo Setup ==="

# 1. System update and basic tools
echo "--- Updating system and installing required tools..."
sudo apt update && sudo apt upgrade -y
sudo apt install -y curl gnupg lsb-release

# 2. ROS repository and keys
echo "--- Adding ROS repository and keys..."
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -


# 3. Install ROS Noetic
echo "--- Installing R OS Noetic Desktop Full..."
sudo apt update
sudo apt install -y ros-noetic-desktop-full python3-rosdep2 ros-noetic-rosbash


# 4. Initialize rosdep
# echo "--- Initializing rosdep..."
# sudo rosdep init || echo "rosdep already initialized"
# rosdep update

# 5. Bashrc configuration for ROS
if ! grep -Fxq "source /opt/ros/noetic/setup.bash" ~/.bashrc; then
  echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
fi
source /opt/ros/noetic/setup.bash

# 6. Development tools
echo "--- Installing ROS development tools..."
sudo apt install -y python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-rosdep python3-catkin-tools rviz
echo "--- Initializing rosdep..."
sudo rosdep init
rosdep update

# 7. Catkin workspace setup
echo "--- Creating catkin workspace..."
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make

if ! grep -Fxq "source ~/catkin_ws/devel/setup.bash" ~/.bashrc; then
  echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
fi
source ~/catkin_ws/devel/setup.bash

# 8. TurtleBot3 and Gazebo packages
echo "--- Installing TurtleBot3 and simulation packages..."
sudo apt install -y ros-noetic-turtlebot3 ros-noetic-turtlebot3-simulations
#sudo apt-get install ros-noetic-gazebo-plugins
# 9. TurtleBot3 model
if ! grep -Fxq "export TURTLEBOT3_MODEL=burger" ~/.bashrc; then
  echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
fi
export TURTLEBOT3_MODEL=burger

# 10. Clone your custom packages (optional)
echo "--- (Optional) Cloning custom packages into catkin_ws/src..."
cd ~/catkin_ws/src

# Example: Replace or uncomment these lines
# git clone https://github.com/yourusername/your_custom_pkg.git
# cp -r /media/usb/my_package .

# 11. Build workspace
cd ~/catkin_ws
catkin_make

# 12. Final verification
echo "--- Verifying installation ---"
ros_version=$(rosversion -d || echo "not found")
gazebo_version=$(gazebo --version | head -n1 || echo "not found")

echo "ROS Version       : $ros_version"
echo "Gazebo Version    : $gazebo_version"
echo "TurtleBot3 Model  : $TURTLEBOT3_MODEL"

# 13. Run talker/listener test (brief)
echo "--- Testing basic ROS nodes (talker/listener)..."
gnome-terminal -- bash -c "roscore" &
sleep 5
gnome-terminal -- bash -c "source ~/catkin_ws/devel/setup.bash && rosrun rospy_tutorials talker" &
sleep 3
gnome-terminal -- bash -c "source ~/catkin_ws/devel/setup.bash && rosrun rospy_tutorials listener" &
sleep 5

echo "--- Stopping ROS test nodes..."
pkill -f roscore || true
pkill -f talker || true
pkill -f listener || true
sleep 2

# 14. Run Gazebo TurtleBot3 simulation (brief)
echo "--- Testing TurtleBot3 Gazebo launch..."
gnome-terminal -- bash -c "source ~/catkin_ws/devel/setup.bash && export TURTLEBOT3_MODEL=burger && roslaunch turtlebot3_gazebo turtlebot3_world.launch" &
sleep 15
pkill -f turtlebot3_world.launch || true
pkill -f gzserver || true
pkill -f gzclient || true

echo ""
echo "=== ROS Environment Setup & Basic Tests Complete ==="
echo "ROS Version       : $ros_version"
echo "Gazebo Version    : $gazebo_version"
echo "TurtleBot3 Model  : $TURTLEBOT3_MODEL"
echo ""
echo "You can now start developing in ~/catkin_ws."
echo "Restart your terminal or run: source ~/.bashrc"
