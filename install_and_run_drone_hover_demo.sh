source ~/.bashrc
echo "download package dependencies"
sudo apt-get install ros-noetic-gazebo-plugins ros-noetic-gazebo-ros-control


echo "=== Drone Hover Demo (hector_quadrotor) Setup Script ==="

# Set your GitHub source URL here
HECTOR_REPO_URL="https://github.com/RAFALAMAO/hector-quadrotor-noetic.git"
GEO_INFO_REPO_URL="https://github.com/ros-geographic-info/geographic_info.git"
UUID_REPO_URL="https://github.com/ros-geographic-info/unique_identifier.git"
TELEOP_URL="https://github.com/ros-teleop/teleop_twist_keyboard.git"

# 1. Clone hector_quadrotor packages
cd ~/catkin_ws/src
echo "--- Cloning hector_quadrotor from GitHub..."
git clone "$GEO_INFO_REPO_URL"
git clone "$UUID_REPO_URL"
git clone "$TELEOP_URL"

# Build the workspace
cd ~/catkin_ws
echo "--- Building workspace..."
catkin_make
cd ~/catkin_ws/src
git clone "$HECTOR_REPO_URL"
git clone "https://github.com/krutarth3ved/arms-lab-school-workshop-2025.git"
mv arms-lab-school-workshop-2025/workshop/ ./workshop
chmod +x workshop/src/scripts/*.py
cp arms-lab-school-workshop-2025/z_pid_gui.py hector-quadrotor-noetic/hector_ui/src/
chmod +x hector-quadrotor-noetic/hector_ui/src/z_pid_gui.py

# 3. Install missing dependencies
echo "--- Installing dependencies using rosdep..."
cd ~/catkin_ws
#rosdep update
#rosdep install --from-paths src --ignore-src -r -y

# 4. Build the workspace
echo "--- Building workspace..."
catkin_make

# 5. Source setup.bash
source ~/.bashrc
source devel/setup.bash

# 6. Install GUI tools
echo "--- Installing RQT GUI tools..."
sudo apt update
#sudo apt install -y ros-noetic-rqt-reconfigure

# 7. Launch Simulation and Tools
echo "--- Launching simulation and GUI tools..."

gnome-terminal --title="Gazebo Drone Sim" -- bash -c "source ~/.bashrc && roslaunch hector_quadrotor_gazebo quadrotor_empty_world.launch"
sleep 10

#gnome-terminal --title="RQT Reconfigure" -- bash -c "source ~/.bashrc && rosrun rqt_reconfigure rqt_reconfigure"

echo ""
echo "=== Setup Complete ==="
echo "Use the RQT tools to control and tune the drone."
