# arms-lab-school-workshop-2025
Supporting packages required during robotics outreach workshop for school students at ARMS lab, SysCon, IITB


### Steps to set up workshop packages in a fresh ubuntu 20 laptop:
----
- Download this repo and place the shell scripts under home directory of ubuntu. 
- Provide the shell scripts with executable permission `chmod +x ./*.sh`
- Run the shell scripts in following order for ROS and simulation package setup
  ```bash
  ./setup_ros_workspace.sh
  ```
  ```bash
  ./install_and_run_drone_hover_demo.sh
  ```

 - To run example simulation for drone hover:
  ```bash
  roslaunch hector_quadrotor_gazebo quadrotor_empty_world.launch
  ```
  ```bash
  rosrun hector_ui z_pid_gui.py
  ```
- Move the sliders to tune the controller and change setpoint.
