# UR12e + Arduino gripper project

ROS 2 Jazzy on Ubuntu 24.04 / Linux Mint 22.

## Layout
- `arduino/` Arduino sketches
- `arduino_test_ws/` test workspace: button press moves the simulated UR
- `group_project_ws/` the real project workspace

## Setup
```bash
sudo apt install ros-jazzy-ur python3-serial ros-dev-tools docker.io
sudo usermod -aG dialout,docker $USER   # then log out and back in
cd arduino_test_ws
colcon build --symlink-install
source install/setup.bash
```

## Run the test (four terminals)
1. URSim: `docker run -it --net host -e ROBOT_MODEL=UR12 --name ursim universalrobots/ursim_e-series`
2. Driver: `ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur12e robot_ip:=127.0.0.1 launch_rviz:=false`
3. Bridge: `ros2 run button_test arduino_bridge`
4. Control: `ros2 run button_test control`

Flash `arduino/button_toggle` first. Button on pin 2, LED on pin 4.