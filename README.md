# ROS2 Jazzy xacro URDF test

Reference: https://www.youtube.com/watch?v=_F8wVuiEmww&ab_channel=AleksandarHaberPhD

## Setup environment

Update apt package manager
```bash
sudo apt update && sudo apt upgrade
source /opt/ros/jazzy/setup.bash
```

Install dependencies
```bash
sudo apt install ros-jazzy-robot-state-publisher
sudo apt install ros-jazzy-rviz2
sudo apt install ros-jazzy-xacro
sudo apt install ros-jazzy-ros2-control
sudo apt install ros-jazzy-ros2-controllers
sudo apt install ros-jazzy-gz-ros2-control
sudo apt install ros-jazzy-gz-ros2-control-demos
```

Make the project files by cloning the repo
```bash
mkdir -p ~/jazzy_ws/src
cd ~/jazzy_ws/src
git https://github.com/bit-magpie/ros2_jazzy_urdf_test.git
```

<details>
<summary>Make directory list if you want to create the project from the scratch</summary>

```bash
mkdir -p ~/jazzy_ws/src
cd ~/jazzy_ws/src
ros2 pkg create --build-type ament_cmake demo_control
cd demo_control
mkdir launch model config
```
</details>

Compile and install the package
```bash
cd ~/jazzy_ws/
sudo rosdep init
rosdep update
rosdep install -i --from-path src --rosdistro jazzy -y
colcon build --symlink-install --packages-select demo_control
```

If the package is compiled successfully, use following commands to launch the project
```bash
source install/setup.bash
ros2 launch demo_control display.launch.py
```

After Rviz is launched add `RobotModel` and set the `Description topic`to `/robot_description`.