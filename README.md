# EXPERIMENTAL ROBOTICS LABORATORY
# ASSIGNMENT 2

## How to Run from a Fresh Docker Container

### 1. Update and Upgrade apt
Run the following commands to update and upgrade the package manager:

```
apt update
apt upgrade -y --fix-missing
```

### 2. Install Missing Packages
Install the required packages using the commands below:

```
apt install gazebo_ros_pkgs
apt install ros-foxy-xacro
apt install ros-foxy-joint-state-publisher
apt install ros-foxy-gazebo-ros2-control
apt install ros-foxy-ros2-control
apt install ros-foxy-ros2-controllers
apt install ros-foxy-gazebo-ros-pkgs
```

### 3. Install Remaining Dependencies with rosdep
Navigate to the project directory and install the remaining dependencies:

```
cd /exp_assignment2
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro foxy
```

### 4. Build the Workspace
Build the workspace. If building fails on some systems, use the `--parallel-workers` flag:

```
cd /exp_assignment2
colcon build --symlink-install --parallel-workers 1
```

### 5. Run the Necessary Commands in Separate Terminals
Open separate terminals for each of the following commands. Follow the order and ensure each process initializes fully before moving to the next:

```
ros2 launch assignment2 gazebo.launch.py
ros2 launch slam_toolbox online_sync_launch.py
ros2 launch nav2_bringup navigation_launch.py
ros2 launch assignment2 required_nodes.launch.py
ros2 launch assignment2_cpp planner.launch.py
ros2 run plansys2_terminal plansys2_terminal
```

In the terminal running `plansys2_terminal`, copy and paste the following commands:

```
set instance robot1 robot
set instance w_start waypoint
set instance w0 waypoint
set instance w1 waypoint
set instance w2 waypoint
set instance w3 waypoint
set instance w_end waypoint
set predicate (robot_at robot1 w_start)
set goal (and(inspected w0)(inspected w1)(inspected w2)(inspected w3)(robot_at robot1 w_end))
```

You can view the generated plan with:

```
get plan
```

To begin the actions, execute:

```
run
```

The terminals running `required_nodes.launch.py` and `planner.launch.py` will provide real-time feedback.

## Important Considerations

- If Gazebo does not load properly, shows missing laser lines, or crashes, use the following commands to terminate and reset Gazebo:

```
killall gzserver
killall gzclient
```

- If any node encounters issues, terminate all processes using `Ctrl+C`, then relaunch or rerun the nodes in their respective terminals.
- It is implied that the user knows how to source the `setup.bash` from ROS2-FOXY into the `.bashrc` adding `source /opt/ros/foxy/setup.bash`. Later on, he would also need to include the `setup.bash` from the newly built workspace, probably requiring to add `source /root/exp_assignment2/install/setup.bash`.
