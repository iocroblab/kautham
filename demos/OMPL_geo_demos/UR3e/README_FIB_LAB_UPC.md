# KAUTHAM - FIB LAB (UPC)

This README explains in detail how to use [Kautham GUI](#kautham-gui) and [Kautham ROS](#kautham-ros) (ROS2) to test it with the real robot (UR3e) from the FIB LAB (UPC).


## KAUTHAM GUI
To open the Kautham GUI, use the following command:
```
./apps/graphical/kautham-gui
```

Open the problem that you want to use and solve the problem using GetPath button.


## KAUTHAM ROS
First, create a workspace:
```
mkdir -p ws_kautham/src
```
Then, clone the required repositories into the /src folder:
```
kautham
kautham_ros
```

Finally, compile the workspace from its root folder /ws_kautham:
```
colcon build --symlink-install
```

Before using the code, setup the installation (into each terminal that you are using):
```
source install/setup.bash
```

To launch the kautham_node using ROS2, execute the following command in the terminal:
```
ros2 launch kautham_ros kautham_ros_node.launch.py
```

Create a custom client or open another terminal to interactuate with the kautham_node:

### Terminal example:
1. Open the problem:
```
ros2 service call /kautham_node/OpenProblem kautham_ros_interfaces/srv/OpenProblem "problem: '<full_path_problem_file>' dir: [<full_path_models_folder>]" 
```
2. Visualize the opened problem into RVIZ:
```
ros2 service call /kautham_node/visualizeRVIZ kautham_ros_interfaces/srv/KauthamRVIZ "rviz_config_file_path: '<full_path_rviz_file>' rviz_freq: <freq_hz>" 
```
3. Solve the problem:
```
ros2 service call /kautham_node/Solve kautham_ros_interfaces/srv/Solve "{}" 
```
4. Visualize the solved path motion:
```
ros2 service call /kautham_node/visualizePathMotionRVIZ kautham_ros_interfaces/srv/VisualizePathMotionRVIZ "run_motion: <true/false>
sampling_time_ms: <milliseconds>"
```

### Client node example:
Code the client to:
1. Open the problem.
2. Set the initial query based on the current state of the robot.
3. Solve the problem.
4. Request the trajectory.
5. Sent it to the robot as a follow joint trajectory.