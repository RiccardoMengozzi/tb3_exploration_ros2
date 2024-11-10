# tb3_exploration_ros2
The project implement autonomous mapping through slam, localization and navigation of an amr, with the use of turtlebot3 navigation package. Then a "sanification" task is performed: the robot should autonomously navigate through the mapped environment to sanitize it in an efficient way. Read the report for further informations.

# Installation
The default ROS simulator is Gazebo. 

Install Gazebo 11
```console
 sudo apt-get install ros-humble-gazebo-*
```
 Install Cartographer
 ```console
sudo apt install ros-humble-cartographer
sudo apt install ros-humble-cartographer-ros
```
Install Navigation2
 ```console
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
```
Install TurtleBot3 Packages
 ```console
sudo apt install ros-humble-dynamixel-sdk
sudo apt install ros-humble-turtlebot3-msgs
sudo apt install ros-humble-turtlebot3
```

Install Turtlebot3 simulation packages
 ```console
mkdir -p ~/turtlebot3_ws/src/
cd ~/turtlebot3_ws/src/
git clone -b humble-devel \
https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/turtlebot3_ws
rosdep install -i --from-path src --rosdistro humble -y
colcon build --symlink-install
. install/setup.bash
```

# Mapping Usage
Open 4 terminals:<br />
<br />
!!! Be sure to source and to do a colcon build !!!
<br/>

Run Turtlebot3 Gazebo Simulation:
```console
export TURTLEBOT3_MODEL=burger
```
```console
ros2 launch turtlebot3_gazebo turtlebot3_big_house.launch.py
```

Run Turtlebot3 Cartographer:
```console
export TURTLEBOT3_MODEL=burger
```
```console
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True 
```

Run Turtlebot3 Navigation:
```console
export TURTLEBOT3_MODEL=burger
```
```console
ros2 launch turtlebot3_navigation2 navigation2.launch.py \use_sim_time:=True 
```
Change the rviz2 configuration for a better visualization: go to file -> Open config -> select amr_project_ws\extra_files\nav2_exploration_view.rviz

Run Exploration node:
```console
ros2 run exploration exploration
```

Save the map once you are satisfied with the result:
```console
ros2 run nav2_map_server map_saver_cli -f ~/map
```

# Localization Usage
Run Turtlebot3 Gazebo Simulation:
```console
ros2 launch turtlebot3_gazebo turtlebot3_big_house.launch.py
```

Run Turtlebot3 Navigation loading the previously saved map:
```console
export TURTLEBOT3_MODEL=burger
```
```console
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map.yaml
```

Run the Localization node:
```console
ros2 run sanification localization
```

# Sanitization Usage
After the robot has been localized, keeping gazebo simulation and navigation node open:<br />

Change rviz2 config to visualize the energy map: go to file -> open config and select: /home/mengo/amr_project_ws/src/sanification/files/sanification.rviz

Run the saniization node:
```console
ros2 run sanification sanification
```

Run the navigation node:
```console
ros2 run sanification navigation
```

Run the planner node giving as argument a number between 0 and 8 that identifies a specific room:
```console
ros2 run sanification planner
```

After the sanitization of the room has been completed, the planner can be runned again.

