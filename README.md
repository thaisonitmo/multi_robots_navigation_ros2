# Multiple mobile robots Navigation in ROS2

Configure navigation for muilti-robot environments, use the ROS2 Navigation (Nav2) package to make a robot autonomously navigate. Two robots "Turtlebot3" are used to controlled to the goal position. They can find themself the shortest path, and can avoid each other and obstacles in the environment.

## Tasks

 - Build a map of the environment
 - Localize a robot in a map of the environment
 - Path Planning from an initial position to the desired goal
 - Obstacle avoidance using Costmaps
 - Navigation Lifecycle Manager
 - Behavior Trees influence Nav2

## Installation

Run the docker file:

In one terminal, run docker and some commands to open the Gazebo window:
```bash
bash docker/run.bash
```
```bash
colcon build
source install/setup.bash
ros2 launch localization_server localization.launch.py 
```
In another terminal, open RViz window:
```bash
bash docker/into.bash
ros2 launch path_planner_server multi_main.launch.xml 
```
## Usage
In the RViz enviroment, pick a target location for Turtlebot on the map. We can send Turtlebot 3 a goal position and a goal orientation by using the Nav2 Goal or the GoalTool buttons.

Once we define the target pose, Nav2 will find a global path and start navigating the robot on the map.

![alt text](https://github.com/thaisonitmo/multi_robots_navigation_ros2/blob/main/img/Screenshot%20from%202025-06-28%2003-07-58.png)

Two Turtlebot 3 robots can avoid each other and obstacles in the environment and move towards the goal position. 
