# Anthony-Moundalak-ROS2-Assignment-Turtlebot3-Session7
# TurtleBot Navigation System

This project is a ROS 2-based system designed to navigate a TurtleBot3 robot, measure lap times, and interact with walls using custom services and actions.

## Packages

### 1. `turtlebot_navigation`

This package includes the main logic for controlling the TurtleBot3 robot, finding the closest wall, and measuring lap times.

#### Nodes

- **robot_driver.py**:
  - Controls the movement of the robot based on sensor data.
  - Subscribes to the `/scan` topic for laser scan data.
  - Publishes to the `/cmd_vel` topic to control the robot's velocity.
  - Calls the `find_closest_wall` service to determine the closest wall and navigate towards it.

- **wall_finder_service.py**:
  - Provides the `find_closest_wall` service to locate the nearest wall using laser scan data.
  - Moves the robot towards the closest wall.

- **lap_time_action_server.py**:
  - Implements the `MeasureLapTime` action server.
  - Tracks the robot's position using the `/odom` topic and measures lap times as the robot moves.

- **lap_time_action_client.py**:
  - Implements the `MeasureLapTime` action client.
  - Sends a goal to the `MeasureLapTime` action server and receives feedback and results on lap times.

#### Launch File

- **turtlebot_navigation_launch.py**:
  - Launches all nodes together: `robot_driver`, `wall_finder_service`, `lap_time_action_server`, and `lap_time_action_client`.

### 2. `my_interface`

This package defines custom ROS 2 service and action interfaces used in the `turtlebot_navigation` package.

#### Service

- **FindClosestWall.srv**:
  - Request: No request parameters.
  - Response: `bool success` - Indicates whether the robot successfully reached the closest wall.

#### Action

- **MeasureLapTime.action**:
  - Goal: No goal parameters.
  - Result: `float64 total_time` - The total time taken to complete all laps.
  - Feedback: `float64 elapsed_time` - The elapsed time for the current lap.

## Installation and Build

1. Clone the repository and navigate to your ROS 2 workspace.

2. Build the packages and source the setup script.

    ```bash
    source /opt/ros/humble/setup.bash
    colcon build
    source install/setup.bash
    ```
3. Run the launch file.

    ```bash 
    ros2 launch turtlebot_navigation my_robot_launch.py
    ```

