# ROS_FKY

ROS2 burger robot simulation project with navigation, localization, and path planning capabilities.

## Prerequisites

- Ubuntu with ROS2 installed
- WSL (Windows Subsystem for Linux)
- TurtleBot3 packages installed
- Gazebo simulator

## Running the Simulation

To run the complete simulation, you need to open **3 separate WSL terminal windows** and execute the commands in order:

### Terminal 1: Launch Gazebo Simulation

This terminal starts the Gazebo simulator with the TurtleBot3 Burger model in a pre-built world.

```bash
killall -9 gz ruby gzserver gzclient 2>/dev/null; sleep 2; export TURTLEBOT3_MODEL=burger && ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Wait for Gazebo to fully load** before proceeding to Terminal 2.

### Terminal 2: Launch BurgerRobot Simulation

This terminal starts the robot's navigation and control nodes.

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch BurgerRobot_Simulation simulation_bringup.launch.py
```

**Wait for all nodes to initialize** before proceeding to Terminal 3.

### Terminal 3: Send Goal Position

This terminal sends a goal position to the robot (x=2.0, y=2.0).

```bash
cd ~/ros2_ws
source install/setup.bash
python3 send_goal.py 2.0 2.0
```

The robot should now navigate to the specified goal position.

## Notes

- Make sure to execute the commands in the order specified
- Ensure each terminal has fully initialized before moving to the next step
- You can modify the goal coordinates in Terminal 3 by changing the values (e.g., `python3 send_goal.py 3.0 1.5`)
- To stop the simulation, press `Ctrl+C` in each terminal window