from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='BurgerRobot_Simulation',
            executable='localizer',
            name='burger_localizer',
            output='screen',
        ),
        Node(
            package='BurgerRobot_Simulation',
            executable='sensor_bridge',
            name='sensor_bridge',
            output='screen',
            parameters=[{'max_range': 3.5}],
        ),
        Node(
            package='BurgerRobot_Simulation',
            executable='planner',
            name='burger_planner',
            output='screen',
            parameters=[{
                'kp_lin': 0.5,
                'kp_ang': 1.5,
                'max_lin': 0.22,
                'max_ang': 2.84,
                'goal_tolerance': 0.10,
            }],
        ),
        Node(
            package='BurgerRobot_Simulation',
            executable='controller',
            name='burger_controller',
            output='screen',
            parameters=[{
                'publish_rate_hz': 10.0,
                'obstacle_min_range': 0.30,
                'avoid_turn_speed': 0.5,
                'stop_on_obstacle': True,
            }],
        ),
    ])
