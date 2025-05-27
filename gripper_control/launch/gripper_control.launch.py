import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='gripper_control',
            executable='gripper_control',
            name='gripper_control',
            output='screen',
            parameters=[{
                'cylinder_max_force': 2.0 * 9.81,
                'cube_max_force':     4.0 * 9.81,
                'slip_distance_threshold': 0.015,
            }],
        ),
    ])
