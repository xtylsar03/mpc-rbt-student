from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Spuštění naší Node s parametry
        Node(
            package='my_student_pkg',
            executable='my_node',
            name='kuzelna_studentska_node',
            parameters=[{
                'max_voltage': 42.0,
                'min_voltage': 36.0
            }]
        ),
        
        # Spuštění nahrávání všech topiců do ROS Bagu
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a', '-o', 'my_battery_bag'],
            output='screen'
        )
    ])
