import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_dir = get_package_share_directory('mpc_rbt_student')
    rviz_config_path = os.path.join(package_dir, 'rviz', 'config.rviz')
    
    # 1. Tvoje existující lokalizace
    localization_node = Node(
        package='mpc_rbt_student',
        executable='localization',
        name='localization_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # 2. Tvůj existující RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 3. NOVÉ: Warehouse Manager (simulátor skladu)
    warehouse_manager = Node(
        package='mpc_rbt_student', # Opraveno na tvůj balíček
        executable='warehouse_manager',
        name='warehouse_manager',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 4. NOVÉ: Behavior Tree Server (náš hlavní mozek)
    bt_server = Node(
        package='mpc_rbt_student', # Opraveno na tvůj balíček
        executable='bt_server',
        name='bt_server',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            os.path.join(package_dir, 'config', 'bt_server.yaml')
        ]
    )

    # Všechny 4 uzly spustíme najednou
    return LaunchDescription([
        localization_node,
        rviz_node,
        warehouse_manager,
        bt_server
    ])
