import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Lấy đường dẫn URDF
    pkg = get_package_share_directory('step_01_robot_description')
    urdf_file = os.path.join(pkg, 'xacro_agv', 'agv_urdf.xacro')

    # Xử lý xacro
    robot_desc = Command(['xacro ', urdf_file, ' use_gazebo:=true'])

    # Launch argument để dùng sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([

        # Declare sim_time
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock'
        ),

        # Launch Gazebo
        ExecuteProcess(
            cmd=['ros2', 'launch', 'ros_gz_sim', 'gz_sim.launch.py', 'gz_args:=-r empty.sdf'],
            output='screen'
        ),

        # Bridge cho Lidar nếu có
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/lidar_sensor/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
            remappings=[('/lidar_sensor/scan', '/scan')],
            output='screen'
        ),

        # robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc, 'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # Spawn robot vào Gazebo
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_sim', 'create', '-name', 'agv', '-string', robot_desc],
            output='screen'
        ),

        # Load controllers bằng CLI (Jazzy không có spawner.py)
        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', '/diff_cont'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', '/joint_broad'],
            output='screen'
        ),

        # RViz để debug (nếu cần)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
