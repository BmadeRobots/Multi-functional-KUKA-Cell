#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world', default='empty.sdf')
    
    # Set Gazebo resource path
    pkg_share_path = get_package_share_directory("kuka_iontec_support")
    gazebo_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if gazebo_resource_path:
        gazebo_resource_path += ':' + pkg_share_path
    else:
        gazebo_resource_path = pkg_share_path
    
    # Robot description using the final URDF
    urdf_file_path = os.path.join(
        get_package_share_directory("kuka_iontec_support"),
        "urdf",
        "kr70_absolute_paths.urdf"
    )
    
    with open(urdf_file_path, 'r') as infp:
        robot_description_content = infp.read()
    
    robot_description = {"robot_description": robot_description_content}

    # Launch Gazebo Harmonic
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': ['-r -v4 ', world_name],
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Robot state publisher for Robot 1
    robot_state_publisher_1 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher_1",
        namespace="robot1",
        output="log",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
        remappings=[
            ('/robot_description', '/robot1/robot_description'),
            ('/joint_states', '/robot1/joint_states'),
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
        ]
    )

    # Robot state publisher for Robot 2
    robot_state_publisher_2 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher_2",
        namespace="robot2",
        output="log",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
        remappings=[
            ('/robot_description', '/robot2/robot_description'),
            ('/joint_states', '/robot2/joint_states'),
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
        ]
    )

    # Spawn Robot 1 at origin
    spawn_robot_1 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot1/robot_description',
            '-name', 'kr70_robot1',
            '-x', '0.0',
            '-y', '-1.0', 
            '-z', '0.1'
        ],
        output='screen'
    )

    # Spawn Robot 2 at 1 meter distance
    spawn_robot_2 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot2/robot_description',
            '-name', 'kr70_robot2',
            '-x', '0.0',  # 1 meter away in X direction
            '-y', '3.0', 
            '-z', '0.1'
        ],
        output='screen'
    )

    # Joint state publisher GUI for Robot 1
    joint_state_publisher_gui_1 = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui_1",
        namespace="robot1",
        output="log",
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[
            ('/joint_states', '/robot1/joint_states'),
        ]
    )

    # Joint state publisher GUI for Robot 2
    joint_state_publisher_gui_2 = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui_2",
        namespace="robot2",
        output="log",
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[
            ('/joint_states', '/robot2/joint_states'),
        ]
    )

    # Bridge between ROS2 and Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/robot1/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/robot2/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        output='screen'
    )

    return LaunchDescription([
        # Set environment variable for Gazebo to find meshes
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=gazebo_resource_path
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='empty.sdf',
            description='Gazebo world file'
        ),
        gazebo,
        robot_state_publisher_1,
        robot_state_publisher_2,
        spawn_robot_1,
        spawn_robot_2,
        joint_state_publisher_gui_1,
        joint_state_publisher_gui_2,
        bridge,
    ])
