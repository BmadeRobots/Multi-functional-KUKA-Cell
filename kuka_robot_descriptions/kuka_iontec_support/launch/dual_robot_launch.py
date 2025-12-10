#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world', default='empty.sdf')
    paused = LaunchConfiguration('paused', default='true')
    
    # Set Gazebo resource path
    pkg_share_path = get_package_share_directory("kuka_iontec_support")
    gazebo_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if gazebo_resource_path:
        gazebo_resource_path += ':' + pkg_share_path
    else:
        gazebo_resource_path = pkg_share_path
    
    # Robot description using the correct dual robot URDF
    urdf_file_path = os.path.join(
        get_package_share_directory("kuka_iontec_support"),
        "urdf",
        "dual_kr60_r2100.urdf"  # Fixed: Changed from kr60 to kr60
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
            'gz_args': ['-v4 ', world_name],  # Removed -r flag to start paused
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Single robot state publisher for the dual robot system
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="log",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
        remappings=[
            ('/joint_states', '/joint_states'),
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
        ]
    )

    # Spawn the dual robot system (single spawn for both robots)
    spawn_dual_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'dual_kr60_system',  # Fixed: Changed from kr60 to kr70
            '-x', '0.0',
            '-y', '0.0', 
            '-z', '0.1'
        ],
        output='screen'
    )

    # Controller manager for the dual robot system
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            os.path.join(
                get_package_share_directory("kuka_iontec_support"),
                "config",
                "dual_kr60_controllers.yaml"  # Fixed: Changed from kr60 to kr70
            ),
            {"use_sim_time": use_sim_time}
        ],
        output="screen",
    )

    # Spawn controllers for left robot
    spawn_left_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_joint_state_broadcaster"],
        output="screen",
    )

    spawn_left_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_arm_controller"],
        output="screen",
    )

    spawn_left_rail_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_rail_controller"],
        output="screen",
    )

    # Spawn controllers for right robot
    spawn_right_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_joint_state_broadcaster"],
        output="screen",
    )

    spawn_right_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_arm_controller"],
        output="screen",
    )

    spawn_right_rail_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_rail_controller"],
        output="screen",
    )

    # Option A: Spawn individual DKP controllers (recommended for your current URDF structure)
    spawn_dkp1_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["dkp1_joint_state_broadcaster"],
        output="screen",
    )

    spawn_dkp1_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["dkp1_controller"],
        output="screen",
    )

    spawn_dkp2_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["dkp2_joint_state_broadcaster"],
        output="screen",
    )

    spawn_dkp2_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["dkp2_controller"],
        output="screen",
    )

    # Option B: Spawn combined DKP controllers (only use if you modify URDF to single dkp_system)
    # Comment out the individual DKP spawners above and uncomment these if using combined approach
    # spawn_dkp_joint_state_broadcaster = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["dkp_joint_state_broadcaster"],
    #     output="screen",
    # )

    # spawn_dkp_controller = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["dkp_controller"],
    #     output="screen",
    # )

    # Joint state publisher GUI for manual control (with sliders for all joints)
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="log",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time},
            {"rate": 50}
        ],
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    # Bridge between ROS2 and Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            # Add specific joint state bridges if needed
        ],
        output='screen'
    )

    # RViz for visualization
    rviz_config_file = os.path.join(
        get_package_share_directory("kuka_iontec_support"),
        "rviz",
        "dual_robot.rviz"
    )
    
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz'))
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
        DeclareLaunchArgument(
            'paused',
            default_value='true',
            description='Start Gazebo paused'
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start joint state publisher GUI with sliders'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Start RViz'
        ),
        gazebo,
        robot_state_publisher,
        spawn_dual_robot,
        controller_manager,
        spawn_left_joint_state_broadcaster,
        spawn_left_arm_controller,
        spawn_left_rail_controller,
        spawn_right_joint_state_broadcaster,
        spawn_right_arm_controller,
        spawn_right_rail_controller,
        # Individual DKP controllers (Option A - recommended for current URDF)
        spawn_dkp1_joint_state_broadcaster,
        spawn_dkp1_controller,
        spawn_dkp2_joint_state_broadcaster,
        spawn_dkp2_controller,
        joint_state_publisher_gui,
        bridge,
        rviz,
    ])
