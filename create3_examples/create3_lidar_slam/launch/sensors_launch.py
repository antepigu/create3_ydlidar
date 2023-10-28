#!/usr/bin/env python3
# Copyright 2022 iRobot Corporation. All Rights Reserved.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Evaluate at launch the value of the launch configuration 'namespace' 
    namespace = LaunchConfiguration('namespace')

    # Declares an action to allow users to pass the robot namespace from the 
    # CLI into the launch description as an argument.
    namespace_argument = DeclareLaunchArgument(
        'namespace', 
        default_value='',
        description='Robot namespace')

    #Declare directory of the driver
    share_dir = get_package_share_directory('ydlidar_ros2_driver')
    #Evaluate at launch the file of the parameters for the lidar
    parameter_file = LaunchConfiguration('params_file')

    # Declares an action to allow users to pass the name of the file params
    # with a default value for ydlidar tmini.
    params_declare = DeclareLaunchArgument('params_file',
                                           default_value=os.path.join(
                                               share_dir, 'params', 'TminiPro.yaml'),
                                           description='Path to the ROS2 parameters file to use.')
    
    # Declares an action that will launch a node when executed by the launch description.
    # This node is responsible for providing a static transform from the robot's base_footprint
    # frame to a new laser_frame, which will be the coordinate frame for the lidar. 
    static_transform_node = Node(
        package='tf2_ros', 
        executable='static_transform_publisher',
        arguments=['-0.001', '0', '0.144', '0', '0', '0', 'base_link', 'laser_frame'],
        
        # Remaps topics used by the 'tf2_ros' package from absolute (with slash) to relative (no slash).
        # This is necessary to use namespaces with 'tf2_ros'.
        remappings=[
            ('/tf_static', 'tf_static'),
            ('/tf', 'tf')],
        namespace=namespace
    )
    
    # Declares an action that will launch a node when executed by the launch description.
    # This node is responsible for configuring the RPLidar sensor.    
    lidar_node = Node(
        name='ydlidar_composition',
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        namespace=namespace
    )

    lidar_node_tmini = ExecuteProcess(
    	cmd=['ros2', 'launch', 'ydlidar_ros2_driver', 'ydlidar_launch.py'],
    	output='screen',
    )
    
    # Launches all named actions
    return LaunchDescription([
        params_declare,
        namespace_argument,
        static_transform_node,
        TimerAction(
            period=2.0,
            actions=[lidar_node]
        )
    ])
