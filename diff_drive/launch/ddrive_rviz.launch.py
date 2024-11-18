"""Launch rviz."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (Command,
                                  EqualsSubstitution,
                                  LaunchConfiguration,
                                  PathJoinSubstitution,
                                  TextSubstitution)

from launch_ros.actions import Node
from launch_ros.substitutions import ExecutableInPackage, FindPackageShare


def generate_launch_description():
    """Return Launch description for desired nodes."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'view_only', default_value=TextSubstitution(text='true')
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {'robot_description':
                 Command([ExecutableInPackage('xacro', 'xacro'), ' ',
                          PathJoinSubstitution(
                              [FindPackageShare('diff_drive'),
                               'ddrive.urdf.xacro']
                          )])
                 }
            ]
        ),
        Node(
            condition=IfCondition(EqualsSubstitution(
                LaunchConfiguration('view_only'), 'true')),
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),
        Node(
            condition=IfCondition(EqualsSubstitution(
                LaunchConfiguration('view_only'), 'true')),
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution([
                    FindPackageShare('diff_drive'), 'urdf_preview.rviz'
                ])]
        ),
        Node(
            condition=IfCondition(EqualsSubstitution(
                LaunchConfiguration('view_only'), 'false')),
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution([
                    FindPackageShare('diff_drive'), 'odom_urdf_preview.rviz'
                ])]
        ),


    ])
