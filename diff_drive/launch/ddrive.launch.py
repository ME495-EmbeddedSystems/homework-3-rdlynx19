from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Return Launch description for desired nodes."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'world_file', default_value=['-r',PathJoinSubstitution([
                FindPackageShare('diff_drive'), 'worlds', 'asphalt_world.sdf'
            ])]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare
                                      ('diff_drive'),
                                      'ddrive_rviz.launch.py'])
            ]),
            launch_arguments={
                'view_only': 'false'
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare
                                      ('ros_gz_sim'),
                                      'launch','gz_sim.launch.py'])
            ]),
            launch_arguments={
                'gz_args': LaunchConfiguration('world_file')
            }.items()
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[
                'model/diff_drive/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry\
                /world/asphalt_world/model/buttercup/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model\
                /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'
            ],
            remappings=[
                ('/model/diff_drive/odometry', '/odom'),
                ('/world/asphalt_world/model/buttercup/joint_state', 
                 '/joint_states')
            ]
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic /robot_description -x -4.0 -z 0.5'
            ]
        ),
        Node(
            package='diff_drive',
            executable='flip'
        )
    ])