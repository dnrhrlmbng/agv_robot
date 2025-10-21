import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_urdf_path = get_package_share_directory('agv_robot_description')
    pkg_gazebo_path = get_package_share_directory('agv_robot_gazebo')
    pkg_pointcloud_path = get_package_share_directory('pointcloud_concatenate')


    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz.'
    )

    world_arg = DeclareLaunchArgument(
        'world', default_value='maze.world',
        description='Name of the Gazebo world file to load'
    )

    model_arg = DeclareLaunchArgument(
        'model', default_value='robot_3d.urdf.xacro',
        description='Name of the URDF description to load'
    )

    pointcloud_concatenate_arg = DeclareLaunchArgument(
        'pointcloud_concatenate', default_value='true',
        description='Enable pointcloud concatenate from 4 cameras'
    )

    urdf_file_path = PathJoinSubstitution([
        pkg_urdf_path, 
        "urdf",
        "robots",
        LaunchConfiguration('model')
    ])

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_path, 'launch', 'world.launch.py'),
        ),
        launch_arguments={
        'world': LaunchConfiguration('world'),
        }.items()
    )

    pointcloud_concatenate_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_pointcloud_path,
                'launch',
                'pointcloud_concatenate.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('pointcloud_concatenate'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_urdf_path, 'rviz', 'rviz.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': True},
        ]
    )

    spawn_urdf_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "my_robot",
            "-topic", "robot_description",
            "-x", "-0.3", "-y", "-2.0", "-z", "0.5", "-Y", "1.59"
        ],
        output="screen",
        parameters=[
            {'use_sim_time': True},
        ]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': Command(['xacro', ' ', urdf_file_path]),
             'use_sim_time': True},
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    gz_bridge_node = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    arguments=[
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
        "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
        # "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        "/camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
        "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        "/cam_front/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        "/cam_back/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        "/cam_left/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        "/cam_right/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
    ],
    output="screen",
    parameters=[{'use_sim_time': True}]
)

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(rviz_launch_arg)
    launchDescriptionObject.add_action(world_arg)
    launchDescriptionObject.add_action(model_arg)
    launchDescriptionObject.add_action(pointcloud_concatenate_arg) 
    launchDescriptionObject.add_action(world_launch)
    launchDescriptionObject.add_action(pointcloud_concatenate_launch) 
    launchDescriptionObject.add_action(rviz_node)
    launchDescriptionObject.add_action(spawn_urdf_node)
    launchDescriptionObject.add_action(robot_state_publisher_node)
    launchDescriptionObject.add_action(gz_bridge_node)
    return launchDescriptionObject
