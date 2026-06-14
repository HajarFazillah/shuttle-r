import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('shuttler_sim')
    pkg_tb4_bringup = get_package_share_directory('turtlebot4_ignition_bringup')
    pkg_tb4_description = get_package_share_directory('turtlebot4_description')
    pkg_tb4_gui_plugins = get_package_share_directory('turtlebot4_ignition_gui_plugins')
    pkg_create_description = get_package_share_directory('irobot_create_description')
    pkg_create_ign_bringup = get_package_share_directory('irobot_create_ignition_bringup')
    pkg_create_ign_plugins = get_package_share_directory('irobot_create_ignition_plugins')
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')

    world_path = os.path.join(pkg_share, 'worlds', 'empty_court.sdf')

    # Let Ignition find TurtleBot4 and Create3 model assets
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.path.join(pkg_tb4_bringup, 'worlds'), ':' +
            os.path.join(pkg_create_ign_bringup, 'worlds'), ':' +
            str(Path(pkg_tb4_description).parent.resolve()), ':' +
            str(Path(pkg_create_description).parent.resolve())
        ]
    )

    ign_gui_plugin_path = SetEnvironmentVariable(
        name='IGN_GUI_PLUGIN_PATH',
        value=[
            os.path.join(pkg_tb4_gui_plugins, 'lib'), ':' +
            os.path.join(pkg_create_ign_plugins, 'lib')
        ]
    )

    # Launch Gazebo Fortress with the badminton court world
    ignition_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')
        ),
        launch_arguments={
            'ign_args': world_path + ' -r -s'
        }.items()
    )

    # Clock bridge (sim time -> ROS time)
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock']
    )

    # Spawn TurtleBot4 Standard in one half of the court
    turtlebot4_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb4_bringup, 'launch', 'turtlebot4_spawn.launch.py')
        ),
        launch_arguments={
            'model': 'standard',
            'x': '1.0',
            'y': '0.0',
            'z': '0.05',
            'yaw': '0.0',
        }.items()
    )

    # Bridge the TurtleBot4's simulated RGBD camera to ROS2
    # The actual Ignition sensor is named 'rgbd_camera', not 'oakd_rgb_camera'
    camera_image_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_image_bridge',
        output='screen',
        arguments=[
            '/world/empty_court/model/turtlebot4/link/oakd_rgb_camera_frame'
            '/sensor/rgbd_camera/image'
            '@sensor_msgs/msg/Image[ignition.msgs.Image'
        ],
        remappings=[(
            '/world/empty_court/model/turtlebot4/link/oakd_rgb_camera_frame'
            '/sensor/rgbd_camera/image',
            '/camera/image_raw'
        )]
    )

    # Bridge the RGBD camera's depth image (32FC1, meters)
    camera_depth_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_depth_bridge',
        output='screen',
        arguments=[
            '/world/empty_court/model/turtlebot4/link/oakd_rgb_camera_frame'
            '/sensor/rgbd_camera/depth_image'
            '@sensor_msgs/msg/Image[ignition.msgs.Image'
        ],
        remappings=[(
            '/world/empty_court/model/turtlebot4/link/oakd_rgb_camera_frame'
            '/sensor/rgbd_camera/depth_image',
            '/camera/depth/image_raw'
        )]
    )

    # Bridge the RGBD camera's intrinsics
    camera_info_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_info_bridge',
        output='screen',
        arguments=[
            '/world/empty_court/model/turtlebot4/link/oakd_rgb_camera_frame'
            '/sensor/rgbd_camera/camera_info'
            '@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo'
        ],
        remappings=[(
            '/world/empty_court/model/turtlebot4/link/oakd_rgb_camera_frame'
            '/sensor/rgbd_camera/camera_info',
            '/camera/camera_info'
        )]
    )

    # Auto-undock after simulation has had time to fully start
    auto_undock = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'action', 'send_goal', '/undock',
                     'irobot_create_msgs/action/Undock', '{}'],
                output='screen'
            )
        ]
    )

    # Bridge TurtleBot4 RPLidar → /scan for SLAM
    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge',
        output='screen',
        arguments=[
            '/world/empty_court/model/turtlebot4/link/rplidar_link/sensor/rplidar/scan'
            '@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'
        ],
        remappings=[(
            '/world/empty_court/model/turtlebot4/link/rplidar_link/sensor/rplidar/scan',
            '/scan'
        )]
    )

    shuttlecock_detector = Node(
        package='shuttler_sim',
        executable='shuttlecock_detector',
        name='shuttlecock_detector',
        output='screen',
    )

    # Keeps the passive scoop + hopper bin rigidly attached to the robot
    scoop_follower = Node(
        package='shuttler_sim',
        executable='scoop_follower',
        name='scoop_follower',
        output='screen',
    )

    return LaunchDescription([
        ign_resource_path,
        ign_gui_plugin_path,
        ignition_gazebo,
        clock_bridge,
        camera_image_bridge,
        camera_depth_bridge,
        camera_info_bridge,
        turtlebot4_spawn,
        auto_undock,
        lidar_bridge,
        shuttlecock_detector,
        scoop_follower,
    ])
