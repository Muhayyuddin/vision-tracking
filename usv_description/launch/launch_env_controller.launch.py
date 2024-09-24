import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    mbzirc_ros_share = get_package_share_directory('mbzirc_ros')
    mbzirc_ign_share = get_package_share_directory('mbzirc_ign')
    usv_description_share = get_package_share_directory('usv_description')

    # Launch files and arguments
    competition_local_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mbzirc_ros_share, 'launch', 'competition_local.launch.py')),
        launch_arguments={'ign_args': '-v 4 -r coast.sdf'}.items()
    )

    spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mbzirc_ign_share, 'launch', 'spawn.launch.py')),
        launch_arguments={
            'name': 'usv',
            'world': 'coast',
            'model': 'usv',
            'x': '-1380',
            'y': '50',
            'z': '0.3',
            'R': '0',
            'P': '0',
            'Y': '-0.5',
            'slot0': 'mbzirc_rgbd_camera',
            'slot0_rpy': '0 -14 0',
            'slot4': 'mbzirc_planar_lidar'
        }.items()
    )

    usv_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(usv_description_share, 'launch', 'usv.launch.py'))
    )

    # Node for usv_control twist publisher
    controller_node = Node(
        package='usv_control',
        executable='lqr_controller',
        output='screen'
    )

    # Node for custom_tf_broadcaster publish_odometry_and_tf
    tf_broadcaster_node = Node(
        package='custom_tf_broadcaster',
        executable='publish_odometry_and_tf',
        output='screen'
    )

    # Define LaunchDescription
    return LaunchDescription([
        # Launch all the necessary files and nodes
        competition_local_launch,
        spawn_launch,
        controller_node,  # This runs the twist_publisher node
        tf_broadcaster_node,  # This runs the custom TF broadcaster node
        usv_description_launch  # Launch the usv_description twice as per your request
    ])
