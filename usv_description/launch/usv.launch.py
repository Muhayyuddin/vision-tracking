import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='usv_description').find('usv_description')
    default_model_path = os.path.join(pkg_share, 'urdf/usv.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': Command(['xacro ', default_model_path])}],
        arguments=[default_model_path])
        
    
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    robot_localization_node = launch_ros.actions.Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )   
    static_transformation2_node = launch_ros.actions.Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen" ,
        arguments=["-1350", "60.0", "0.3", "0", "0", "0", "odom", "usv"]
        #arguments=["-1450", "-16.5", "0.3", "0", "0", "0", "odom", "usv"]

    )
    static_transformation1_node = launch_ros.actions.Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen" ,
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]
    )
   


    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        #joint_state_publisher_node,
        #robot_state_publisher_node,
        robot_localization_node,
        static_transformation1_node,
        static_transformation2_node,
        #static_transformation_gripper,
        #static_transformation_left_finger,
        #static_transformation_right_finger,
        #rviz_node


    ])
