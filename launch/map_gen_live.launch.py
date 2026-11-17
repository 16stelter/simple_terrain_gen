from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
import os, yaml
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    use_rviz = LaunchConfiguration('use_rviz').perform(context).lower()
    package_share = get_package_share_directory('map_gen')
    rtabmap_params = os.path.join(package_share, 'config', 'rtabmap_params.yaml')
    rviz_config = os.path.join(package_share, 'config', 'map_gen_live.rviz')

    remappings=[
        ('rgb/image',       '/camera/color/image_raw'),
        ('depth/image',     '/camera/aligned_depth_to_color/image_raw'),
        ('rgb/camera_info', '/camera/color/camera_info'),
        ('odom',            '/ov_msckf/odomimu')]

    camera_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robot_bringup'), 'launch', 'bringup_camera.launch.py')),
                launch_arguments={
                    'namespace': namespace,
                    'use_rviz': 'false'
            }.items(),
    )

    rtabmap_sync_cmd = Node(
        package='rtabmap_sync', executable='rgbd_sync', output='screen',
        parameters=[rtabmap_params],
        remappings=remappings
    )

    rtabmap_slam_cmd = Node(
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=[rtabmap_params],
        remappings=remappings,
        arguments=['-d']
    )

    rviz_cmd = Node(
        package='rviz2', executable='rviz2', name="rviz2", output='screen',
        condition=IfCondition(use_rviz),
        arguments=[["-d"], rviz_config]
    )

    return [camera_cmd, rtabmap_sync_cmd, rtabmap_slam_cmd, rviz_cmd]

def generate_launch_description():
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='true', description='Whether to launch RViz'
    )

    return LaunchDescription([
        declare_namespace_cmd,
        declare_rviz_cmd,
        OpaqueFunction(function=launch_setup)
    ])