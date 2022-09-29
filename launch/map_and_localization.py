from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_slam_toolbox_launch_description():


    slam_toolbox_node = Node(
        parameters=[
            # if true, processing new measurements is paused at startup
            # and must be started via service call. otherwise, start processing scans at startup.
            {'paused_new_measurements': True},
        ],
        package='slam_toolbox',
        executable='map_and_localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')



    return [slam_toolbox_node]


def generate_launch_description():
    return LaunchDescription(
        generate_slam_toolbox_launch_description()
    )
