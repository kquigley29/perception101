import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    perception_meta_dir = get_package_share_directory('perception_meta')
    lidar101_share = get_package_share_directory('lidar101')

    bag = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource([perception_meta_dir + '/launch/fsuk18.launch.py']),
        launch_arguments={'robot_name': 'wheelchair'}.items()
    )

    lidar101_node = launch_ros.actions.Node(
        package='lidar101',
        executable='lidar101',
        name='lidar101',
        output='screen',
        parameters=[
            lidar101_share + '/config/params.yaml'
        ]
    )

    rviz2 = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d ' + get_package_share_directory('lidar_grid') + '/rviz/lidar_grid.rviz']
    )

    ld = launch.LaunchDescription([
        bag,
        lidar101_node,
        rviz2
    ])

    return ld


if __name__ == '__main__':
    generate_launch_description()




