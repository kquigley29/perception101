import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import OpaqueFunction


def launch_setup(context, *args, **kwargs):
    velodyne_driver_share = get_package_share_directory('velodyne_driver')
    velodyne_pc_share = get_package_share_directory(("velodyne_pointcloud"))
    perception_meta_share = get_package_share_directory('perception_meta')
    lidar101_share = get_package_share_directory('lidar101')

    use_driver_str = launch.substitutions.LaunchConfiguration("use_driver").perform(context)
    use_driver = use_driver_str == "true"

    ld_components = []

    if use_driver:
        velodyne_driver = launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource([velodyne_driver_share + '/launch/velodyne_driver_node-VLP16-launch.py']))

        velodyne_pc = launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource([velodyne_pc_share + '/launch/velodyne_convert_node-VLP16-launch.py']))

        ld_components.append(velodyne_driver)
        ld_components.append(velodyne_pc)

    tf = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource([perception_meta_share + '/misc_files/publish_transforms.launch.py']),
        launch_arguments={'robot_name': 'wheelchair'}.items()
    )
    ld_components.append(tf)

    lidar101_node = launch_ros.actions.Node(
        package='lidar101',
        executable='lidar101',
        name='lidar101',
        output='screen',
        parameters=[
            lidar101_share + '/config/params.yaml'
        ])
    ld_components.append(lidar101_node)

    rviz2 = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d ' + get_package_share_directory('lidar_grid') + '/rviz/lidar_grid.rviz'])
    ld_components.append(rviz2)

    return ld_components


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='use_driver',
            default_value='true'
        ),
        OpaqueFunction(function=launch_setup)
    ])

    return ld


if __name__ == '__main__':
    generate_launch_description()




