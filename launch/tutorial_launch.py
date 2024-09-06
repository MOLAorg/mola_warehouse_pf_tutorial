import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Get the path to the mola_warehouse_pf_tutorial package
    mola_warehouse_pkg_share = get_package_share_directory(
        'mola_warehouse_pf_tutorial')

    # Define the path to the mm_file under /maps/mvsim-warehouse01.mm
    mm_file_path = os.path.join(
        mola_warehouse_pkg_share, 'maps', 'mvsim-warehouse01.mm')

    # Define the path to the custom rviz config file
    rviz_config_file = os.path.join(
        mola_warehouse_pkg_share, 'rviz', 'config.rviz')

    # Include the mrpt_map_server.launch.py from the mrpt_map_server package
    mrpt_map_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('mrpt_map_server'), '/launch/mrpt_map_server.launch.py']),
        launch_arguments={'mm_file': mm_file_path}.items()
    )

    # Launch for pf_localization:
    pf_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('mrpt_pf_localization'), 'launch',
            'localization.launch.py')]),
        launch_arguments={
            'log_level': 'INFO',
            'log_level_core': 'INFO',
            # 'topic_sensors_2d_scan': '/laser1',
            'topic_sensors_point_clouds': '/pf_input_points',

            # For robots with wheels odometry, use:     'base_link'-> 'odom'      -> 'map'
            # For systems without wheels odometry, use: 'base_link'-> 'base_link' -> 'map'
            'base_link_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'global_frame_id': 'map',
        }.items()
    )

    # Launch RViz2 with the custom configuration
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    # MVSim:
    mvsim_node = Node(
        package='mvsim',
        executable='mvsim_node',
        name='mvsim',
        output='screen',
        parameters=[
            os.path.join(mola_warehouse_pkg_share, 'params',
                         'mvsim_ros2_params.yaml'),
            {
                "world_file": os.path.join(get_package_share_directory('mvsim'), 'mvsim_tutorial', 'demo_warehouse.world.xml'),
            }]
    )

    # Launch for mrpt_pointcloud_pipeline:
    pointcloud_pipeline_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('mrpt_pointcloud_pipeline'), 'launch',
            'pointcloud_pipeline.launch.py')]),
        launch_arguments={
            'log_level': 'INFO',
            'pipeline_yaml_file': os.path.join(
                mola_warehouse_pkg_share, 'params', 'point-cloud-pipeline.yaml'),
            'points_topic_name': '/rslidar_points',
            'filter_output_layer_name': 'output_for_pf',
            'filter_output_topic_name': '/pf_input_points',
            'time_window': '0.20',
            'show_gui': 'False',
            'one_observation_per_topic': 'True',
            'frameid_robot': 'base_link',
            'frameid_reference': 'odom',
        }.items()
    )

    # Create and return the launch description
    return LaunchDescription([
        mrpt_map_server_launch,
        rviz_node,
        mvsim_node,
        pointcloud_pipeline_launch,
        pf_localization_launch,
    ])
