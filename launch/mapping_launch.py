from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory  
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg = 'VacServer'
    ekf_params = os.path.join(get_package_share_directory(pkg), 'config', 'ekf_params.yaml')

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_file = os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')

    nav2_params_file = os.path.join(get_package_share_directory(pkg), 'config', 'nav2_params.yaml')

    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        parameters=[{
                    'laser_scan_topic' : '/scan',
                    'odom_topic' : '/rf2o',
                    'publish_tf' : False,
                    'base_frame_id' : 'laser_link',
                    'odom_frame_id' : 'odom',
                    'init_pose_from_topic' : '',
                    'freq' : 40.0
                    }],
        output='screen',
    )

    RobotClientNode = Node(
        package=pkg,
        executable='RobotClientNode',
        output='screen',
        parameters=[{'ip': '192.168.1.247', 
                     "min_lidar_range_m": 0.1,
                     "max_lidar_range_m": 3.0
                    }],
    )

    ekf_node = Node(  
        package='robot_localization',  
        executable='ekf_node',  
        name='ekf_filter_node',  
        output='screen',  
        parameters=[ekf_params],  
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')],
    )

    explore_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        output='screen',
        parameters=[{
            'robot_base_frame': 'base_footprint',
            'return_to_init': False,
            'costmap_topic': 'global_costmap/costmap',
            'costmap_updates_topic': 'global_costmap/costmap_updates',
            'visualize': True,
            'planner_frequency': 20.0,
            'progress_timeout': 10.0,
            'potential_scale': 3.0,
            'orientation_scale': 0.0,
            'gain_scale': 1.0,
            'transform_tolerance': 0.3,
            'min_frontier_size': 0.2,
        }],
    )

    return LaunchDescription([
        RobotClientNode,
        rf2o_node,
        ekf_node,
        # explore_node,
        rviz_node,
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['-0.15', '0', '0', '0', '0', '0', 'base_link', 'laser_link'],
            name='static_tf_lidar'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['-0.06', '-0.085', '0', '0', '0', '0', 'base_link', 'imu_link'],
            name='static_tf_imu'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
            name='static_tf_footprint'
        ),
        # Node(
        #     package='slam_toolbox',
        #     executable='async_slam_toolbox_node',  # или 'sync_slam_toolbox_node'
        #     name='slam_toolbox',
        #     output='screen',
        #     parameters=[{
        #         'use_sim_time': False,           # True, если используете симуляцию
        #         'odom_frame': 'odom',            # Ваш фрейм одометрии
        #         'map_frame': 'map',              # Фрейм для публикации карты
        #         'base_frame': 'base_footprint',       # Фрейм вашего робота
        #         'scan_topic': '/scan',           # Топик со сканами лидара
        #         'mode': 'mapping',               # 'mapping' или 'localization'
        #         'publish_tf' : True,
        #         'map_update_interval': 0.5,          # обновлять карту каждую секунду
        #         'minimum_travel_distance': 0.1,      # обновлять при движении более 10 см
        #         'minimum_travel_heading': 0.2,       # обновлсять при повороте более 0.1 рад
        #         'minimum_time_interval': 0.2,        # обновлять не чаще чем раз в 0.5 сек
        #         'transform_publish_period': 0.1,
        #         'odom_alpha': 0.01,
        #         'do_loop_closing': False,

        #         # 'do_loop_closing': True,
        #         # 'loop_match_minimum_chain_size': 10,
        #         # 'loop_match_maximum_variance_coarse': 3.0,  
        #         # 'loop_match_minimum_response_coarse': 0.35,  
        #         # 'loop_match_minimum_response_fine': 0.45,
        #         # 'loop_search_maximum_distance': 3.0,
        #         # 'loop_search_maximum_distance_to_match': 10.0,
        #         # 'loop_search_minimum_distance_to_match': 2.0,
        #         # 'correction_type': 'Lidar',
        #     }]
        # ),
        Node(  
            package='slam_toolbox',  
            executable='sync_slam_toolbox_node',  
            name='slam_toolbox',  
            output='screen',  
            parameters=[{  
                'use_sim_time': False,  
                'odom_frame': 'odom',  
                'map_frame': 'map',  
                'base_frame': 'base_link',  
                'scan_topic': '/scan',  
                'mode': 'mapping',  
                'publish_tf': False,  
                'map_update_interval': 0.5,  
                'minimum_travel_distance': 0.1,  
                'minimum_travel_heading': 0.3,  
                'minimum_time_interval': 0.2,  
                'transform_publish_period': 3.0,  
                'transform_timeout': 3.0,  
                'use_scan_matching': False,
            }]  
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file),
            launch_arguments={
                'use_sim_time': 'False',
                'params_file': nav2_params_file,
            }.items()
        ),
    ])