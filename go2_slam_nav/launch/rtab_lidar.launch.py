from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # ============ Launch Arguments ============
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            choices=['true','false'],
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            name='deskewing',
            default_value='false',
            choices=['true','false'],
            description='Enable lidar deskewing'
        ),
        DeclareLaunchArgument(
            name='localize_only',
            default_value='true',
            choices=['true','false'],
            description='Localize only, do not add new places to the map'
        ),
        DeclareLaunchArgument(
            name='restart_map',
            default_value='false',
            choices=['true','false'],
            description='Delete previous map/database and restart'
        ),
        DeclareLaunchArgument(
            name='use_rtabmapviz',
            default_value='true',
            choices=['true','false'],
            description='Start rtabmapviz node'
        ),
        DeclareLaunchArgument(
            name='rtabmap_log_level',
            default_value='WARN',
            choices=['ERROR', 'WARN', 'INFO', 'DEBUG'],
            description='Set logger level for rtabmap.'
        ),
        DeclareLaunchArgument(
            name='icp_odometry_log_level',
            default_value='WARN',
            choices=['ERROR', 'WARN', 'INFO', 'DEBUG'],
            description='Set logger level for icp_odometry.'
        ),

        # ============ ICP Odometry Node ============
        Node(
            package='rtabmap_odom',
            executable='icp_odometry',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'wait_for_transform': 0.3,
                'expected_update_rate': 15.0,
                'subscribe_odom_info': False,
                'deskewing': LaunchConfiguration('deskewing'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            remappings=[
                ('scan_cloud', '/rslidar_points')
            ],
            arguments=[
                'Icp/PointToPlane', 'true',
                'Icp/Iterations', '10',
                'Icp/VoxelSize', '0.1',
                'Icp/Epsilon', '0.001',
                'Icp/PointToPlaneK', '20',
                'Icp/PointToPlaneRadius', '0',
                'Icp/MaxTranslation', '2',
                'Icp/MaxCorrespondenceDistance', '1',
                'Icp/Strategy', '1',
                'Icp/OutlierRatio', '0.7',
                'Icp/CorrespondenceRatio', '0.01',
                'Odom/ScanKeyFrameThr', '0.6',
                'OdomF2M/ScanSubtractRadius', '0.1',
                'OdomF2M/ScanMaxSize', '15000',
                'OdomF2M/BundleAdjustment', 'false',
                '--ros-args',
                '--log-level',
                LaunchConfiguration('icp_odometry_log_level'),
            ]
        ),

        # ============ point_cloud_assembler Node ============
        Node(
            package='rtabmap_util',
            executable='point_cloud_assembler',
            output='screen',
            parameters=[{
                'max_clouds': 10,
                'fixed_frame_id': '',
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            remappings=[
                ('cloud', 'odom_filtered_input_scan')
            ]
        ),

        # ============ RTAB-Map Node (Reusing DB) ============
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'frame_id': 'base_laser',
                'subscribe_depth': False,
                'subscribe_rgb': False,
                'subscribe_scan_cloud': True,
                'approx_sync': True,
                'wait_for_transform': 0.3,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            remappings=[
                ('scan_cloud', 'assembled_cloud')
            ],
            # Only launch if restart_map == "false"
            condition=IfCondition(
                PythonExpression([
                    '"', LaunchConfiguration('restart_map'), '" == "false"'
                ])
            ),
            arguments=[
                # Decide if we do mapping (IncrementalMemory=true) or localization (false)
                'Mem/IncrementalMemory',
                PythonExpression([
                    '"false" if "', LaunchConfiguration('localize_only'), '" == "true" else "true"'
                ]),

                # If localizing only, we often want to load all nodes in WM:
                'Mem/InitWMWithAllNodes',
                PythonExpression([
                    '"true" if "', LaunchConfiguration('localize_only'), '" == "true" else "false"'
                ]),

                # Optionally do not grow DB in localization mode:
                'Mem/LocalizationDataSaved',
                PythonExpression([
                    '"false" if "', LaunchConfiguration('localize_only'), '" == "true" else "true"'
                ]),

                # Other parameters unchanged...
                'RGBD/ProximityMaxGraphDepth', '0',
                'RGBD/ProximityPathMaxNeighbors', '1',
                'RGBD/AngularUpdate', '0.05',
                'RGBD/LinearUpdate', '0.05',
                'RGBD/CreateOccupancyGrid', 'false',
                'Mem/NotLinkedNodesKept', 'false',
                'Mem/STMSize', '30',
                'Mem/LaserScanNormalK', '20',
                'Reg/Strategy', '1',
                'Icp/VoxelSize', '0.1',
                'Icp/PointToPlaneK', '20',
                'Icp/PointToPlaneRadius', '0',
                'Icp/PointToPlane', 'true',
                'Icp/Iterations', '10',
                'Icp/Epsilon', '0.001',
                'Icp/MaxTranslation', '3',
                'Icp/MaxCorrespondenceDistance', '1',
                'Icp/Strategy', '1',
                'Icp/OutlierRatio', '0.7',
                'Icp/CorrespondenceRatio', '0.2',

                '--ros-args',
                '--log-level',
                LaunchConfiguration('rtabmap_log_level'),
            ]
        ),

        # ============ RTAB-Map Node (Restarting DB) ============
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap_reset',
            output='screen',
            parameters=[{
                'frame_id': 'base_laser',
                'subscribe_depth': False,
                'subscribe_rgb': False,
                'subscribe_scan_cloud': True,
                'approx_sync': True,
                'wait_for_transform': 0.3,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            remappings=[
                ('scan_cloud', 'assembled_cloud')
            ],
            # Only launch if restart_map == "true"
            condition=IfCondition(
                PythonExpression([
                    '"', LaunchConfiguration('restart_map'), '" == "true"'
                ])
            ),
            arguments=[
                # Same logic for mapping vs. localization:
                'Mem/IncrementalMemory',
                PythonExpression([
                    '"false" if "', LaunchConfiguration('localize_only'), '" == "true" else "true"'
                ]),
                'Mem/InitWMWithAllNodes',
                PythonExpression([
                    '"true" if "', LaunchConfiguration('localize_only'), '" == "true" else "false"'
                ]),
                'Mem/LocalizationDataSaved',
                PythonExpression([
                    '"false" if "', LaunchConfiguration('localize_only'), '" == "true" else "true"'
                ]),

                # Wipe old DB:
                '--delete_db_on_start',

                # Other parameters unchanged...
                'RGBD/ProximityMaxGraphDepth', '0',
                'RGBD/ProximityPathMaxNeighbors', '1',
                'RGBD/AngularUpdate', '0.05',
                'RGBD/LinearUpdate', '0.05',
                'RGBD/CreateOccupancyGrid', 'false',
                'Mem/NotLinkedNodesKept', 'false',
                'Mem/STMSize', '30',
                'Mem/LaserScanNormalK', '20',
                'Reg/Strategy', '1',
                'Icp/VoxelSize', '0.1',
                'Icp/PointToPlaneK', '20',
                'Icp/PointToPlaneRadius', '0',
                'Icp/PointToPlane', 'true',
                'Icp/Iterations', '10',
                'Icp/Epsilon', '0.001',
                'Icp/MaxTranslation', '3',
                'Icp/MaxCorrespondenceDistance', '1',
                'Icp/Strategy', '1',
                'Icp/OutlierRatio', '0.7',
                'Icp/CorrespondenceRatio', '0.2',

                '--ros-args',
                '--log-level',
                LaunchConfiguration('rtabmap_log_level'),
            ]
        ),

        # ============ (Optional) RTAB-Map Viz Node ============
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            output='screen',
            parameters=[{
                'frame_id': 'base_laser',
                'odom_frame_id': 'odom',
                'subscribe_odom_info': True,
                'subscribe_scan_cloud': True,
                'approx_sync': True,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            remappings=[
                ('scan_cloud', 'odom_filtered_input_scan')
            ],
            condition=IfCondition(LaunchConfiguration('use_rtabmapviz'))
        ),

        # ============ Static TF between base_link and base_laser ============
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'base_laser']
        ),
    ])


