# Example:
#   $ ros2 launch rslidar_sdk start.py
#
#   SLAM:
#   $ ros2 launch rtabmap_ros rslidar_robosense.launch.py


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # Launch arguments
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
            name='use_rtabmapviz',
            default_value='true',
            choices=['true','false'],
            description='Start rtabmapviz node'
        ),

        DeclareLaunchArgument(
            name='rtabmap_log_level',
            default_value='INFO',
            choices=['ERROR', 'WARN', 'INFO', 'DEBUG'],
            description='Set logger level for rtabmap.'
        ),

        DeclareLaunchArgument(
            name='icp_odometry_log_level',
            default_value='INFO',
            choices=['ERROR', 'WARN', 'INFO', 'DEBUG'],
            description='Set logger level for icp_odometry. Can set to WARN to reduce message output from this node.'
        ),

        # Nodes to launch
        Node(
            package='rtabmap_odom', executable='icp_odometry', output='screen',
            parameters=[{
                'frame_id':'base_link',
                'odom_frame_id':'odom',
                'wait_for_transform':0.3, # 0.2
                'expected_update_rate':15.0,
                'deskewing':LaunchConfiguration('deskewing'),
                'use_sim_time':LaunchConfiguration('use_sim_time'),
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
                [
                    TextSubstitution(text='icp_odometry:='),
                    LaunchConfiguration('icp_odometry_log_level'),
                ],
            ]
            ),
            
        Node(
            package='rtabmap_util', executable='point_cloud_assembler', output='screen',
            parameters=[{
                'max_clouds':10,
                'fixed_frame_id':'',
                'use_sim_time':LaunchConfiguration('use_sim_time'),
            }],
            remappings=[
                ('cloud', 'odom_filtered_input_scan')
            ]),
            
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[{
                'frame_id':'base_laser',
                'queue_size': 15,
                'subscribe_depth':True, #True
                'subscribe_rgb':True,   #True
                'subscribe_scan_cloud':True,
                'approx_sync':True, # False
                'wait_for_transform': 0.3, #0.2,
                'use_sim_time':LaunchConfiguration('use_sim_time'),
            }],
            remappings=[
                ('scan_cloud', 'assembled_cloud'),
                ('imu', '/imu/data'),
                ('rgb/image', '/camera/infra1/image_rect_raw'),
                ('rgb/camera_info', '/camera/infra1/camera_info'),
                ('depth/image', '/camera/depth/image_rect_raw') ],
            arguments=[
                '-d', # This will delete the previous database (~/.ros/rtabmap.db)
                'RGBD/ProximityMaxGraphDepth', '0',
                'Grid/Sensor', '1',
                'Grid/RayTracing', 'true',
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
                [
                    TextSubstitution(text='rtabmap:='),
                    LaunchConfiguration('rtabmap_log_level'),
                ],
            ]), 
        Node(
            package='rtabmap_ros', executable='rtabmapviz', output='screen',
            parameters=[{
                'frame_id':'base_laser',
                'odom_frame_id':'odom',
                'subscribe_odom_info':True,
                'subscribe_scan_cloud':True,
                'approx_sync':True, # False
                'use_sim_time':LaunchConfiguration('use_sim_time'),
            }],
            remappings=[
                ('scan_cloud', 'odom_filtered_input_scan')
            ],
            condition=IfCondition(LaunchConfiguration('use_rtabmapviz'))
        ),
    ])
