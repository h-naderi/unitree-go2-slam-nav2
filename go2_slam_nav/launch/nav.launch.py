from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            name='use_rviz',
            default_value='true',
            choices=['true','false'],
            description='Open RVIZ for Go1 visualization'
        ),

        DeclareLaunchArgument(
            name='use_nav2_rviz',
            default_value='true',
            choices=['true','false'],
            description='Open RVIZ for Nav2 visualization'
        ),

        DeclareLaunchArgument(
            name='localize_only',
            default_value='true',
            choices=['true','false'],
            description='Localize only, do not change loaded map'
        ),

        DeclareLaunchArgument(
            name='restart_map',
            default_value='false',
            choices=['true','false'],
            description='Delete previous map and restart'
        ),
        
        DeclareLaunchArgument(
            name='log_level',
            default_value='warn',
            choices=['debug', 'info', 'warn', 'error', 'fatal'],
            description='Logging level for sport_ctrl node'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=[
                '-d',
                PathJoinSubstitution([
                    FindPackageShare('go2_slam_nav'),
                    'config',
                    'nav.rviz'
                ])
            ],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
        
        Node(
            package='go2_cmd_processor',
            executable='sport_ctrl',
            name='sport_ctrl',
            output='screen',
            parameters=[{
                'log_level': LaunchConfiguration('log_level')
            }]
        ),
        
        Node(
	    package='nav2_lifecycle_manager',
	    executable='lifecycle_manager',
	    name='lifecycle_manager_navigation',
	    output='screen',
	    parameters=[{
		'use_sim_time': True,
		'autostart': True,
		'node_names': [
		    'controller_server',
		    'planner_server',
		    'recoveries_server',
		    'bt_navigator',
		    'local_costmap',
		    'global_costmap',
		    'waypoint_follower'
		]
	    }]
	),

      #  IncludeLaunchDescription(
       #     PythonLaunchDescriptionSource(
        #        PathJoinSubstitution([
         #           FindPackageShare('rtabmap_launch_pkg'),
          #          'launch',
           #         'control.launch.py'
            #    ])
            #),
            #launch_arguments=[
            #    ('use_rviz', 'false'),
            #],
        #),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('go2_slam_nav'),
                    'launch',
                    'mapping.launch.py'
                ])
            ),
            launch_arguments=[
                ('use_rviz', 'false'),
                ('publish_static_tf', 'false'),
                ('localize_only', LaunchConfiguration('localize_only')),
                ('restart_map', LaunchConfiguration('restart_map')),
            ],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'navigation_launch.py'
                ])
            ),
            launch_arguments=[
                ('params_file',
                    PathJoinSubstitution([
                        FindPackageShare('go2_slam_nav'),
                        'config',
                        'nav2_params.yaml'
                    ])
                ),
            ],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'rviz_launch.py'
                ])
            ),
            condition=IfCondition(LaunchConfiguration('use_nav2_rviz')),
        ),
    ])

