import os
from pathlib import Path
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():
	pointcloud_config = os.path.join(str(Path(__file__).parents[4]), 'src/pointcloud', 'config')
	rviz_config_path = os.path.join(pointcloud_config, 'cloud.rviz')
	tb3_path = os.path.join(get_package_share_directory("turtlebot3_bringup"), "launch")

	# Rviz
	rviz_node = Node(
		package='rviz2',
		executable='rviz2',
		name='rviz2',
		arguments=['-d', rviz_config_path],
		parameters=[{'use_sim_time': False}]
		)
	
	depth_robot_tf = Node(
		package='tf2_ros',
		executable = 'static_transform_publisher',
		arguments = ["0.03", "0", "1.175", "-1.5708", "0", "-1.5184", "base_footprint", "depth_frame"]
		)

	camera_node = Node(
		package='pointcloud', 
		executable='point_cloud_ros2',
		output='screen'
		)
	tb3_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource([tb3_path, '/robot.launch.py'])
		)
	
	return LaunchDescription([
		rviz_node,
		camera_node,
		depth_robot_tf,
		tb3_launch
		])

