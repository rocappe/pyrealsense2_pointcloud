import os
from pathlib import Path
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():
	pic4track_config = os.path.join(str(Path(__file__).parents[4]), 'src/pic4track_person', 'config')
	rviz_config_path = os.path.join(pic4track_config, 'cloud.rviz')
	tb3_path = os.path.join(get_package_share_directory("turtlebot3_bringup"), "launch")
	
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

	
	return LaunchDescription([
		camera_node,
		depth_robot_tf
		])

