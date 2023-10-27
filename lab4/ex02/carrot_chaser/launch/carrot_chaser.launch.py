import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
	base_nodes = IncludeLaunchDescription(
		PythonLaunchDescriptionSource([os.path.join(
			get_package_share_directory('carrot_chaser'), 'launch'),
			'/base.launch.py']),
		launch_arguments={'target_frame': 'carrot1'}.items(),
	)	
	
	return LaunchDescription([
		base_nodes,
	])





















