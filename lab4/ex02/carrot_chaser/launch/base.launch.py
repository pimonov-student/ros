from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		DeclareLaunchArgument(
			'target_frame', default_value='carrot1',
			description='Target frame name'
		),
		
		Node(
			package='turtlesim',
			executable='turtlesim_node',
			name='sim'
		),
		Node(
			package='carrot_chaser',
			executable='broadcaster',
			name='broadcaster1',
			parameters=[
				{'turtlename': 'turtle1'}
			]
		),
		Node(
			package='carrot_chaser',
			executable='broadcaster',
			name='broadcaster2',
			parameters=[
				{'turtlename': 'turtle2'}
			]
		),
		Node(
			package='carrot_chaser',
			executable='listener',
			name='listener',
			parameters=[
				{'target_frame': LaunchConfiguration('target_frame')}
			]
		),
	])











































