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
		DeclareLaunchArgument(
			'radius', default_value='5',
			description='Distance between turtle1 and carrot'
		),
		DeclareLaunchArgument(
			'direction_of_rotation', default_value='1',
			description='In which direction the carrot will rotate, 1 - clockwise, -1 - counterclockwise'
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
		Node(
			package='carrot_chaser',
			executable='dynamic_frame_broadcaster',
			name='dynamic_broadcaster',
			parameters=[
				{'radius': '2'},
				{'direction_of_rotation': '-1'}
			]
		),
	])











































