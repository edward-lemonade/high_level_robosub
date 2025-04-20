from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction


def launch_setup(context, *args, **kwargs):
	robosub_arguments = ([
		"/keyboard/keypress@std_msgs/msg/Int32@gz.msgs.Int32"
	])
	robosub_bridge = Node(
		package="ros_gz_bridge",
		executable="parameter_bridge",
		arguments=robosub_arguments,
		output="screen",
	)

	rigid_controls = Node(
		package="high_level_robosub",
		executable="rigid_controls", # defined in setup.py
		output="screen",
		# parameters=[{'use_sim_time': True}],
	)

	#twist_to_pose = Node(
	#	package='high_level_robosub',
	#	executable='twist_to_pose',
	#	name='twist_to_pose',
	#	output='screen'
	#)

	return [robosub_bridge, rigid_controls]


def generate_launch_description():
	args = [
		DeclareLaunchArgument(
			"namespace",
			default_value="",
			description="Namespace",
		),
	]

	return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
