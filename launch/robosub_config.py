from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction


def launch_setup(context, *args, **kwargs):
	robosub_arguments = ([
		"/keyboard/keypress@std_msgs/msg/Int32@gz.msgs.Int32",
		"/gazebo/set_entity_pose@ros_gz_interfaces/srv/SetEntityPose",
	])
	robosub_bridge = Node(
		package="ros_gz_bridge",
		executable="parameter_bridge",
		arguments=robosub_arguments,
		output="screen",
	)

	controls = Node(
		package="high_level_robosub",
		executable="twist_controls", # defined in setup.py
		output="screen",
	)

	twist_to_pose = Node(
		package='high_level_robosub',
		executable='twist_to_pose',
		name='twist_to_pose',
		output='screen'
	)

	return [robosub_bridge, controls, twist_to_pose]


def generate_launch_description():
	args = [
		DeclareLaunchArgument(
			"namespace",
			default_value="",
			description="Namespace",
		),
	]

	return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
