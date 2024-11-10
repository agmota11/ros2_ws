import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtle_sim_node'),

        launch_ros.actions.Node(
            package='maadm_practica_nodes',
            executable='keyboard_listener',
            name='keyboard_listener',
            parameters=[{'freq': 10}, {'log_level': 10}]),

        launch_ros.actions.Node(
            package='maadm_practica_nodes',
            executable='turtle_position',
            name='turtle_position',
            parameters=[{'log_level': 10}]),

        launch_ros.actions.Node(
            package='maadm_practica_nodes',
            executable='trace_controller',
            name='trace_controller',
            parameters=[{'log_level': 10}]),
    ])