import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='moth_bridge',
            executable='limo_to_moth',
            name='limo_to_moth'),
        launch_ros.actions.Node(
            package='moth_bridge',
            executable='moth_to_limo',
            name='lmoth_to_limo'),
  ])