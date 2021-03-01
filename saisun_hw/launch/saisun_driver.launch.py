import os
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    para_dir = os.path.join(get_package_share_directory('saisun_hw'), 'config', 'config.yaml')
    print(para_dir)
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            '',
            default_value=[launch.substitutions.EnvironmentVariable('USER'),'_'],
            description='Interface for Saisun Robot and CUHK vision algorithm'),
        launch_ros.actions.Node(
            package='saisun_hw',
            executable='saisun_hw',
            output='screen',
            name=['saisun_driver'],
            parameters=[para_dir]
        )
    ])