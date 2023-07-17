from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def find_diffbot_launch():
    install_folder = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../..'))
    for root, dirs, files in os.walk(install_folder):
        if 'diffbot.launch.py' in files:
            return os.path.join(root, 'diffbot.launch.py')
    return None

def generate_launch_description():
    diffbot_launch_path = find_diffbot_launch()
    if diffbot_launch_path is None:
        raise RuntimeError('Could not find diffbot.launch.py in install folder')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(diffbot_launch_path)
        ),
        Node(
            package='challenge_handin',
            executable='velocity_converter.py',
        ),
    ])