import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  ld = LaunchDescription()

  config_file_path = os.path.join(
    get_package_share_directory('l3xz_watchdog'),
    'config',
    'watchdog-config.json'
  )

  return LaunchDescription([
    Node(
      package='l3xz_watchdog',
      namespace='l3xz',
      executable='l3xz_watchdog_node',
      name='l3xz_watchdog',
      output='screen',
      emulate_tty=True,
      parameters=[
        {'config_file' : config_file_path},
      ]
    )
  ])
