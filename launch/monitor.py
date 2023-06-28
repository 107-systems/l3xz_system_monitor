import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='l3xz_system_monitor',
      namespace='l3xz',
      executable='l3xz_system_monitor_node',
      name='l3xz_system_monitor',
      output='screen',
      emulate_tty=True,
      parameters=[
        {'heartbeat_monitor_list' : ['l3xz_joy', 'l3xz_teleop', 'l3xz_gait_ctrl', 'l3xz_head_ctrl', 'ros2_cyphal_bridge', 'ros2_dynamixel_bridge', 'l3xz_pump_ctrl', 'l3xz_valve_ctrl']},
      ]
    )
  ])
