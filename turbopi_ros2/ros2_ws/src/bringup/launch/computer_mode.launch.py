import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # --- 1. GET PATHS ---
    # We locate the specific launch files you already have on the robot
    controller_pkg = get_package_share_directory('controller')
    peripherals_pkg = get_package_share_directory('peripherals')

    # --- 2. DEFINE NODES ---
    
    # A. THE MUSCLES (Motor Controller + Kinematics)
    # This turns on the chassis so it can move.
    base_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_pkg, 'launch', 'controller.launch.py')
        )
    )

    # B. THE EYES (USB Camera)
    # This publishes raw images to '/image_raw' for your computer to see.
    # We stripped out 'web_video_server' to save CPU.
    camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_pkg, 'launch', 'usb_cam.launch.py')
        )
    )

    # C. THE INPUT (Teleop Keyboard)
    # This opens a small "Popup Terminal" (xterm) so you can type WASD.
    # CRITICAL: We remap 'controller/cmd_vel' to '/cmd_vel' so it actually drives!
    teleop_node = Node(
        package='peripherals',
        executable='teleop_key_control',
        name='computer_teleop',
        output='screen',
        # 'xterm -e' forces this node to open in a new window so it can catch keystrokes
        prefix='xterm -e',
        remappings=[
            ('controller/cmd_vel', '/cmd_vel')
        ]
    )

    # --- 3. LAUNCH EVERYTHING ---
    return LaunchDescription([
        base_driver,
        camera_node,
        teleop_node
    ])
