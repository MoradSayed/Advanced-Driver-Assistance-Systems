from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
import os
import pathlib

def generate_launch_description():
    webots_home = os.environ.get("WEBOTS_HOME")
    windows_ip = os.environ.get("WINDOWS_IP")
    webots_protocol = os.environ.get("WEBOTS_PROTOCOL")
    project_dir = os.environ.get("GP_WS_DIR")

    controller_dir = pathlib.Path(project_dir).resolve() / 'WBs/controllers/'
    ros2av_loc = str(controller_dir / 'ros2av/ros2av.py')
    drivecycle_loc = str(controller_dir / 'drive_cycle/drive_cycle.py')

    # Define the Webots controller processes
    ego_controller = ExecuteProcess(
        cmd=[
            f"{webots_home}/webots-controller",
            f"--protocol={webots_protocol}",
            *([f"--ip-address={windows_ip}"] if webots_protocol == "tcp" else []),
            "--robot-name=ego_vehicle",
            ros2av_loc
        ],
        name='ego_controller',
        output='screen',
        shell=False
    )

    lead_controller = ExecuteProcess(
        cmd=[
            f"{webots_home}/webots-controller",
            f"--protocol={webots_protocol}",
            *([f"--ip-address={windows_ip}"] if webots_protocol == "tcp" else []),
            "--robot-name=lead_vehicle",
            drivecycle_loc
        ],
        name='lead_controller',
        output='screen',
        shell=False
    )

    # Define the ROS 2 node
    ego_node = Node(
        package='vehicle_controller',
        executable='adas',
        name='Ego_vehicle',
        output='screen',
    )

    # Shared flags to track process exits
    process_flags = {'ego': False, 'lead': False}

    # Function to check both flags and trigger shutdown
    def check_and_shutdown(context):
        if process_flags['ego'] and process_flags['lead']:
            return [Shutdown(reason='Both Webots controllers finished')]
        return []

    # Function to create process exit handlers
    def track_exit(process_name):
        def handler(event, context):
            process_flags[process_name] = True
            return check_and_shutdown(context)
        return handler

    # Register event handlers for both controller exits
    on_ego_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=ego_controller,
            on_exit=track_exit('ego')
        )
    )

    on_lead_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=lead_controller,
            on_exit=track_exit('lead')
        )
    )

    # Return full launch description
    return LaunchDescription([
        ego_controller,
        lead_controller,
        ego_node,
        on_ego_exit,
        on_lead_exit
    ])
