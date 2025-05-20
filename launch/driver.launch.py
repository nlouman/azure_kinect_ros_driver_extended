# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

# Added support for color control parameters and YAML file reading
# For more information, contact: https://github.com/nlouman/

import os
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, conditions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command

import launch.actions
import launch_ros.actions
from launch_ros.actions import PushRosNamespace


def to_urdf(xacro_path, urdf_path=None):
    """Convert the given xacro file to URDF file.
    * xacro_path -- the path to the xacro file
    * urdf_path -- the path to the urdf file
    """
    if urdf_path is None:
        urdf_path = "/tmp/azure_kinect.urdf"
        # urdf_path = os.path.join(get_package_share_directory("azure_kinect_ros_driver"), "urdf", "azure_kinect.urdf")
    doc = xacro.process_file(xacro_path)
    out = xacro.open_output(urdf_path)
    out.write(doc.toprettyxml(indent="  "))
    return urdf_path


def generate_launch_description():
    # Convert the azure_kinect.urdf.xacro into a URDF file
    xacro_file = os.path.join(get_package_share_directory("azure_kinect_ros_driver"), "urdf", "azure_kinect.urdf.xacro")
    print("Robot description xacro_file : {}".format(xacro_file))

    urdf_path = to_urdf(xacro_file)
    urdf = open(urdf_path).read()
    print("Robot description urdf_path : {}".format(urdf_path))

    # We'll read the parameters from a YAML file if it exists
    # The file path is set by the CONFIG_PATH environment variable
    # or defaults to /azure_kinect_0
    # This is used to override the default launch arguments
    param_file_path = os.getenv("CONFIG_PATH", "/azure_kinect_0")
    param_keys = [
        "depth_enabled",
        "depth_mode",
        "depth_unit",
        "color_enabled",
        "color_format",
        "color_resolution",
        "fps",
        "point_cloud",
        "rgb_point_cloud",
        "point_cloud_in_depth_frame",
        "sensor_sn",
        "recording_file",
        "recording_loop_enabled",
        "body_tracking_enabled",
        "body_tracking_smoothing_factor",
        "rescale_ir_to_mono8",
        "ir_mono8_scaling_factor",
        "imu_rate_target",
        "wired_sync_mode",
        "subordinate_delay_off_master_usec",
        # NEW color control parameters
        "exposure_time_absolute",
        "exposure_control_mode",
        "white_balance",
        "white_balance_control_mode",
        "brightness",
        "contrast",
        "saturation",
        "sharpness",
        "gain",
        "backlight_compensation",
        "powerline_frequency",
        # NEW camera namespace and name
        "camera_namespace",
        "camera_name",
    ]

    param_overrides = {}
    camera_namespace = "datahub_01"
    camera_name = "azure_kinect_0"

    if os.path.exists(param_file_path):
        print(f"INFO: Found parameter file at {param_file_path}. Overriding defaults.")
        with open(param_file_path, "r") as f:
            config = yaml.safe_load(f) or {}
        # collect overrides
        for k in param_keys:
            if k in config:
                val = config[k]
                if k == "sensor_sn":
                    # Force it to be a string by prefixing with 'x'
                    val = 'x' + str(val) if val != "" else str(val) # prefix with 'x' to avoid conflict with int
                param_overrides[k] = val
        # read camera namespace and name
        camera_namespace = config.get("camera_namespace", camera_namespace)
        camera_name = config.get("camera_name", camera_name)
    else:
        print(f"WARNING: No param file found at {param_file_path}. Using default launch arguments.")

    # Construct the full namespace (/namespace/device)
    full_namespace = f"/{camera_namespace}/{camera_name}"

    # These remappings are used to publish azure_description vs. robot_description
    remappings = [("robot_description", "azure_description")]

    return LaunchDescription(
        [
            # Push into the configured namespace
            PushRosNamespace(full_namespace),

            DeclareLaunchArgument(
                "overwrite_robot_description",
                default_value="true",
                description="Flag to publish a standalone azure_description instead of the default robot_description parameter.",
            ),
            # Default launch arguments (same as before)
            DeclareLaunchArgument("depth_enabled", default_value="true"),
            DeclareLaunchArgument("depth_mode", default_value="WFOV_UNBINNED"),
            DeclareLaunchArgument("depth_unit", default_value="16UC1"),
            DeclareLaunchArgument("color_enabled", default_value="true"),
            DeclareLaunchArgument("color_format", default_value="bgra"),
            DeclareLaunchArgument("color_resolution", default_value="1536P"),
            DeclareLaunchArgument("fps", default_value="5"),
            DeclareLaunchArgument("point_cloud", default_value="true"),
            DeclareLaunchArgument("rgb_point_cloud", default_value="true"),
            DeclareLaunchArgument("point_cloud_in_depth_frame", default_value="false"),
            DeclareLaunchArgument("required", default_value="false"),
            DeclareLaunchArgument("sensor_sn", default_value=""),
            DeclareLaunchArgument("recording_file", default_value=""),
            DeclareLaunchArgument("recording_loop_enabled", default_value="false"),
            DeclareLaunchArgument("body_tracking_enabled", default_value="false"),
            DeclareLaunchArgument("body_tracking_smoothing_factor", default_value="0.0"),
            DeclareLaunchArgument("rescale_ir_to_mono8", default_value="false"),
            DeclareLaunchArgument("ir_mono8_scaling_factor", default_value="1.0"),
            DeclareLaunchArgument("imu_rate_target", default_value="0"),
            DeclareLaunchArgument("wired_sync_mode", default_value="0"),
            DeclareLaunchArgument("subordinate_delay_off_master_usec", default_value="0"),
            # NEW color control launch arguments
            DeclareLaunchArgument(
                "exposure_time_absolute",
                default_value="16670",
                description="Exposure time in microseconds (manual mode).",
            ),
            DeclareLaunchArgument(
                "exposure_control_mode",
                default_value="auto",
                description="Exposure control mode: auto or manual"
            ),
            DeclareLaunchArgument(
                "white_balance",
                default_value="4500",
                description="White balance in Kelvin (manual mode).",
            ),
            DeclareLaunchArgument(
                "white_balance_control_mode",
                default_value="auto",
                description="White balance control mode: auto or manual"
            ),
            DeclareLaunchArgument(
                "brightness",
                default_value="128",
                description="Brightness (0–255).",
            ),
            DeclareLaunchArgument(
                "contrast",
                default_value="5",
                description="Contrast.",
            ),
            DeclareLaunchArgument(
                "saturation",
                default_value="32",
                description="Saturation.",
            ),
            DeclareLaunchArgument(
                "sharpness",
                default_value="2",
                description="Sharpness.",
            ),
            DeclareLaunchArgument(
                "gain",
                default_value="0",
                description="Gain.",
            ),
            DeclareLaunchArgument(
                "backlight_compensation",
                default_value="0",
                description="Backlight compensation (0=off, 1=on).",
            ),
            DeclareLaunchArgument(
                "powerline_frequency",
                default_value="2",
                description="Powerline frequency (1=50Hz, 2=60Hz).",
            ),
            # The main Azure Kinect node
            launch_ros.actions.Node(
                package="azure_kinect_ros_driver",
                executable="node",
                output="screen",
                # Pass default arguments plus param_overrides from /azure_kinect_0
                parameters=[
                    {
                        "depth_enabled": LaunchConfiguration("depth_enabled"),
                        "depth_mode": LaunchConfiguration("depth_mode"),
                        "depth_unit": LaunchConfiguration("depth_unit"),
                        "color_enabled": LaunchConfiguration("color_enabled"),
                        "color_format": LaunchConfiguration("color_format"),
                        "color_resolution": LaunchConfiguration("color_resolution"),
                        "fps": LaunchConfiguration("fps"),
                        "point_cloud": LaunchConfiguration("point_cloud"),
                        "rgb_point_cloud": LaunchConfiguration("rgb_point_cloud"),
                        "point_cloud_in_depth_frame": LaunchConfiguration("point_cloud_in_depth_frame"),
                        "sensor_sn": LaunchConfiguration("sensor_sn"),
                        "recording_file": LaunchConfiguration("recording_file"),
                        "recording_loop_enabled": LaunchConfiguration("recording_loop_enabled"),
                        "body_tracking_enabled": LaunchConfiguration("body_tracking_enabled"),
                        "body_tracking_smoothing_factor": LaunchConfiguration("body_tracking_smoothing_factor"),
                        "rescale_ir_to_mono8": LaunchConfiguration("rescale_ir_to_mono8"),
                        "ir_mono8_scaling_factor": LaunchConfiguration("ir_mono8_scaling_factor"),
                        "imu_rate_target": LaunchConfiguration("imu_rate_target"),
                        "wired_sync_mode": LaunchConfiguration("wired_sync_mode"),
                        "subordinate_delay_off_master_usec": LaunchConfiguration("subordinate_delay_off_master_usec"),
                        # NEW color control parameters
                        "exposure_time_absolute": LaunchConfiguration("exposure_time_absolute"),
                        "exposure_control_mode":    LaunchConfiguration("exposure_control_mode"),
                        "white_balance": LaunchConfiguration("white_balance"),
                        "white_balance_control_mode": LaunchConfiguration("white_balance_control_mode"),
                        "brightness": LaunchConfiguration("brightness"),
                        "contrast": LaunchConfiguration("contrast"),
                        "saturation": LaunchConfiguration("saturation"),
                        "sharpness": LaunchConfiguration("sharpness"),
                        "gain": LaunchConfiguration("gain"),
                        "backlight_compensation": LaunchConfiguration("backlight_compensation"),
                        "powerline_frequency": LaunchConfiguration("powerline_frequency"),
                    },
                    param_overrides,  # <-- This overrides the above if /azure_kinect_0 is found
                ],
            ),
            # Robot/Joint State Publishers
            launch_ros.actions.Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[{"robot_description": urdf}],
                condition=conditions.IfCondition(LaunchConfiguration("overwrite_robot_description")),
            ),
            launch_ros.actions.Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                arguments=[urdf_path],
                condition=conditions.IfCondition(LaunchConfiguration("overwrite_robot_description")),
            ),
            # If overwrite_robot_description is false, remap to azure_description
            launch_ros.actions.Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[{"robot_description": urdf}],
                remappings=remappings,
                condition=conditions.UnlessCondition(LaunchConfiguration("overwrite_robot_description")),
            ),
            launch_ros.actions.Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                arguments=[urdf_path],
                remappings=remappings,
                condition=conditions.UnlessCondition(LaunchConfiguration("overwrite_robot_description")),
            ),
        ]
    )
