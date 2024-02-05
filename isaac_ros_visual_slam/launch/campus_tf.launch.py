from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "base_link", default_value="/r0_link", description=""
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_link_to_camera_link",
                arguments=[
                    "--x",
                    "0.0",
                    "--y",
                    "0.0",
                    "--z",
                    "0.0",
                    "--qx",
                    "0.0",
                    "--qy",
                    "0.0",
                    "--qz",
                    "0.0",
                    "--qw",
                    "1.0",
                    "--frame-id",
                    LaunchConfiguration("base_link"),
                    "--child-frame-id",
                    "acl_jackal2/forward_link",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="camera_link_to_camera_color_frame",
                arguments=[
                    "--x",
                    "0.0",
                    "--y",
                    "-0.059",
                    "--z",
                    "0.0",
                    "--qx",
                    "0.002",
                    "--qy",
                    "-0.002",
                    "--qz",
                    "0.0",
                    "--qw",
                    "1.0",
                    "--frame-id",
                    "acl_jackal2/forward_link",
                    "--child-frame-id",
                    "acl_jackal2/forward_color_frame",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="camera_link_to_camera_depth_frame",
                arguments=[
                    "--x",
                    "0.0",
                    "--y",
                    "0.0",
                    "--z",
                    "0.0",
                    "--qx",
                    "0.0",
                    "--qy",
                    "0.0",
                    "--qz",
                    "0.0",
                    "--qw",
                    "1.0",
                    "--frame-id",
                    "acl_jackal2/forward_link",
                    "--child-frame-id",
                    "acl_jackal2/forward_depth_frame",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="camera_depth_frame_to_optical_frame",
                arguments=[
                    "--x",
                    "0.0",
                    "--y",
                    "0.0",
                    "--z",
                    "0.0",
                    "--qx",
                    "-0.5",
                    "--qy",
                    "0.5",
                    "--qz",
                    "-0.5",
                    "--qw",
                    "0.5",
                    "--frame-id",
                    "acl_jackal2/forward_depth_frame",
                    "--child-frame-id",
                    "acl_jackal2/forward_depth_optical_frame",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="camera_color_frame_to_optical_frame",
                arguments=[
                    "--x",
                    "0.0",
                    "--y",
                    "0.0",
                    "--z",
                    "0.0",
                    "--qx",
                    "-0.5",
                    "--qy",
                    "0.5",
                    "--qz",
                    "-0.5",
                    "--qw",
                    "0.5",
                    "--frame-id",
                    "acl_jackal2/forward_color_frame",
                    "--child-frame-id",
                    "acl_jackal2/forward_color_optical_frame",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="gyro_optical_frame_to_imu_optical_frame",
                arguments=[
                    "--x",
                    "0.0",
                    "--y",
                    "0.0",
                    "--z",
                    "0.0",
                    "--qx",
                    "0.0",
                    "--qy",
                    "0.0",
                    "--qz",
                    "0.0",
                    "--qw",
                    "1.0",
                    "--frame-id",
                    "acl_jackal2/forward_gyro_optical_frame",
                    "--child-frame-id",
                    "acl_jackal2/forward_imu_optical_frame"
                ],
            ),

            # camera_link to camera_gyro_frame
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="camera_link_to_camera_gyro_frame",
                arguments=[
                    "--x",
                    "-0.016",
                    "--y",
                    "-0.030",
                    "--z",
                    "0.007",
                    "--qx",
                    "0.0",
                    "--qy",
                    "0.0",
                    "--qz",
                    "0.0",
                    "--qw",
                    "1.0",
                    "--frame-id",
                    LaunchConfiguration("base_link"),
                    "--child-frame-id",
                    "acl_jackal2/forward_gyro_frame",
                ],
            ),

            # camera_gyro_frame to camera_gyro_optical_frame
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="camera_gyro_frame_to_camera_gyro_optical_frame",
                arguments=[
                    "--x",
                    "0.0",
                    "--y",
                    "0.0",
                    "--z",
                    "0.0",
                    "--qx",
                    "-0.5",
                    "--qy",
                    "0.5",
                    "--qz",
                    "-0.5",
                    "--qw",
                    "0.5",
                    "--frame-id",
                    "acl_jackal2/forward_gyro_frame",
                    "--child-frame-id",
                    "acl_jackal2/forward_gyro_optical_frame",
                ],
            ),
        ]
    )
