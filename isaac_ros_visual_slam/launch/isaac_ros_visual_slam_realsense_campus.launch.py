# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

# fmt: off
# turn off formatting to not scare people

def launch_setup(context, *args, **kwargs):
    visual_slam_node = ComposableNode(
        name="visual_slam_node",
        package="isaac_ros_visual_slam",
        plugin="nvidia::isaac_ros::visual_slam::VisualSlamNode",
        parameters=[
            {
                "denoise_input_images": False,
                "rectified_images": True,
                "enable_debug_mode": False,
                "debug_dump_path": "/tmp/cuvslam",
                "enable_slam_visualization": True,
                "enable_landmarks_view": True,
                "enable_observations_view": True,
                "map_frame": "map",
                "odom_frame": "odom",
                "base_frame": LaunchConfiguration("base_link").perform(context),
                "input_imu_frame": LaunchConfiguration("imu_frame").perform(context),
                "enable_imu_fusion": True,
                "gyro_noise_density": 0.04,
                "gyro_random_walk": 0.00266,
                "accel_noise_density": 0.04,
                "accel_random_walk": 0.00433,
                "calibration_frequency": 200.0,
                "img_jitter_threshold_ms": 22.00,
                "enable_localization_n_mapping": True,
            }
        ],
        remappings=[
                ("stereo_camera/left/image", LaunchConfiguration("camera_name").perform(context) + "/infra1/image_rect_raw"),
                ("stereo_camera/left/camera_info", LaunchConfiguration("camera_name").perform(context) + "/infra1/camera_info"),
                ("stereo_camera/right/image", LaunchConfiguration("camera_name").perform(context) + "/infra2/image_rect_raw"),
                ("stereo_camera/right/camera_info", LaunchConfiguration("camera_name").perform(context) + "/infra2/camera_info"),
                ("visual_slam/imu", LaunchConfiguration("camera_name").perform(context) + "/imu"),
        ],
    )

    decompress_node_infra1=Node(
        package="image_transport",
        executable="republish",
        name="decompress_node_infra1",
        arguments=["compressed", "raw"],
        remappings=[
            ("in/compressed", LaunchConfiguration("camera_name").perform(context) + "/infra1/image_rect_raw/compressed"),
            ("out", LaunchConfiguration("camera_name").perform(context) + "/infra1/image_rect_raw"),
        ],
        output="screen",
    )

    decompress_node_infra2=Node(
        package="image_transport",
        executable="republish",
        name="decompress_node_infra2",
        arguments=["compressed", "raw"],
        remappings=[
            ("in/compressed", LaunchConfiguration("camera_name").perform(context) + "/infra2/image_rect_raw/compressed"),
            ("out", LaunchConfiguration("camera_name").perform(context) + "/infra2/image_rect_raw"),
        ],
        output="screen",
    )

    # Compute quaternion of the IMU
    # imu_interp_node=Node(
    #     package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
    #     parameters=[{'use_mag': False, 
    #                     'world_frame':'enu', 
    #                     'publish_tf':False}],
    #     remappings=[('imu/data_raw', LaunchConfiguration('camera_name').perform(context) + '/forward/imu'),
    #                 ('imu/data', LaunchConfiguration('camera_name').perform(context) + '/forward/imu/data')])


    visual_slam_launch_container = ComposableNodeContainer(
        name="visual_slam_launch_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[visual_slam_node],
        output="screen",
    )

    play_rosbag = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration("play")),
        cmd=["ros2", "bag", "play", LaunchConfiguration("bag_file"),
            "--clock",
            "-r", LaunchConfiguration("rate"),
            "--start-offset", LaunchConfiguration("start"),
            # "--playback-duration", LaunchConfiguration("duration"),
        ],
        shell=True,
    )

    # include campus_tf.launch.py here
    tf_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("isaac_ros_visual_slam"), "launch", "campus_tf.launch.py",),
        ),
        launch_arguments={
            "base_link": LaunchConfiguration("base_link"),
        }.items(),
    )

    return [visual_slam_launch_container, play_rosbag, tf_launch, decompress_node_infra1, decompress_node_infra2]

def generate_launch_description():
    robot_id = DeclareLaunchArgument( "robot_id", default_value="", description="Robot ID",)
    camera_name = DeclareLaunchArgument( "camera_name", default_value="/sparkal1/forward", description="Camera Name",)
    rate = DeclareLaunchArgument( "rate", default_value="1.0", description="Rate at which to play the bag file",)
    start = DeclareLaunchArgument( "start", default_value="0.1", description="Start time to play the bag file",)
    duration = DeclareLaunchArgument( "duration", default_value="-1.0", description="Duration for which to play the bag file",)
    bag_file = DeclareLaunchArgument( "bag_file", default_value="/datasets/campus-x/outdoor/10_14_sparkal1", description="Path to bag file",)
    play_arg = DeclareLaunchArgument( "play", default_value="true", description="Enable playing of topics with rosbag",)
    base_link=DeclareLaunchArgument("base_link", default_value="camera_link", description="Base Link")
    imu_frame=DeclareLaunchArgument("imu_frame", default_value="acl_jackal2/forward_gyro_optical_frame", description="IMU Frame")

    return launch.LaunchDescription(
        [
            robot_id,
            camera_name,
            rate,
            start,
            duration,
            bag_file,
            play_arg,
            base_link,
            imu_frame,
            OpaqueFunction(function=launch_setup),
        ]
    )
