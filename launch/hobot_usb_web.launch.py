# Copyright (c) 2022，Horizon Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from ament_index_python.packages import get_package_prefix

def generate_launch_description():

    # include web launch file
    web_service_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('websocket'),
                'launch/websocket_service.launch.py'))
    )

    image_width_launch_arg = DeclareLaunchArgument(
        "image_width", default_value=TextSubstitution(text="960")
    )
    image_height_launch_arg = DeclareLaunchArgument(
        "image_height", default_value=TextSubstitution(text="544")
    )

    # usb cam图片发布pkg
    usb_node = Node(
        package='hobot_usb_cam',
        executable='hobot_usb_cam',
        name='hobot_usb_cam',
        parameters=[
                    {"frame_id": "default_usb_cam"},
                    {"image_height": 480},
                    {"image_width": 640},
                    {"zero_copy": False},
                    {"video_device": "/dev/video8"}
                    ],
        arguments=['--ros-args', '--log-level', 'error']
    )

    # web展示pkg
    web_node = Node(
        package='websocket',
        executable='websocket',
        output='screen',
        parameters=[
            {"image_topic": "/image"},
            {"image_type": "mjpeg"},
            {"only_show_image": True},
            {"smart_topic": "/hobot_dnn_detection"}
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    print("using usb cam")
    cam_node = usb_node
    camera_type_mipi = False

    return LaunchDescription([
            web_service_launch_include,
            image_width_launch_arg,
            image_height_launch_arg,
            # 图片发布pkg
            cam_node,
            # 启动web展示pkg
            web_node
        ])
