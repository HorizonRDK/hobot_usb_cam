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
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


def generate_launch_description():
    camera_node = None

    print("using usb camera")
    # using usb cam publish image
    usb_cam_device_arg = DeclareLaunchArgument(
        'device',
        default_value='/dev/video8',
        description='usb camera device')

    usb_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_usb_cam'),
                'launch/hobot_usb_cam.launch.py')),
        launch_arguments={
            'usb_image_width': '640',
            'usb_image_height': '480',
            'usb_pixel_format': 'yuyv2rgb',
            'usb_zero_copy': 'False',
            'usb_video_device': LaunchConfiguration('device')
        }.items()
    )

    # nv12->jpeg
    jpeg_codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_codec'),
                'launch/hobot_codec_encode.launch.py')),
        launch_arguments={
            #'codec_in_mode': 'shared_mem',
            'codec_in_mode': 'ros',
            'codec_in_format': 'rgb8',
            'codec_out_mode': 'ros',
            #'codec_sub_topic': '/hbmem_img',
            'codec_sub_topic': '/image',
            'codec_pub_topic': '/image_mjpeg'
        }.items()
    )
    # web
    web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('websocket'),
                'launch/websocket.launch.py')),
        launch_arguments={
            'websocket_image_topic': '/image_mjpeg',
            'websocket_only_show_image': 'True'
        }.items()
    )


    return LaunchDescription([
        # 启动零拷贝环境配置node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('hobot_shm'),
                    'launch/hobot_shm.launch.py'))
        ),
        usb_cam_device_arg,
        usb_node,
        # image codec
        jpeg_codec_node,
        # web display
        web_node
    ])
