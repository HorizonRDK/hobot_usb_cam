English| [简体中文](./README_cn.md)

# Function Introduction

The **hobot_usb_cam** collects image data from a USB camera and publishes it as ROS standard image messages or zero-copy (hbmem) image messages for other modules that need to use the image data to subscribe to.

# Bill of Materials

| Number | Name  | Manufacturer | Reference Link                            |
| ------ | ------ | ------------ | ----------------------------------------- |
| 1      | USB Camera | Multiple Manufacturers | Self-selection |
| 2      | RDK X3 | Multiple Manufacturers | [Click Here](https://developer.horizon.cc/rdkx3) |

# Instructions for Use

## Hardware Assembly

Connect the USB camera to the USB slot on the RDK.

## Install the Package

After starting the RDK, connect to the RDK via terminal or VNC, copy and run the following command on the RDK system to install the relevant nodes.

```bash
sudo apt update
sudo apt install -y tros-hobot-usb-cam
```

## Publish Images using USB Camera

Run the following command in the terminal of the RDK system to start the connected camera:

```bash
# Configure the tros.b environment:
source /opt/tros/setup.bash
# Launch with specified USB camera device name /dev/video8
ros2 launch hobot_usb_cam hobot_usb_cam.launch.py usb_video_device:=/dev/video8
```

hobot_usb_cam.launch.py configures default output of MJPEG format images with a resolution of 640x480, and publishes to the topic named `/image`.

To use other resolutions, you can specify them in the launch command, for example, to publish 1920x1080 resolution MJPEG format images: `ros2 launch hobot_usb_cam hobot_usb_cam.launch.py usb_video_device:=/dev/video8 usb_image_width:=1920 usb_image_height:=1080`.

If the following information is output, it indicates that the node has been successfully launched:

```text
[INFO] [launch]: All log files can be found below /root/.ros/log/2023-07-18-18-30-54-961749-ubuntu-16326
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [hobot_usb_cam-1]: process started with pid [16328]
[hobot_usb_cam-1] If you need calibration msg, please make sure the calibration file path is correct and the calibration file exists!
[hobot_usb_cam-1] [WARN] [1689676255.349211776] [hobot_usb_cam]: get camera calibration parameters failed
[hobot_usb_cam-1] [WARN] [1689676255.349432974] [hobot_usb_cam]: Start to open device /dev/video8.
[hobot_usb_cam-1] [WARN] [1689676255.596271213] [hobot_usb_cam]: Open & Init device /dev/video8 success.
```
## View Effect

Here we use the web interface to visualize the image. Open a new terminal for publishing the web service.

In a new terminal, start the following commands:

```shell
source /opt/tros/local_setup.bash
# Start websocket
ros2 launch websocket websocket.launch.py websocket_image_topic:=/image websocket_only_show_image:=true
```

Open a web browser on the same network computer, visit the IP address (type http://IP:8000 in the browser, where IP is the Horizon RDK IP address), then click on `Web Display` on the top left to see the real-time image output from the USB camera:
     ![web_usb](./image/web_usb.png "Real-time Image")


# API Description

## Topics

### Published Topics
| Name         | Message Type                         | Description                                      |
| ------------ | ------------------------------------  | -------------------------------------------------|
| /camera_info | sensor_msgs/msg/CameraInfo           | Camera intrinsics topic published based on the camera calibration file set |
| /image_raw   | sensor_msgs/msg/Image                | Periodically published image topic in JPEG format |
| /hbmem_image | [hbm_img_msgs/msg/HbmMsg1080P](https://github.com/HorizonRDK/hobot_msgs/blob/develop/hbm_img_msgs/msg/HbmMsg1080P.msg) | Image topic based on shared memory, in JPEG format |

## Parameters
| Parameter Name | Description                | Type   | Supported Configurations         | Required | Default Value       |
| -------------- | -------------------------- | ------ | ---------------------------------| -------- | ------------------- |
| frame_id       | Message identifier         | string | Set the frame_id as needed       | No       | "default_usb_cam"   |
| framerate      | Frame rate                 | int    | Choose as per sensor support     | No       | 30                  | 
| image_height   | Image height resolution    | int    | Choose as per sensor support     | No       | 640                 |
| image_width    | Image width resolution     | int    | Choose as per sensor support     | No       | 480                 |
| io_method      | IO method for obtaining images from USB camera | string | mmap/read/userptr          | No       | "mmap"              |
| pixel_format   | Image encoding format for publishing | string | "mjpeg,mjpeg2rgb,rgb8,yuyv,yuyv2rgb,uyvy,uyvy2rgb,m4202rgb,mono8,mono16,y102mono8" | No | "mjpeg" |
| video_device   | Device driver name          | string | Device name usually as /dev/videox | No       | "/dev/video0"       |
| zero_copy      | Image transmission mode, enabling shared_mem will use zero-copy mechanism for transmission | bool  | True/False                  | No       | "True"              |
| camera_calibration_file_path | Path to store the camera calibration file | string | Configure according to the actual camera calibration file path | No  | "/opt/tros/${TROS_DISTRO}/lib/hobot_usb_cam/config/usb_camera_calibration.yaml" |

# FAQs

1. How to set the video_device parameter?

    After connecting the USB camera to RDK, a new device number will appear, such as `/dev/video8`. Use this device number as the video_device parameter.Additionally, this Node supports automatic adaptation of device numbers. If set incorrectly, it will automatically adapt at runtime.

2. Will the absence of a camera calibration file affect camera functionality?

   It will not affect. 
   
   If there is no camera calibration file, the camera intrinsic parameters cannot be published, but it will not affect the image acquisition and publishing functions.

3. Is the setting of pixel_format correct?
    ```text
    [hobot_usb_cam-1] [WARN] [1705548544.174669672] [hobot_usb_cam]: This devices supported formats:
    [hobot_usb_cam-1] [WARN] [1705548544.174844917] [hobot_usb_cam]:        Motion-JPEG: 640 x 480 (30 Hz)
    [hobot_usb_cam-1] [WARN] [1705548544.174903166] [hobot_usb_cam]:        Motion-JPEG: 1920 x 1080 (30 Hz)
    [hobot_usb_cam-1] [WARN] [1705548544.174950581] [hobot_usb_cam]:        Motion-JPEG: 320 x 240 (30 Hz)
    [hobot_usb_cam-1] [WARN] [1705548544.174996788] [hobot_usb_cam]:        Motion-JPEG: 800 x 600 (30 Hz)
    [hobot_usb_cam-1] [WARN] [1705548544.175043412] [hobot_usb_cam]:        Motion-JPEG: 1280 x 720 (30 Hz)
    [hobot_usb_cam-1] [WARN] [1705548544.175089161] [hobot_usb_cam]:        Motion-JPEG: 1024 x 576 (30 Hz)
    [hobot_usb_cam-1] [WARN] [1705548544.175135035] [hobot_usb_cam]:        YUYV 4:2:2: 640 x 480 (30 Hz)
    [hobot_usb_cam-1] [WARN] [1705548544.175180325] [hobot_usb_cam]:        YUYV 4:2:2: 1920 x 1080 (5 Hz)
    [hobot_usb_cam-1] [WARN] [1705548544.175226449] [hobot_usb_cam]:        YUYV 4:2:2: 320 x 240 (30 Hz)
    [hobot_usb_cam-1] [WARN] [1705548544.175272365] [hobot_usb_cam]:        YUYV 4:2:2: 800 x 600 (20 Hz)
    [hobot_usb_cam-1] [WARN] [1705548544.175318697] [hobot_usb_cam]:        YUYV 4:2:2: 1280 x 720 (10 Hz)
    [hobot_usb_cam-1] [WARN] [1705548544.175365195] [hobot_usb_cam]:        YUYV 4:2:2: 1024 x 576 (15 Hz)
    ```
    a. Query the supported image formats of the usb camera, as shown in the above log, which displays support for MJPEG and YUYV;
    b. Modify the configuration of pixel_format, but it must be a format supported by the usb camera, otherwise hobot_usb_cam will exit.
