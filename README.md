# 功能介绍

hobot_usb_cam从USB摄像头采集图像数据，以ROS标准图像消息或者零拷贝（hbmem）图像消息进行发布，供需要使用图像数据的其他模块订阅。

# 物料清单

- USB摄像头，自行采购。

- [RDK X3](https://developer.horizon.cc/sunrise)


# 使用方法

## 硬件连接

将USB摄像头连接到RDK X3的USB插槽。

## 功能安装

在RDK系统的终端中运行如下指令，即可快速安装：

```bash
sudo apt update
sudo apt install -y tros-hobot-usb-cam
```

## 启动相机

在RDK系统的终端中运行如下指令，启动已连接的相机：

```bash
# 配置 tros.b 环境：
source /opt/tros/setup.bash
# launch 方式启动，指定了USB摄像头设备名称为/dev/video8
ros2 launch hobot_usb_cam hobot_usb_cam.launch.py usb_video_device:=/dev/video8
```

hobot_usb_cam.launch.py配置默认输出640x480分辨率mjpeg格式图像，发布的话题名称为`/image`。

如需使用其他分辨率可以在启动命令中指定，比如发布1920x1080分辨率mjpeg格式图像：`ros2 launch hobot_usb_cam hobot_usb_cam.launch.py usb_video_device:=/dev/video8 usb_image_width:=1920 usb_image_height:=1080`。

如程序输出如下信息，说明节点已成功启动

```text
[INFO] [launch]: All log files can be found below /root/.ros/log/2023-07-18-18-30-54-961749-ubuntu-16326
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [hobot_usb_cam-1]: process started with pid [16328]
[hobot_usb_cam-1] [WARN] [1689676255.348424228] [hobot_usb_cam]: Camera calibration file: [/opt/tros/lib/hobot_usb_cam/config/usb_camera_calibration.yaml] does not exist!
[hobot_usb_cam-1] If you need calibration msg, please make sure the calibration file path is correct and the calibration file exists!
[hobot_usb_cam-1] [WARN] [1689676255.349211776] [hobot_usb_cam]: get camera calibration parameters failed
[hobot_usb_cam-1] [WARN] [1689676255.349432974] [hobot_usb_cam]: Start to open device /dev/video8.
[hobot_usb_cam-1] [WARN] [1689676255.596271213] [hobot_usb_cam]: Open & Init device /dev/video8 success.
```

## 图像可视化

### 使用WEB浏览器

这里采用web端方式实现图像可视化，另起一个终端用于webservice发布。

 打开一个新的终端，启动如下命令：

```shell
source /opt/tros/local_setup.bash
# 启动websocket
ros2 launch websocket websocket.launch.py websocket_image_topic:=/image websocket_only_show_image:=true
```

PC打开浏览器（chrome/firefox/edge）输入<http://IP:8000>（IP为地平线RDK IP地址），点击左上方`Web 展示端`即可看到USB摄像头输出的实时画面：
    ![web_usb](./image/web_usb.png "实时图像")


# 接口说明

## 话题

### 发布话题
| 名称         | 消息类型                             | 说明                                     |
| ------------ | ------------------------------------ | ---------------------------------------- |
| /camera_info | sensor_msgs/msg/CameraInfo           | 相机内参话题，根据设置的相机标定文件发布 |
| /image_raw   | sensor_msgs/msg/Image                | 周期发布的图像话题，jpeg格式             |
| /hbmem_image   | hbm_img_msgs/msg/HbmMsg1080P | 基于共享内存share mem的图像话题，jpeg格式        |

## 参数

| 名称                         | 参数值                                          | 说明                                               |
| ---------------------------- | ----------------------------------------------- | -------------------------------------------------- |
| frame_id                 | "default_usb_cam"（默认） | 消息标志符                         |
| framerate                 | 30（默认） | 帧率                         |
| image_width                  | 640（默认）                                    | 图像宽方向分辨率                                   |
| image_height                 | 480（默认）                                    | 图像高方向分辨率                                   |
| pixel_format                   | "mjpeg"（默认）                       | 发布图像编码方式                              |
| io_method                    | mmap（默认）<br />read<br />userptr        | 从USB摄像头获取图像的io类型  |
| zero_copy                    | True（默认）<br />False                      | 图像传输方式，配置shared_mem后将使用零拷贝机制传输 |
| video_device | "/dev/video8"（默认）<br />"/dev/video0"等             | 设备驱动名称                                 |
| camera_calibration_file_path | /opt/tros/lib/hobot_usb_cam/config/usb_camera_calibration.yaml（默认）                                      | 相机标定文件的路径                                 |

# 常见问题

1. 如何设置video_device参数？

    RDK接入USB摄像头之后会出现新的设备号，例如`/dev/video8`，使用此设备号作为video_device参数。

    另外，该Node支持设备号自适应，如果设置错误，运行时会自动适配。

2. 无相机标定文件是否影响相机功能？

   不影响。
   
   如果没有相机标定文件，则无法发布相机内参，但不影响图像获取和发布功能。
