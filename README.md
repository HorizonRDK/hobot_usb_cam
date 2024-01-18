# 功能介绍

hobot_usb_cam从USB摄像头采集图像数据，以ROS标准图像消息或者零拷贝（hbmem）图像消息进行发布，供需要使用图像数据的其他模块订阅。

# 物料清单

| 序号 | 名称   | 生产厂家 | 参考链接                                                     |
| ---- | ------ | -------- | ------------------------------------------------------------ |
| 1    | USB摄像头    | 多厂家 | 自行选择 |
| 2    | RDK X3 | 多厂家 | [点击跳转](https://developer.horizon.cc/rdkx3) |

# 使用方法

## 硬件组装

将USB摄像头连接到RDK的USB插槽。

## 安装功能包

启动RDK后，通过终端或者VNC连接RDK，复制如下命令在RDK的系统上运行，完成相关Node的安装。

```bash
sudo apt update
sudo apt install -y tros-hobot-usb-cam
```

## 使用USB摄像头发布图片

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

## 查看效果

这里采用web端方式实现图像可视化，另起一个终端用于webservice发布。

打开一个新的终端，启动如下命令：

```shell
source /opt/tros/local_setup.bash
# 启动websocket
ros2 launch websocket websocket.launch.py websocket_image_topic:=/image websocket_only_show_image:=true
```

打开同一网络电脑的浏览器，访问IP地址（浏览器输入http://IP:8000，IP为地平线RDK IP地址），点击左上方`Web 展示端`即可看到USB摄像头输出的实时画面：
     ![web_usb](./image/web_usb.png "实时图像")


# 接口说明

## 话题

### 发布话题
| 名称         | 消息类型                             | 说明                                     |
| ------------ | ------------------------------------ | ---------------------------------------- |
| /camera_info | sensor_msgs/msg/CameraInfo           | 相机内参话题，根据设置的相机标定文件发布 |
| /image_raw   | sensor_msgs/msg/Image                | 周期发布的图像话题，jpeg格式             |
| /hbmem_image   | [hbm_img_msgs/msg/HbmMsg1080P](https://github.com/HorizonRDK/hobot_msgs/blob/develop/hbm_img_msgs/msg/HbmMsg1080P.msg) | 基于共享内存share mem的图像话题，jpeg格式        |

## 参数
| 参数名      | 解释             | 类型   | 支持的配置                 | 是否必须 | 默认值             |
| ------------| -----------------| -------| --------------------------| -------- | -------------------|
| frame_id    | 消息标志符       | string | 根据需要设置frame_id名字   | 否       | "default_usb_cam"  |
| framerate   | 帧率             | int    | 根据sensor支持选择         | 否       | 30                 |
| image_height| 图像高方向分辨率 | int    | 根据sensor支持选择         | 否       | 640                |
| image_width | 图像宽方向分辨率 | int    | 根据sensor支持选择         | 否        | 480               |
| io_method   | 从USB摄像头获取图像的io类型            | string | mmap/read/userptr          | 否       | "mmap"         |
| pixel_format| 发布图像编码方式          | string | "mjpeg,mjpeg-compressed,mjpeg2rgb,rgb8,yuyv,yuyv2rgb,uyvy,uyvy2rgb,m4202rgb,mono8,mono16,y102mono8"        | 否        | “mjpeg”           |
| video_device| 设备驱动名称     | string | 设备名称一般为/dev/videox  | 否        | "/dev/video0"     |
| zero_copy   | 图像传输方式，配置shared_mem后将使用零拷贝机制传输   | bool   | True/False                 | 否       | "True"           |
| camera_calibration_file_path  | 相机标定文件的存放路径  | string   | 根据实际的相机标定文件存放路径配置   | 否  | "/opt/tros/lib/hobot_usb_cam/config/usb_camera_calibration.yaml" |

## 注意事项
 ```text
[hobot_usb_cam-1] [WARN] [1705548544.174669672] [hobot_usb_cam]: This devices supproted formats:
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
1.查询usb camera支持的图像格式，如上述log，log显示支持mjpeg和YUYV;
2.修改pixel_format的配置，但是必须usb camera支持的格式，否则hobot_usb_cam退出。

# 常见问题

1. 如何设置video_device参数？

    RDK接入USB摄像头之后会出现新的设备号，例如`/dev/video8`，使用此设备号作为video_device参数。

    另外，该Node支持设备号自适应，如果设置错误，运行时会自动适配。

2. 无相机标定文件是否影响相机功能？

   不影响。
   
   如果没有相机标定文件，则无法发布相机内参，但不影响图像获取和发布功能。
