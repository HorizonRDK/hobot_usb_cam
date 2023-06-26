// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef HOBOT_USB_CAM_HPP_
#define HOBOT_USB_CAM_HPP_
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "sensor_msgs/msg/camera_info.hpp"

extern "C" {
#include <linux/videodev2.h>
}

namespace hobot_usb_cam {
class HobotUSBCam {
 public:
  typedef enum {
    kIO_METHOD_READ,
    kIO_METHOD_MMAP,
    kIO_METHOD_USERPTR
  } IOMethod;
  typedef enum {
    kPIXEL_FORMAT_YUYV,
    kPIXEL_FORMAT_UYVY,
    kPIXEL_FORMAT_MJPEG,
    kPIXEL_FORMAT_YUVMONO10,
    kPIXEL_FORMAT_RGB24,
    kPIXEL_FORMAT_GREY
  } PixelFormat;
  struct CamInformation {
    CamInformation() = default;
    // CamInformation(const CamInformation &cam_information);
    std::string dev;
    IOMethod io;
    PixelFormat pixel_format;
    int32_t image_width;
    int32_t image_height;
    int32_t framerate;
  };
  struct CamBuffer {
    CamBuffer() = default;
    void *start;
    size_t length;
    std::chrono::system_clock::time_point time_point;
    struct v4l2_buffer reserved_buffer;
  };
  HobotUSBCam();
  ~HobotUSBCam();
  bool Init(CamInformation &cam_infomation);
  bool Start(void);
  bool Stop(void);
  bool DeInit();
  bool GetFrame(CamBuffer &cam_buffer);
  bool ReleaseFrame(CamBuffer &cam_buffer);
  bool ReadCalibrationFile(sensor_msgs::msg::CameraInfo &cam_calibration_info,
                           const std::string &file_path);

  // check分辨率是否支持，如果不支持返回false并输出error log
  bool CheckResolutionFromFormats(int width, int height);

 private:
  int32_t xioctl(int fh, uint32_t request, void *arg);
  void errno_exit(const char *s);
  bool OpenDevice(void);
  bool InitDevice(void);
  bool InitRead(unsigned int buffer_size);
  bool InitMmap(void);
  bool InitUserspace(unsigned int buffer_size);
  bool CloseDevice(void);
  bool ReadFrame(CamBuffer &cam_buffer);
  void GetFormats();  //获取device支持的图片格式，分辨率以及framerate

  struct strFormats {
    int width;
    int height;
    std::vector<int> frameRate;
  };
  std::map<uint32_t, std::vector<strFormats> >
      map_formats;  // key = pixelformat
                    // 用于保存device支持的格式，分辨率以及帧率

  /* data */
  typedef enum {
    kSTATE_INITIALLED,
    kSTATE_RUNING,
    kSTATE_STOP,
    kSTATE_UNINITIALLED
  } CamState;
  CamState cam_state_;
  std::mutex cam_mutex_;
  CamInformation cam_information_;
  int cam_fd_;
  int buffer_numbers_;
  CamBuffer *buffers_;
};
}  // namespace hobot_usb_cam
#endif  // USB_CAM__USB_CAM_NODE_HPP_
