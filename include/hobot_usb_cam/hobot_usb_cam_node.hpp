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

#ifndef HOBOT_USB_CAM_NODE_HPP_
#define HOBOT_USB_CAM_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include <vector>

#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#include "hbm_img_msgs/msg/hbm_msg480_p.hpp"
#include "hbm_img_msgs/msg/hbm_msg540_p.hpp"
#include "hobot_usb_cam.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace hobot_usb_cam {
class HobotUSBCamNode : public rclcpp::Node {
 public:
  explicit HobotUSBCamNode(const rclcpp::NodeOptions &ndoe_options);
  ~HobotUSBCamNode();

 private:
  typedef enum {
    kSTATE_INITIALLED,
    kSTATE_RUNING,
    kSTATE_STOP,
    kSTATE_UNINITIALLED
  } CamNodeState;
  bool GetParams(void);
  bool AssignParams(const std::vector<rclcpp::Parameter> &parameters);
  bool SetPixelFormat(const std::string &pixel_format_name);
  bool SetIOMethod(const std::string &io_method_name);
  bool SetPublisher(void);
  void ReadFrame(void);
  HobotUSBCam cam_;
  std::string video_device_name_;
  std::string frame_id_;
  std::string camera_calibration_file_path_;
  HobotUSBCam::IOMethod io_method_name_;
  HobotUSBCam::PixelFormat pixel_format_name_;
  int image_width_;
  int image_height_;
  int framerate_;
  bool zero_copy_enabled_;
  bool read_cam_calibration_enabled_ = true;
  bool reading_;
  std::thread *read_thread_;
  CamNodeState cam_node_state_;

  sensor_msgs::msg::CameraInfo camera_calibration_info_;

  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_ =
      nullptr;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_ = nullptr;
  rclcpp::PublisherHbmem<hbm_img_msgs::msg::HbmMsg1080P>::SharedPtr
      hbmem_image_pub_1080_;
  rclcpp::PublisherHbmem<hbm_img_msgs::msg::HbmMsg540P>::SharedPtr
      hbmem_image_pub_540_;
  rclcpp::PublisherHbmem<hbm_img_msgs::msg::HbmMsg480P>::SharedPtr
      hbmem_image_pub_480_;
};
}  // namespace hobot_usb_cam

#endif  // HOBOT_USB_CAM_NODE_HPP_
