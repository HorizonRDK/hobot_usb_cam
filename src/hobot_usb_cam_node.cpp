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

#include "hobot_usb_cam_node.hpp"

#include <fstream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "hobot_usb_cam.hpp"

namespace hobot_usb_cam {
HobotUSBCamNode::HobotUSBCamNode(const rclcpp::NodeOptions &ndoe_options)
    : Node("hobot_usb_cam", ndoe_options),
      reading_(false),
      read_thread_(nullptr),
      cam_node_state_(kSTATE_UNINITIALLED) {
  this->declare_parameter("framerate", 30);
  this->declare_parameter("frame_id", "default_usb_cam");
  this->declare_parameter("image_height", 480);
  this->declare_parameter("image_width", 640);
  this->declare_parameter("io_method", "mmap");
  this->declare_parameter("pixel_format", "mjpeg");
  this->declare_parameter("video_device", "/dev/video0");
  this->declare_parameter("zero_copy", true);
  this->declare_parameter("camera_calibration_file_path", "");

  if (GetParams() == false) {
    RCLCPP_ERROR(this->get_logger(), "Hobot USB Cam GetParams() failed\n\n");
    return;
  }
  if (!cam_.ReadCalibrationFile(camera_calibration_info_,
                                camera_calibration_file_path_)) {
    read_cam_calibration_enabled_ = false;
    RCLCPP_WARN(rclcpp::get_logger("hobot_usb_cam"),
                "get camera calibration parameters failed");
  }

  HobotUSBCam::CamInformation cam_information;
  cam_information.dev = video_device_name_;
  cam_information.framerate = framerate_;
  cam_information.image_height = image_height_;
  cam_information.image_width = image_width_;
  cam_information.io = io_method_name_;
  cam_information.pixel_format = pixel_format_name_;
  if (cam_.Init(cam_information) == false) {
    RCLCPP_ERROR(this->get_logger(), "Hobot USB Cam Init failed\n\n");
    return;
  }
  cam_node_state_ = kSTATE_INITIALLED;
  // Update camera params.
  framerate_ = cam_information.framerate;
  image_height_ = cam_information.image_height;
  image_width_ = cam_information.image_width;
  pixel_format_name_ = cam_information.pixel_format;

  // This should be done after get_params()
  if (SetPublisher() == false) {
    return;
  }
  if (cam_.Start() == false) {
    RCLCPP_ERROR(this->get_logger(), "Hobot USB Cam Start failed\n\n");
    return;
  }
  cam_node_state_ = kSTATE_RUNING;
  reading_ = true;
  read_thread_ = new std::thread(&HobotUSBCamNode::ReadFrame, this);
}

HobotUSBCamNode::~HobotUSBCamNode() {
  reading_ = false;
  if (read_thread_ != nullptr) read_thread_->join();
  if (cam_node_state_ == kSTATE_RUNING) {
    if (cam_.Stop() == false) {
      RCLCPP_ERROR(this->get_logger(), "Hobot USB Cam Stop failed\n\n");
    }
    cam_node_state_ = kSTATE_STOP;
  }
  if (cam_node_state_ == kSTATE_STOP) {
    if (cam_.DeInit() == false) {
      RCLCPP_ERROR(this->get_logger(), "Hobot USB Cam Dinit failed\n\n");
    }
  }
}

bool HobotUSBCamNode::GetParams() {
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);
  auto parameters =
      parameters_client->get_parameters({"frame_id",
                                         "framerate",
                                         "image_height",
                                         "image_width",
                                         "io_method",
                                         "pixel_format",
                                         "video_device",
                                         "zero_copy",
                                         "camera_calibration_file_path"});
  return AssignParams(parameters);
}

bool HobotUSBCamNode::AssignParams(
    const std::vector<rclcpp::Parameter> &parameters) {
  for (auto &parameter : parameters) {
    if (parameter.get_name() == "frame_id") {
      frame_id_ = parameter.value_to_string();
    } else if (parameter.get_name() == "framerate") {
      framerate_ = parameter.as_int();
    } else if (parameter.get_name() == "image_height") {
      image_height_ = parameter.as_int();
    } else if (parameter.get_name() == "image_width") {
      image_width_ = parameter.as_int();
    } else if (parameter.get_name() == "io_method") {
      std::string io_method_name = parameter.value_to_string();
      if (SetIOMethod(io_method_name) == false) return false;
    } else if (parameter.get_name() == "pixel_format") {
      std::string pixel_format_name = parameter.value_to_string();
      if (SetPixelFormat(pixel_format_name) ==
          false) {  //目前SetPixelFormat只会返回true
        return false;
      }
    } else if (parameter.get_name() == "video_device") {
      video_device_name_ = parameter.value_to_string();
    } else if (parameter.get_name() == "zero_copy") {
      zero_copy_enabled_ = parameter.as_bool();
    } else if (parameter.get_name() == "camera_calibration_file_path") {
      camera_calibration_file_path_ = parameter.value_to_string();
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Invalid parameter name: %s",
                  parameter.get_name().c_str());
    }
  }
  return true;
}

bool HobotUSBCamNode::SetPixelFormat(const std::string &pixel_format_name) {
  if (pixel_format_name == "mjpeg") {
  } else {
    RCLCPP_WARN(this->get_logger(),
                "Invalid pixel format: %s! Currently only support "
                "mjpeg pixel format. Set pixel format to mjepg",
                pixel_format_name.c_str());
  }
  pixel_format_name_ = HobotUSBCam::kPIXEL_FORMAT_MJPEG;
  // Todo : support yuyv uyvy yuvmono10 rgb24 grey
  // if (pixel_format_name == "yuyv") {
  //   pixel_format_name_ = HobotUSBCam::kPIXEL_FORMAT_YUYV;
  // } else if (pixel_format_name == "uyvy") {
  //   pixel_format_name_ = HobotUSBCam::kPIXEL_FORMAT_UYVY;
  // } else if (pixel_format_name == "mjpeg") {
  //   pixel_format_name_ = HobotUSBCam::kPIXEL_FORMAT_MJPEG;
  // } else if (pixel_format_name == "yuvmono10") {
  //   pixel_format_name_ = HobotUSBCam::kPIXEL_FORMAT_YUVMONO10;
  // } else if (pixel_format_name == "rgb24") {
  //   pixel_format_name_ = HobotUSBCam::kPIXEL_FORMAT_RGB24;
  // } else if (pixel_format_name == "grey") {
  //   pixel_format_name_ = HobotUSBCam::kPIXEL_FORMAT_GREY;
  // } else {
  //   RCLCPP_ERROR(this->get_logger(), "Invalid parameter name: %s",
  //   pixel_format_name.c_str());
  //   return false;
  // }
  return true;
}

bool HobotUSBCamNode::SetIOMethod(const std::string &io_method_name) {
  if (io_method_name == "mmap") {
    io_method_name_ = HobotUSBCam::kIO_METHOD_MMAP;
  } else if (io_method_name == "read") {
    io_method_name_ = HobotUSBCam::kIO_METHOD_READ;
  } else if (io_method_name == "userptr") {
    io_method_name_ = HobotUSBCam::kIO_METHOD_USERPTR;
  } else {
    RCLCPP_ERROR(this->get_logger(),
                 "Invalid io_method name: %s! "
                 "mmap、read and userptr are supported! Please check the "
                 "io_method parameter!",
                 io_method_name.c_str());
    return false;
  }
  return true;
}

bool HobotUSBCamNode::SetPublisher() {
  if (zero_copy_enabled_) {
    if (cam_.CheckResolutionFromFormats(image_width_, image_height_)) {
      hbmem_image_pub_1080_ =
          this->create_publisher_hbmem<hbm_img_msgs::msg::HbmMsg1080P>(
              "hbmem_image", 5);
      hbmem_image_pub_540_ = nullptr;
      hbmem_image_pub_480_ = nullptr;
      image_pub_ = nullptr;
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "Invalid publish resolution width:%d height:%d. Please "
                   "modify the parameters image_height or image_width",
                   image_width_,
                   image_height_);
      return false;
    }
  } else {
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image", 5);
  }
  info_pub_ =
      this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 5);
  return true;
}

void HobotUSBCamNode::ReadFrame() {
  while (reading_) {
    HobotUSBCam::CamBuffer cam_buffer;
    cam_.GetFrame(cam_buffer);
    switch (pixel_format_name_) {
      case HobotUSBCam::kPIXEL_FORMAT_MJPEG:
        /* code */
        break;
      case HobotUSBCam::kPIXEL_FORMAT_YUYV:
        break;
      case HobotUSBCam::kPIXEL_FORMAT_UYVY:
        break;
      case HobotUSBCam::kPIXEL_FORMAT_YUVMONO10:
        break;
      case HobotUSBCam::kPIXEL_FORMAT_RGB24:
        break;
      case HobotUSBCam::kPIXEL_FORMAT_GREY:
        break;
      default:
        break;
    }
    if (zero_copy_enabled_) {
      if (image_width_ == 1080) {
        auto loanedMsg = hbmem_image_pub_1080_->borrow_loaned_message();
        if (loanedMsg.is_valid()) {
          auto &message = loanedMsg.get();
          sensor_msgs::msg::CameraInfo camera_calibration_info;
          message.height = 1080;
          message.width = 1920;
          message.step = 1920 * 3;
          memcpy(message.encoding.data(), "jpeg", strlen("jpeg"));
          message.time_stamp.sec = cam_buffer.reserved_buffer.timestamp.tv_sec;
          message.time_stamp.nanosec =
              cam_buffer.reserved_buffer.timestamp.tv_usec * 1000;
          camera_calibration_info.header.stamp = message.time_stamp;
          if (cam_buffer.length < message.step * message.height) {
            memcpy(&message.data[0], cam_buffer.start, cam_buffer.length);
            message.data_size = cam_buffer.length;
          } else {
            memcpy(&message.data[0],
                   cam_buffer.start,
                   message.step * message.height);
            message.data_size = message.step * message.height;
          }
          hbmem_image_pub_1080_->publish(std::move(loanedMsg));
          if (read_cam_calibration_enabled_ != false) {
            camera_calibration_info = camera_calibration_info_;
            camera_calibration_info.header.frame_id = frame_id_;
            info_pub_->publish(camera_calibration_info);
            RCLCPP_INFO(rclcpp::get_logger("hobot_usb_cam"),
                        "publish camera info.\n");
          } else {
            RCLCPP_WARN(rclcpp::get_logger("hobot_usb_cam"),
                        "Unable to publish camera info.\n");
          }
        }
      } else if (image_width_ == 540) {
        auto loanedMsg = hbmem_image_pub_1080_->borrow_loaned_message();
        if (loanedMsg.is_valid()) {
          auto &message = loanedMsg.get();
          sensor_msgs::msg::CameraInfo camera_calibration_info;
          message.height = 540;
          message.width = 960;
          message.step = 960 * 3;
          memcpy(message.encoding.data(), "jpeg", strlen("jpeg"));
          message.time_stamp.sec = cam_buffer.reserved_buffer.timestamp.tv_sec;
          message.time_stamp.nanosec =
              cam_buffer.reserved_buffer.timestamp.tv_usec * 1000;
          camera_calibration_info.header.stamp = message.time_stamp;
          if (cam_buffer.length < message.step * message.height) {
            memcpy(&message.data[0], cam_buffer.start, cam_buffer.length);
            message.data_size = cam_buffer.length;
          } else {
            memcpy(&message.data[0],
                   cam_buffer.start,
                   message.step * message.height);
            message.data_size = message.step * message.height;
          }
          hbmem_image_pub_1080_->publish(std::move(loanedMsg));
          if (read_cam_calibration_enabled_ != false) {
            camera_calibration_info = camera_calibration_info_;
            camera_calibration_info.header.frame_id = frame_id_;
            info_pub_->publish(camera_calibration_info);
            RCLCPP_INFO(rclcpp::get_logger("hobot_usb_cam"),
                        "publish camera info.\n");
          } else {
            RCLCPP_WARN(rclcpp::get_logger("hobot_usb_cam"),
                        "Unable to publish camera info.\n");
          }
        }
      } else {
        auto loanedMsg = hbmem_image_pub_1080_->borrow_loaned_message();
        if (loanedMsg.is_valid()) {
          auto &message = loanedMsg.get();
          sensor_msgs::msg::CameraInfo camera_calibration_info;
          message.height = 480;
          message.width = 640;
          message.step = 640 * 3;
          memcpy(message.encoding.data(), "jpeg", strlen("jpeg"));
          message.time_stamp.sec = cam_buffer.reserved_buffer.timestamp.tv_sec;
          message.time_stamp.nanosec =
              cam_buffer.reserved_buffer.timestamp.tv_usec * 1000;
          camera_calibration_info.header.stamp = message.time_stamp;
          if (cam_buffer.length < message.step * message.height) {
            memcpy(&message.data[0], cam_buffer.start, cam_buffer.length);
            message.data_size = cam_buffer.length;
          } else {
            memcpy(&message.data[0],
                   cam_buffer.start,
                   message.step * message.height);
            message.data_size = message.step * message.height;
          }
          hbmem_image_pub_1080_->publish(std::move(loanedMsg));
          if (read_cam_calibration_enabled_ != false) {
            camera_calibration_info = camera_calibration_info_;
            camera_calibration_info.header.frame_id = frame_id_;
            info_pub_->publish(camera_calibration_info);
            RCLCPP_INFO(rclcpp::get_logger("hobot_usb_cam"),
                        "publish camera info.\n");
          } else {
            RCLCPP_WARN(rclcpp::get_logger("hobot_usb_cam"),
                        "Unable to publish camera info.\n");
          }
        }
      }
    } else {
      auto message = sensor_msgs::msg::Image();
      sensor_msgs::msg::CameraInfo camera_calibration_info;
      size_t size;
      message.header.frame_id = frame_id_;
      message.height = image_height_;
      message.width = image_width_;
      message.step = image_width_ * 3;
      message.header.stamp.sec = cam_buffer.reserved_buffer.timestamp.tv_sec;
      message.header.stamp.nanosec =
          cam_buffer.reserved_buffer.timestamp.tv_usec * 1000;
      camera_calibration_info.header.stamp = message.header.stamp;
      if (pixel_format_name_ == HobotUSBCam::kPIXEL_FORMAT_MJPEG) {
        message.encoding = "jpeg";
        size = cam_buffer.length;
        message.data.resize(size);
        memcpy(&message.data[0], cam_buffer.start, cam_buffer.length);
      } else {
        // Todo support other pixel format
        // message.encoding = "rgb8";
        // size = message.step * message.height;
        // message.data.resize(size);
        // memcpy(&message.data[0], );
      }
      image_pub_->publish(message);
      if (read_cam_calibration_enabled_ != false) {
        camera_calibration_info = camera_calibration_info_;
        camera_calibration_info.header.frame_id = frame_id_;
        info_pub_->publish(camera_calibration_info);
        RCLCPP_INFO(rclcpp::get_logger("hobot_usb_cam"),
                    "publish camera info.\n");
      } else {
        RCLCPP_WARN(rclcpp::get_logger("hobot_usb_cam"),
                    "Unable to publish camera info.\n");
      }
    }
    RCLCPP_INFO(this->get_logger(),
                "publish image %dx%d"
                " encoding:%d size:%d\n",
                image_width_,
                image_height_,
                pixel_format_name_,
                cam_buffer.length);
    cam_.ReleaseFrame(cam_buffer);
  }
}

}  // namespace hobot_usb_cam
