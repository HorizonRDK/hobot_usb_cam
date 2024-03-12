// Copyright 2014 Robert Bosch, LLC
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Robert Bosch, LLC nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#define CLEAR(x) memset(&(x), 0, sizeof(x))

#include <sys/sysinfo.h>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/distortion_models.hpp"

extern "C" {
#include <linux/videodev2.h>  // Defines V4L2 format constants
#include <malloc.h>  // for memalign
#include <sys/mman.h>  // for mmap
#include <sys/stat.h>  // for stat
#include <unistd.h>  // for getpagesize()
#include <fcntl.h>  // for O_* constants and open()
}

#include <chrono>
#include <ctime>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <fstream>

#include "opencv2/imgproc.hpp"

#include "hobot_usb_cam/hobot_usb_cam.hpp"
#include "hobot_usb_cam/conversions.hpp"
#include "hobot_usb_cam/utils.hpp"


namespace usb_cam
{

using utils::io_method_t;


UsbCam::UsbCam()
: m_device_name(), m_io(io_method_t::IO_METHOD_MMAP), m_fd(-1),
  m_number_of_buffers(4), m_buffers(new usb_cam::utils::buffer[m_number_of_buffers]), m_image(),
  m_avframe(NULL), m_avcodec(NULL), m_avoptions(NULL),
  m_avcodec_context(NULL), m_is_capturing(false), m_framerate(0),
  m_epoch_time_shift_us(usb_cam::utils::get_epoch_time_shift_us()), m_supported_formats()
{}

UsbCam::~UsbCam()
{
  shutdown();
}


/// @brief Fill destination image with source image. If required, convert a given
/// V4L2 Image into another type. Look up possible V4L2 pixe formats in the
/// `linux/videodev2.h` header file.
/// @param src a pointer to a V4L2 source image
/// @param dest a pointer to where the source image should be copied (if required)
/// @param bytes_used number of bytes used by the src buffer
void UsbCam::process_image(const char * src, char * & dest, const int & bytes_used)
{
  // TODO(flynneva): could we skip the copy here somehow?
  // If no conversion required, just copy the image from V4L2 buffer

  if (m_image.pixel_format->requires_conversion() == false) {
    memcpy(dest, src, bytes_used);
    m_image.data_size = bytes_used;
  } else {
    m_image.pixel_format->convert(src, dest, bytes_used);
    m_image.data_size = m_image.size_in_bytes;
  }
}

void UsbCam::read_frame()
{
  struct v4l2_buffer buf;
  unsigned int i;
  int len;
  switch (m_io) {
    case io_method_t::IO_METHOD_READ:
      len = read(m_fd, m_buffers[0].start, m_buffers[0].length);
      if (len == -1) {
        switch (errno) {
          case EAGAIN:
            return;
          default:
            throw std::runtime_error("Unable to read frame");
        }
      }
      return process_image(m_buffers[0].start, m_image.data, len);
    case io_method_t::IO_METHOD_MMAP:
      CLEAR(buf);
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      m_image.v4l2_fmt.type = buf.type;
      buf.memory = V4L2_MEMORY_MMAP;

      // Get current v4l2 pixel format
      if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_G_FMT), &m_image.v4l2_fmt)) {
        switch (errno) {
          case EAGAIN:
            return;
          default:
            throw std::runtime_error("Invalid v4l2 format");
        }
      }
      /// Dequeue buffer with the new image
      if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_DQBUF), &buf)) {
        switch (errno) {
          case EAGAIN:
            return;
          default:
            throw std::runtime_error("Unable to retrieve frame with mmap");
        }
      }
      // Get timestamp from V4L2 image buffer
      m_image.stamp = usb_cam::utils::calc_img_timestamp(buf.timestamp, m_epoch_time_shift_us);

      assert(buf.index < m_number_of_buffers);
      RCLCPP_INFO(rclcpp::get_logger("hobot_usb_cam"),
                  "bufd.bytesused:%d,fmt: %d",buf.bytesused,m_image.v4l2_fmt.fmt.pix.pixelformat);
      process_image(m_buffers[buf.index].start, m_image.data, buf.bytesused);
      /// Requeue buffer so it can be reused
      if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_QBUF), &buf)) {
        throw std::runtime_error("Unable to exchange buffer with the driver");
      }
      return;
    case io_method_t::IO_METHOD_USERPTR:
      CLEAR(buf);

      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_USERPTR;

      if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_DQBUF), &buf)) {
        switch (errno) {
          case EAGAIN:
            return;
          default:
            throw std::runtime_error("Unable to exchange buffer with driver");
        }
      }

      // Get timestamp from V4L2 image buffer
      m_image.stamp = usb_cam::utils::calc_img_timestamp(buf.timestamp, m_epoch_time_shift_us);

      for (i = 0; i < m_number_of_buffers; ++i) {
        if (buf.m.userptr == reinterpret_cast<uint64_t>(m_buffers[i].start) && \
          buf.length == m_buffers[i].length)
        {
          return;
        }
      }

      assert(i < m_number_of_buffers);
      process_image(reinterpret_cast<const char *>(buf.m.userptr), m_image.data, buf.bytesused);
      if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_QBUF), &buf)) {
        throw std::runtime_error("Unable to exchange buffer with driver");
      }
      return;
    case io_method_t::IO_METHOD_UNKNOWN:
      throw std::invalid_argument("IO method unknown");
  }
}

void UsbCam::stop_capturing()
{
  if (!m_is_capturing) {return;}

  m_is_capturing = false;
  enum v4l2_buf_type type;

  switch (m_io) {
    case io_method_t::IO_METHOD_READ:
      /* Nothing to do. */
      return;
    case io_method_t::IO_METHOD_MMAP:
    case io_method_t::IO_METHOD_USERPTR:
      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      if (-1 == usb_cam::utils::xioctl(m_fd, VIDIOC_STREAMOFF, &type)) {
        // Set capturing variable to true again, since stream was not stopped successfully
        m_is_capturing = true;
        throw std::runtime_error("Unable to stop capturing stream");
      }
      return;
    case io_method_t::IO_METHOD_UNKNOWN:
      throw std::invalid_argument("IO method unknown");
  }
}

void UsbCam::start_capturing()
{
  if (m_is_capturing) {return;}

  unsigned int i;
  enum v4l2_buf_type type;

  switch (m_io) {
    case io_method_t::IO_METHOD_READ:
      /* Nothing to do. */
      break;
    case io_method_t::IO_METHOD_MMAP:
      // Queue the buffers
      for (i = 0; i < m_number_of_buffers; ++i) {
        struct v4l2_buffer buf;
        CLEAR(buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_QBUF), &buf)) {
          throw std::runtime_error("Unable to queue image buffer");
        }
      }

      // Start the stream
      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      if (-1 == usb_cam::utils::xioctl(m_fd, VIDIOC_STREAMON, &type)) {
        throw std::runtime_error("Unable to start stream");
      }
      break;
    case io_method_t::IO_METHOD_USERPTR:
      for (i = 0; i < m_number_of_buffers; ++i) {
        struct v4l2_buffer buf;

        CLEAR(buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_USERPTR;
        buf.index = i;
        buf.m.userptr = reinterpret_cast<uint64_t>(m_buffers[i].start);
        buf.length = m_buffers[i].length;

        if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_QBUF), &buf)) {
          throw std::runtime_error("Unable to configure stream");
        }
      }

      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

      if (-1 == usb_cam::utils::xioctl(m_fd, VIDIOC_STREAMON, &type)) {
        throw std::runtime_error("Unable to start stream");
      }
      break;
    case io_method_t::IO_METHOD_UNKNOWN:
      throw std::invalid_argument("IO method unknown");
  }
  m_is_capturing = true;
}

void UsbCam::uninit_device()
{
  m_buffers.reset();
}

void UsbCam::init_read()
{
  if (!m_buffers) {
    throw std::overflow_error("Out of memory");
  }

  m_buffers[0].length = m_image.size_in_bytes;

  if (!m_buffers[0].start) {
    throw std::overflow_error("Out of memory");
  }
}

void UsbCam::init_mmap()
{
  struct v4l2_requestbuffers req;

  CLEAR(req);

  req.count = m_number_of_buffers;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;

  if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_REQBUFS), &req)) {
    if (EINVAL == errno) {
      throw std::runtime_error("Device does not support memory mapping");
    } else {
      throw std::runtime_error("Unable to initialize memory mapping");
    }
  }

  if (req.count < m_number_of_buffers) {
    throw std::overflow_error("Insufficient buffer memory on device");
  }

  if (!m_buffers) {
    throw std::overflow_error("Out of memory");
  }

  for (uint32_t current_buffer = 0; current_buffer < req.count; ++current_buffer) {
    struct v4l2_buffer buf;

    CLEAR(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = current_buffer;

    if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_QUERYBUF), &buf)) {
      throw std::runtime_error("Unable to query status of buffer");
    }

    m_buffers[current_buffer].length = buf.length;
    m_buffers[current_buffer].start =
      reinterpret_cast<char *>(mmap(
        NULL /* start anywhere */, buf.length, PROT_READ | PROT_WRITE /* required */,
        MAP_SHARED /* recommended */, m_fd, buf.m.offset));

    if (MAP_FAILED == m_buffers[current_buffer].start) {
      throw std::runtime_error("Unable to allocate memory for image buffers");
    }
  }
}

void UsbCam::init_userp()
{
  struct v4l2_requestbuffers req;
  unsigned int page_size;

  page_size = getpagesize();
  auto buffer_size = (m_image.size_in_bytes + page_size - 1) & ~(page_size - 1);

  CLEAR(req);

  req.count = m_number_of_buffers;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_USERPTR;

  if (-1 == usb_cam::utils::xioctl(m_fd, VIDIOC_REQBUFS, &req)) {
    if (EINVAL == errno) {
      throw std::invalid_argument("Device does not support user pointer i/o");
    } else {
      throw std::invalid_argument("Unable to initialize memory mapping");
    }
  }

  if (!m_buffers) {
    throw std::overflow_error("Out of memory");
  }

  for (uint32_t current_buffer = 0; current_buffer < req.count; ++current_buffer) {
    m_buffers[current_buffer].length = buffer_size;
    m_buffers[current_buffer].start =
      reinterpret_cast<char *>(memalign(/* boundary */ page_size, buffer_size));

    if (!m_buffers[current_buffer].start) {
      throw std::overflow_error("Out of memory");
    }
  }
}

void UsbCam::init_device()
{
  struct v4l2_capability cap;
  struct v4l2_cropcap cropcap;
  struct v4l2_crop crop;

  if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_QUERYCAP), &cap)) {
    if (EINVAL == errno) {
      throw std::invalid_argument("Device is not a V4L2 device");
    } else {
      throw std::invalid_argument("Unable to query device capabilities");
    }
  }

  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    throw std::invalid_argument("Device is not a video capture device");
  }

  switch (m_io) {
    case io_method_t::IO_METHOD_READ:
      if (!(cap.capabilities & V4L2_CAP_READWRITE)) {
        throw std::invalid_argument("Device does not support read i/o");
      }
      break;
    case io_method_t::IO_METHOD_MMAP:
    case io_method_t::IO_METHOD_USERPTR:
      if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
        throw std::invalid_argument("Device does not support streaming i/o");
      }
      break;
    case io_method_t::IO_METHOD_UNKNOWN:
      throw std::invalid_argument("Unsupported IO method specified");
  }

  /* Select video input, video standard and tune here. */

  CLEAR(cropcap);

  cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  if (0 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_CROPCAP), &cropcap)) {
    crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    crop.c = cropcap.defrect; /* reset to default */

    if (-1 == usb_cam::utils::xioctl(m_fd, VIDIOC_S_CROP, &crop)) {
      switch (errno) {
        case EINVAL:
          /* Cropping not supported. */
          break;
        default:
          /* Errors ignored. */
          break;
      }
    }
  } else {
    /* Errors ignored. */
  }

  RCLCPP_WARN(rclcpp::get_logger("hobot_usb_cam"), "This devices supproted formats:");
  for (auto fmt : supported_formats()) {
    RCLCPP_WARN(
      rclcpp::get_logger("hobot_usb_cam"),
      "\t%s: %d x %d (%d Hz)",
      fmt.format.description,
      fmt.v4l2_fmt.width,
      fmt.v4l2_fmt.height,
      fmt.v4l2_fmt.discrete.denominator / fmt.v4l2_fmt.discrete.numerator);
  }

  m_image.v4l2_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  m_image.v4l2_fmt.fmt.pix.width = m_image.width;
  m_image.v4l2_fmt.fmt.pix.height = m_image.height;
  m_image.v4l2_fmt.fmt.pix.pixelformat = m_image.pixel_format->v4l2();
  m_image.v4l2_fmt.fmt.pix.field = V4L2_FIELD_ANY;
  // Set v4l2 capture format
  // Note VIDIOC_S_FMT may change width and height
  if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_S_FMT), &m_image.v4l2_fmt)) {
    throw strerror(errno);
  }
  if (m_image.v4l2_fmt.fmt.pix.pixelformat != m_image.pixel_format->v4l2()) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_usb_cam"),
                  "this device isn't support pixel format %s", m_image.pixel_format->name().c_str());
    throw strerror(errno);
  }

  struct v4l2_streamparm stream_params;
  memset(&stream_params, 0, sizeof(stream_params));
  stream_params.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_G_PARM), &stream_params) < 0) {
    throw strerror(errno);
  }

  if (!stream_params.parm.capture.capability && V4L2_CAP_TIMEPERFRAME) {
    throw "V4L2_CAP_TIMEPERFRAME not supported";
  }

  // TODO(lucasw) need to get list of valid numerator/denominator pairs
  // and match closest to what user put in.
  stream_params.parm.capture.timeperframe.numerator = 1;
  stream_params.parm.capture.timeperframe.denominator = m_framerate;
  if (usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_S_PARM), &stream_params) < 0) {
    throw std::invalid_argument("Couldn't set camera framerate");
  }

  switch (m_io) {
    case io_method_t::IO_METHOD_READ:
      init_read();
      break;
    case io_method_t::IO_METHOD_MMAP:
      init_mmap();
      break;
    case io_method_t::IO_METHOD_USERPTR:
      init_userp();
      break;
    case io_method_t::IO_METHOD_UNKNOWN:
      // TODO(flynneva): log something
      break;
  }
}

void UsbCam::close_device()
{
  // Device is already closed
  if (m_fd == -1) {return;}

  if (-1 == close(m_fd)) {
    throw strerror(errno);
  }

  m_fd = -1;
}

void UsbCam::open_device()
{
  struct stat st;

  if (-1 == stat(m_device_name.c_str(), &st)) {
    throw strerror(errno);
  }

  if (!S_ISCHR(st.st_mode)) {
    throw strerror(errno);
  }

  m_fd = open(m_device_name.c_str(), O_RDWR /* required */ | O_NONBLOCK, 0);

  if (-1 == m_fd) {
    throw strerror(errno);
  }
}

void UsbCam::configure(
  parameters_t & parameters, const io_method_t & io_method)
{
  m_device_name = parameters.device_name;
  m_io = io_method;
  m_image.width = static_cast<int>(parameters.image_width);
  m_image.height = static_cast<int>(parameters.image_height);
  m_image.set_number_of_pixels();

  // Do this before calling set_bytes_per_line and set_size_in_bytes
  m_image.pixel_format = set_pixel_format(parameters);
  m_image.set_bytes_per_line();
  m_image.set_size_in_bytes();
  m_framerate = parameters.framerate;
  bool init_success = true;

  if (!configure_exe()) {
    // 遍历/dev/下的video设备
    // 使用 find 命令查找/dev/下的video设备
    std::string command = "find /dev -name \"video[0-9]*\" | sort";
    FILE* fp = popen(command.c_str(), "r");
    if (fp) {
      size_t video_dev_len = 20;
      char* video_dev = new char[video_dev_len];
      int ret_len = 0;
      while ((ret_len = getline(&video_dev, &video_dev_len, fp)) > 0) {
        init_success = false;
        close_device();
        m_device_name = std::string(video_dev, ret_len - 1);
        RCLCPP_WARN_STREAM(rclcpp::get_logger("hobot_usb_cam"),
                    "Try to open device [" << m_device_name << "]");
        memset(video_dev, '0', video_dev_len);
        if (configure_exe()) {
          RCLCPP_WARN_STREAM(rclcpp::get_logger("hobot_usb_cam"),
                      "Open & Init device " << m_device_name << " success.");
          init_success = true;
          break;
        }
      }
      delete []video_dev;
    }
  }
  if (init_success == false) {
    throw strerror(errno);
  }
}

bool UsbCam::configure_exe()
{
  try {
    // Open device file descriptor before anything else
    open_device();
    init_device();
  } catch (...) {
    return false;
  }
  return true;

}

void UsbCam::start()
{
  start_capturing();
}

void UsbCam::shutdown()
{
  stop_capturing();
  uninit_device();
  close_device();
}

/// @brief Grab new image from V4L2 device, return pointer to image
/// @return pointer to image data
char * UsbCam::get_image()
{
  if ((m_image.width == 0) || (m_image.height == 0)) {
    return nullptr;
  }
  if (m_image.data == nullptr) {
    m_image.data = (char*)malloc(m_image.size_in_bytes);
  }
  // grab the image
  grab_image();
  return m_image.data;
}

/// @brief Overload get_image so users can pass in an image pointer to fill
/// @param destination destination to fill in with image
void UsbCam::get_image(char * destination)
{
  if ((m_image.width == 0) || (m_image.height == 0)) {
    return;
  }
  // Set the destination pointer to be filled
  m_image.data = destination;
  // grab the image
  grab_image();
}

std::vector<capture_format_t> UsbCam::get_supported_formats()
{
  m_supported_formats.clear();
  struct v4l2_fmtdesc * current_format = new v4l2_fmtdesc();
  struct v4l2_frmsizeenum * current_size = new v4l2_frmsizeenum();
  struct v4l2_frmivalenum * current_interval = new v4l2_frmivalenum();

  current_format->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  current_format->index = 0;
  for (current_format->index = 0;
    usb_cam::utils::xioctl(
      m_fd, VIDIOC_ENUM_FMT, current_format) == 0;
    ++current_format->index)
  {
    current_size->index = 0;
    current_size->pixel_format = current_format->pixelformat;

    for (current_size->index = 0;
      usb_cam::utils::xioctl(
        m_fd, VIDIOC_ENUM_FRAMESIZES, current_size) == 0;
      ++current_size->index)
    {
      current_interval->index = 0;
      current_interval->pixel_format = current_size->pixel_format;
      current_interval->width = current_size->discrete.width;
      current_interval->height = current_size->discrete.height;
      for (current_interval->index = 0;
        usb_cam::utils::xioctl(
          m_fd, VIDIOC_ENUM_FRAMEINTERVALS, current_interval) == 0;
        ++current_interval->index)
      {
        if (current_interval->type == V4L2_FRMIVAL_TYPE_DISCRETE) {
          capture_format_t capture_format;
          capture_format.format = *current_format;
          capture_format.v4l2_fmt = *current_interval;
          m_supported_formats.push_back(capture_format);
        }
      }  // interval loop
    }  // size loop
  }  // fmt loop

  delete (current_format);
  delete (current_size);
  delete (current_interval);

  return m_supported_formats;
}

void UsbCam::grab_image()
{
  fd_set fds;
  struct timeval tv;
  int r;

  FD_ZERO(&fds);
  FD_SET(m_fd, &fds);

  /* Timeout. */
  tv.tv_sec = 5;
  tv.tv_usec = 0;

  r = select(m_fd + 1, &fds, NULL, NULL, &tv);

  if (-1 == r) {
    if (EINTR == errno) {
      // interruped (e.g. maybe Ctrl + c) so don't throw anything
      return;
    }

    std::cerr << "Something went wrong, exiting..." << errno << std::endl;
    throw errno;
  }

  if (0 == r) {
    std::cerr << "Select timeout, exiting..." << std::endl;
    throw "select timeout";
  }

  read_frame();
}

// enables/disables auto focus
bool UsbCam::set_auto_focus(int value)
{
  struct v4l2_queryctrl queryctrl;
  struct v4l2_ext_control control;

  memset(&queryctrl, 0, sizeof(queryctrl));
  queryctrl.id = V4L2_CID_FOCUS_AUTO;

  if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_QUERYCTRL), &queryctrl)) {
    if (errno != EINVAL) {
      std::cerr << "VIDIOC_QUERYCTRL" << std::endl;
      return false;
    } else {
      std::cerr << "V4L2_CID_FOCUS_AUTO is not supported" << std::endl;
      return false;
    }
  } else if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
    std::cerr << "V4L2_CID_FOCUS_AUTO is not supported" << std::endl;
    return false;
  } else {
    memset(&control, 0, sizeof(control));
    control.id = V4L2_CID_FOCUS_AUTO;
    control.value = value;

    if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_S_CTRL), &control)) {
      std::cerr << "VIDIOC_S_CTRL" << std::endl;
      return false;
    }
  }
  return true;
}

/**
* Set video device parameter via call to v4l-utils.
*
* @param param The name of the parameter to set
* @param param The value to assign
*/
bool UsbCam::set_v4l_parameter(const std::string & param, int value)
{
  char buf[33];
  snprintf(buf, sizeof(buf), "%i", value);
  return set_v4l_parameter(param, buf);
}

/**
* Set video device parameter via call to v4l-utils.
*
* @param param The name of the parameter to set
* @param param The value to assign
*/
bool UsbCam::set_v4l_parameter(const std::string & param, const std::string & value)
{
  int retcode = 0;
  // build the command
  std::stringstream ss;
  ss << "v4l2-ctl --device=" << m_device_name << " -c " << param << "=" << value << " 2>&1";
  std::string cmd = ss.str();

  // capture the output
  std::string output;
  const int kBufferSize = 256;
  char buffer[kBufferSize];
  FILE * stream = popen(cmd.c_str(), "r");
  if (stream) {
    while (!feof(stream)) {
      if (fgets(buffer, kBufferSize, stream) != NULL) {
        output.append(buffer);
      }
    }
    pclose(stream);
    // any output should be an error
    if (output.length() > 0) {
      std::cout << output.c_str() << std::endl;
      retcode = 1;
    }
  } else {
    std::cerr << "usb_cam_node could not run '" << cmd.c_str() << "'" << std::endl;
    retcode = 1;
  }
  return retcode;
}


bool UsbCam::ReadCalibrationFile(
    sensor_msgs::msg::CameraInfo &cam_calibration_info,
    const std::string &file_path) {
  try {
    std::string camera_name;
    std::ifstream fin(file_path.c_str());
    if (!fin) {
      RCLCPP_WARN(
          rclcpp::get_logger("hobot_usb_cam"),
          "Camera calibration file: [%s] does not exist!"
          "\nIf you need calibration msg, please make sure the calibration file path is correct and the calibration file exists!",
          file_path.c_str());
      return false;
    }
    YAML::Node calibration_doc = YAML::Load(fin);
    if (calibration_doc["camera_name"]) {
      camera_name = calibration_doc["camera_name"].as<std::string>();
    } else {
      camera_name = "unknown";
    }
    cam_calibration_info.width = calibration_doc["image_width"].as<int>();
    cam_calibration_info.height = calibration_doc["image_height"].as<int>();

    const YAML::Node &camera_matrix = calibration_doc["camera_matrix"];
    const YAML::Node &camera_matrix_data = camera_matrix["data"];
    for (int i = 0; i < 9; i++) {
      cam_calibration_info.k[i] = camera_matrix_data[i].as<double>();
    }
    const YAML::Node &rectification_matrix =
        calibration_doc["rectification_matrix"];
    const YAML::Node &rectification_matrix_data = rectification_matrix["data"];
    for (int i = 0; i < 9; i++) {
      cam_calibration_info.r[i] = rectification_matrix_data[i].as<double>();
    }
    const YAML::Node &projection_matrix = calibration_doc["projection_matrix"];
    const YAML::Node &projection_matrix_data = projection_matrix["data"];
    for (int i = 0; i < 12; i++) {
      cam_calibration_info.p[i] = projection_matrix_data[i].as<double>();
    }

    if (calibration_doc["distortion_model"]) {
      cam_calibration_info.distortion_model =
          calibration_doc["distortion_model"].as<std::string>();
    } else {
      cam_calibration_info.distortion_model =
          sensor_msgs::distortion_models::PLUMB_BOB;
      RCLCPP_INFO(rclcpp::get_logger("hobot_usb_cam"),
                  "Camera calibration file did not specify distortion model, "
                  "assuming plumb bob");
    }
    const YAML::Node &distortion_coefficients =
        calibration_doc["distortion_coefficients"];
    int d_rows, d_cols;
    d_rows = distortion_coefficients["rows"].as<int>();
    d_cols = distortion_coefficients["cols"].as<int>();
    const YAML::Node &distortion_coefficients_data =
        distortion_coefficients["data"];
    cam_calibration_info.d.resize(d_rows * d_cols);
    for (int i = 0; i < d_rows * d_cols; ++i) {
      cam_calibration_info.d[i] = distortion_coefficients_data[i].as<double>();
    }
    RCLCPP_INFO(rclcpp::get_logger("hobot_usb_cam"),
                "[get_cam_calibration]->parse calibration file successfully");
    return true;
  } catch (YAML::Exception &e) {
    RCLCPP_WARN(rclcpp::get_logger("hobot_usb_cam"),
                "Unable to parse camera calibration file normally:%s",
                e.what());
    return false;
  }
}

}  // namespace usb_cam
