// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// Associated header
//
#include "azure_kinect_ros_driver/k4a_ros_device_params.h"

// System headers
//

// Library headers
//
#include <k4a/k4a.h>

// Project headers
//

// Max exposure times for 50Hz and 60Hz
static constexpr int kMaxExposure50Hz = 130000;
static constexpr int kMaxExposure60Hz = 133330;

K4AROSDeviceParams::K4AROSDeviceParams() : rclcpp::Node("k4a_ros_device_params_node") {}

k4a_result_t K4AROSDeviceParams::GetDeviceConfig(k4a_device_configuration_t* configuration)
{
  configuration->depth_delay_off_color_usec = 0;
  configuration->disable_streaming_indicator = false;

  RCLCPP_INFO_STREAM(this->get_logger(), "Setting wired sync mode: " << wired_sync_mode);
  if (wired_sync_mode == 0)
  {
      configuration->wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
  }
  else if (wired_sync_mode == 1)
  {
      configuration->wired_sync_mode = K4A_WIRED_SYNC_MODE_MASTER;
  }
  else if (wired_sync_mode == 2)
  {
      configuration->wired_sync_mode = K4A_WIRED_SYNC_MODE_SUBORDINATE;
  }
  else
  {
      RCLCPP_ERROR_STREAM(this->get_logger(),"Invalid wired sync mode: " << wired_sync_mode);
      return K4A_RESULT_FAILED;
  }


  RCLCPP_INFO_STREAM(this->get_logger(),"Setting subordinate delay: " << subordinate_delay_off_master_usec);
  configuration->subordinate_delay_off_master_usec = subordinate_delay_off_master_usec;

  if (!color_enabled)
  {
    RCLCPP_INFO_STREAM(this->get_logger(),"Disabling RGB Camera");

    configuration->color_resolution = K4A_COLOR_RESOLUTION_OFF;
  }
  else
  {
    RCLCPP_INFO_STREAM(this->get_logger(),"Setting RGB Camera Format: " << color_format);

    if (color_format == "jpeg")
    {
      configuration->color_format = K4A_IMAGE_FORMAT_COLOR_MJPG;
    }
    else if (color_format == "bgra")
    {
      configuration->color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    }
    else
    {
      RCLCPP_ERROR_STREAM(this->get_logger(),"Invalid RGB Camera Format: " << color_format);
      return K4A_RESULT_FAILED;
    }

    RCLCPP_INFO_STREAM(this->get_logger(),"Setting RGB Camera Resolution: " << color_resolution);

    if (color_resolution == "720P")
    {
      configuration->color_resolution = K4A_COLOR_RESOLUTION_720P;
    }
    else if (color_resolution == "1080P")
    {
      configuration->color_resolution = K4A_COLOR_RESOLUTION_1080P;
    }
    else if (color_resolution == "1440P")
    {
      configuration->color_resolution = K4A_COLOR_RESOLUTION_1440P;
    }
    else if (color_resolution == "1536P")
    {
      configuration->color_resolution = K4A_COLOR_RESOLUTION_1536P;
    }
    else if (color_resolution == "2160P")
    {
      configuration->color_resolution = K4A_COLOR_RESOLUTION_2160P;
    }
    else if (color_resolution == "3072P")
    {
      configuration->color_resolution = K4A_COLOR_RESOLUTION_3072P;
    }
    else
    {
      RCLCPP_ERROR_STREAM(this->get_logger(),"Invalid RGB Camera Resolution: " << color_resolution);
      return K4A_RESULT_FAILED;
    }
  }

  if (!depth_enabled)
  {
    RCLCPP_INFO_STREAM(this->get_logger(),"Disabling Depth Camera");

    configuration->depth_mode = K4A_DEPTH_MODE_OFF;
  }
  else
  {
    RCLCPP_INFO_STREAM(this->get_logger(),"Setting Depth Camera Mode: " << depth_mode);

    if (depth_mode == "NFOV_2X2BINNED")
    {
      configuration->depth_mode = K4A_DEPTH_MODE_NFOV_2X2BINNED;
    }
    else if (depth_mode == "NFOV_UNBINNED")
    {
      configuration->depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    }
    else if (depth_mode == "WFOV_2X2BINNED")
    {
      configuration->depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
    }
    else if (depth_mode == "WFOV_UNBINNED")
    {
      configuration->depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED;
    }
    else if (depth_mode == "PASSIVE_IR")
    {
      configuration->depth_mode = K4A_DEPTH_MODE_PASSIVE_IR;
    }
    else
    {
      RCLCPP_ERROR_STREAM(this->get_logger(),"Invalid Depth Camera Mode: " << depth_mode);
      return K4A_RESULT_FAILED;
    }
  }

  RCLCPP_INFO_STREAM(this->get_logger(),"Setting Camera FPS: " << fps);

  if (fps == 5)
  {
    configuration->camera_fps = K4A_FRAMES_PER_SECOND_5;
  }
  else if (fps == 15)
  {
    configuration->camera_fps = K4A_FRAMES_PER_SECOND_15;
  }
  else if (fps == 30)
  {
    configuration->camera_fps = K4A_FRAMES_PER_SECOND_30;
  }
  else
  {
    RCLCPP_ERROR_STREAM(this->get_logger(),"Invalid Camera FPS: " << fps);
    return K4A_RESULT_FAILED;
  }

  // Ensure that if RGB and depth cameras are enabled, we ask for synchronized frames
  if (depth_enabled && color_enabled)
  {
    configuration->synchronized_images_only = true;
  }
  else
  {
    configuration->synchronized_images_only = false;
  }

  // Ensure that the "point_cloud" option is not used with passive IR mode, since they are incompatible
  if (point_cloud && (configuration->depth_mode == K4A_DEPTH_MODE_PASSIVE_IR))
  {
    RCLCPP_ERROR_STREAM(this->get_logger(),"Incompatible options: cannot generate point cloud if depth camera is using PASSIVE_IR mode.");
    return K4A_RESULT_FAILED;
  }

  // Ensure that point_cloud is enabled if using rgb_point_cloud
  if (rgb_point_cloud && !point_cloud)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(),"Incompatible options: cannot generate RGB point cloud if point_cloud is not enabled.");
    return K4A_RESULT_FAILED;
  }

  // Ensure that color camera is enabled when generating a color point cloud
  if (rgb_point_cloud && !color_enabled)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(),"Incompatible options: cannot generate RGB point cloud if color camera is not enabled.");
    return K4A_RESULT_FAILED;
  }

  // Ensure that color image contains RGB pixels instead of compressed JPEG data.
  if (rgb_point_cloud && color_format == "jpeg")
  {
    RCLCPP_ERROR_STREAM(this->get_logger(),"Incompatible options: cannot generate RGB point cloud if color format is JPEG.");
    return K4A_RESULT_FAILED;
  }

  // Ensure that target IMU rate is feasible
  if (imu_rate_target == 0)
  {
    imu_rate_target = IMU_MAX_RATE;
    RCLCPP_INFO_STREAM(this->get_logger(),"Using default IMU rate. Setting to maximum: " << IMU_MAX_RATE << " Hz.");
  }

  if (imu_rate_target < 0 || imu_rate_target > IMU_MAX_RATE)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(),"Incompatible options: desired IMU rate of " << imu_rate_target << "is not supported.");
    return K4A_RESULT_FAILED;
  }

  int div = IMU_MAX_RATE / imu_rate_target;
  float imu_rate_rounded = IMU_MAX_RATE / div;
  // Since we will throttle the IMU by averaging div samples together, this is the
  // achievable rate when rouded to the nearest whole number div.

  RCLCPP_INFO_STREAM(this->get_logger(),"Setting Target IMU rate to " << imu_rate_rounded << " (desired: " << imu_rate_target << ")");

  return K4A_RESULT_SUCCEEDED;
}
bool K4AROSDeviceParams::SetColorControl(k4a_device_t device, const std::string &param_name, const rclcpp::Parameter &param)
{
  auto error = [this](const std::string &msg) {
    RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
  };

  if (param_name == "exposure_control_mode") {
    exposure_control_mode = param.as_string();
    SetColorControl(device, "exposure_time_absolute", rclcpp::Parameter("exposure_time_absolute", exposure_time_absolute));
    RCLCPP_INFO(this->get_logger(), "Set exposure_control_mode: %s", exposure_control_mode.c_str());
    return true;
  }
  else if (param_name == "white_balance_control_mode") {
    white_balance_control_mode = param.as_string();
    SetColorControl(device, "white_balance", rclcpp::Parameter("white_balance", white_balance));
    RCLCPP_INFO(this->get_logger(), "Set white_balance_control_mode: %s", white_balance_control_mode.c_str());
    return true;
  }
  else if (param_name == "exposure_time_absolute") {
    int value = param.as_int();
    // Range: [500, 133330] (or 130000 for 50Hz)
    bool using_60hz = (powerline_frequency == 2);
    int min_exposure = 500, max_exposure = using_60hz ? 133330 : 130000;
    if (value < min_exposure || value > max_exposure) {
      error("Invalid exposure_time_absolute (" + std::to_string(value) +
            "); allowed range is [" + std::to_string(min_exposure) + "," + std::to_string(max_exposure) + "] us.");
      return false;
    }
    exposure_time_absolute = value;
    k4a_color_control_mode_t mode = (exposure_control_mode == "auto") ? K4A_COLOR_CONTROL_MODE_AUTO : K4A_COLOR_CONTROL_MODE_MANUAL;
    k4a_device_set_color_control(device, K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE, mode, value);
    RCLCPP_INFO(this->get_logger(), "Set exposure_time_absolute: %d (mode=%s)", value, exposure_control_mode.c_str());
    return true;
  }
  else if (param_name == "white_balance") {
    int value = param.as_int();
    // Range: [2500, 12500], divisible by 10 (when in manual)
    if (white_balance_control_mode == "manual") {
      if (value < 2500 || value > 12500 || value % 10 != 0) {
        error("Invalid white_balance (" + std::to_string(value) +
              "); allowed range is [2500,12500], must be divisible by 10.");
        return false;
      }
    }
    white_balance = value;
    k4a_color_control_mode_t mode = (white_balance_control_mode == "auto") ? K4A_COLOR_CONTROL_MODE_AUTO : K4A_COLOR_CONTROL_MODE_MANUAL;
    k4a_device_set_color_control(device, K4A_COLOR_CONTROL_WHITEBALANCE, mode, value);
    RCLCPP_INFO(this->get_logger(), "Set white_balance: %d (mode=%s)", value, white_balance_control_mode.c_str());
    return true;
  }
  else if (param_name == "brightness") {
    int value = param.as_int();
    // Range: [0,255]
    if (value < 0 || value > 255) {
      error("Invalid brightness (" + std::to_string(value) + "); allowed range is [0,255].");
      return false;
    }
    brightness = value;
    k4a_device_set_color_control(device, K4A_COLOR_CONTROL_BRIGHTNESS, K4A_COLOR_CONTROL_MODE_MANUAL, value);
    RCLCPP_INFO(this->get_logger(), "Set brightness: %d", value);
    return true;
  }
  else if (param_name == "contrast") {
    int value = param.as_int();
    // Range: [0,10]
    if (value < 0 || value > 10) {
      error("Invalid contrast (" + std::to_string(value) + "); allowed range is [0,10].");
      return false;
    }
    contrast = value;
    k4a_device_set_color_control(device, K4A_COLOR_CONTROL_CONTRAST, K4A_COLOR_CONTROL_MODE_MANUAL, value);
    RCLCPP_INFO(this->get_logger(), "Set contrast: %d", value);
    return true;
  }
  else if (param_name == "saturation") {
    int value = param.as_int();
    // Range: [0,63]
    if (value < 0 || value > 63) {
      error("Invalid saturation (" + std::to_string(value) + "); allowed range is [0,63].");
      return false;
    }
    saturation = value;
    k4a_device_set_color_control(device, K4A_COLOR_CONTROL_SATURATION, K4A_COLOR_CONTROL_MODE_MANUAL, value);
    RCLCPP_INFO(this->get_logger(), "Set saturation: %d", value);
    return true;
  }
  else if (param_name == "sharpness") {
    int value = param.as_int();
    // Range: [0,4]
    if (value < 0 || value > 4) {
      error("Invalid sharpness (" + std::to_string(value) + "); allowed range is [0,4].");
      return false;
    }
    sharpness = value;
    k4a_device_set_color_control(device, K4A_COLOR_CONTROL_SHARPNESS, K4A_COLOR_CONTROL_MODE_MANUAL, value);
    RCLCPP_INFO(this->get_logger(), "Set sharpness: %d", value);
    return true;
  }
  else if (param_name == "gain") {
    int value = param.as_int();
    // Range: [0,255]
    if (value < 0 || value > 255) {
      error("Invalid gain (" + std::to_string(value) + "); allowed range is [0,255].");
      return false;
    }
    gain = value;
    k4a_device_set_color_control(device, K4A_COLOR_CONTROL_GAIN, K4A_COLOR_CONTROL_MODE_MANUAL, value);
    RCLCPP_INFO(this->get_logger(), "Set gain: %d", value);
    return true;
  }
  else if (param_name == "backlight_compensation") {
    int value = param.as_int();
    // Range: [0,1]
    if (value != 0 && value != 1) {
      error("Invalid backlight_compensation (" + std::to_string(value) + "); allowed values: 0 or 1.");
      return false;
    }
    backlight_compensation = value;
    k4a_device_set_color_control(device, K4A_COLOR_CONTROL_BACKLIGHT_COMPENSATION, K4A_COLOR_CONTROL_MODE_MANUAL, value);
    RCLCPP_INFO(this->get_logger(), "Set backlight_compensation: %d", value);
    return true;
  }
  else if (param_name == "powerline_frequency") {
    int value = param.as_int();
    // Range: [1,2]
    if (value != 1 && value != 2) {
      error("Invalid powerline_frequency (" + std::to_string(value) + "); allowed values: 1 (50Hz), 2 (60Hz).");
      return false;
    }
    powerline_frequency = value;
    k4a_device_set_color_control(device, K4A_COLOR_CONTROL_POWERLINE_FREQUENCY, K4A_COLOR_CONTROL_MODE_MANUAL, value);
    RCLCPP_INFO(this->get_logger(), "Set powerline_frequency: %d", value);
    return true;
  }
  else {
    error("Unknown color control parameter: " + param_name);
    return false;
  }
}

void K4AROSDeviceParams::ApplyColorControls(k4a_device_t device)
{
  SetColorControl(device, "exposure_control_mode", rclcpp::Parameter("exposure_control_mode", exposure_control_mode));
  SetColorControl(device, "exposure_time_absolute", rclcpp::Parameter("exposure_time_absolute", exposure_time_absolute));
  SetColorControl(device, "white_balance_control_mode", rclcpp::Parameter("white_balance_control_mode", white_balance_control_mode));
  SetColorControl(device, "white_balance", rclcpp::Parameter("white_balance", white_balance));
  SetColorControl(device, "brightness", rclcpp::Parameter("brightness", brightness));
  SetColorControl(device, "contrast", rclcpp::Parameter("contrast", contrast));
  SetColorControl(device, "saturation", rclcpp::Parameter("saturation", saturation));
  SetColorControl(device, "sharpness", rclcpp::Parameter("sharpness", sharpness));
  SetColorControl(device, "gain", rclcpp::Parameter("gain", gain));
  SetColorControl(device, "backlight_compensation", rclcpp::Parameter("backlight_compensation", backlight_compensation));
  SetColorControl(device, "powerline_frequency", rclcpp::Parameter("powerline_frequency", powerline_frequency));
}

void K4AROSDeviceParams::Help()
{
#define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val)                                   \
  RCLCPP_INFO(this->get_logger(),"#param_variable - #param_type : param_help_string (#param_default_val)");

  ROS_PARAM_LIST
#undef LIST_ENTRY
}

void K4AROSDeviceParams::Print()
{
#define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val)                                   \
  RCLCPP_INFO_STREAM(this->get_logger(),"" << #param_variable << " - " << #param_type " : " << param_variable);

  ROS_PARAM_LIST
#undef LIST_ENTRY
}
