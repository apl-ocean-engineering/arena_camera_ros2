/******************************************************************************
 * Copyright (C) 2023 University of Washington
 *
 * based on the arena_camera_ros driver released under the BSD License:
 * Copyright (C) 2021, Lucidvision Labs
 * Copyright (C) 2016, Magazino GmbH. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Magazino GmbH nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

// ROS2
#include <rcpputils/asserts.hpp>

// Arena
#include <ArenaApi.h>
#include <GenApi/GenApi.h>
#include <GenApiCustom.h>

// Arena node
#include "arena_camera/arena_camera_nodes.h"
#include "arena_camera/encoding_conversions.h"

// using diagnostic_msgs::DiagnosticStatus;

namespace arena_camera {

ArenaCameraBaseNode::ArenaCameraBaseNode(const std::string &node_name,
                                         const rclcpp::NodeOptions &options)
    : Node(node_name, options),
      pSystem_(nullptr),
      pDevice_(nullptr),
      pNodeMap_(nullptr),
      is_streaming_(false) {
  diagnostics_updater_ = std::make_shared<diagnostic_updater::Updater>(this, 2);

  // ~~ Diagnostic updater functions from ros1 node which haven't been
  // re-implemented...
  //   diagnostics_updater_.setHardwareID("none");
  //   diagnostics_updater_.add("camera_availability", this,
  //                            &ArenaCameraBaseNode::create_diagnostics);
  //   diagnostics_updater_.add(
  //       "intrinsic_calibration", this,
  //       &ArenaCameraBaseNode::create_camera_info_diagnostics);

  img_raw_pub_ = image_transport::create_camera_publisher(this, "image_raw");
  camera_info_manager_ =
      std::make_shared<camera_info_manager::CameraInfoManager>(this,
                                                               "camera_info");

  metadata_pub_ = this->create_publisher<imaging_msgs::msg::ImagingMetadata>(
      "imaging_metadata", 1);

  param_listener_ = std::make_shared<arena_camera::ParamListener>(
      this->get_node_parameters_interface());
  params_ = param_listener_->get_params();

  // Open the Arena SDK
  pSystem_ = Arena::OpenSystem();
  pSystem_->UpdateDevices(100);
  if (pSystem_->GetDevices().size() == 0) {
    RCLCPP_FATAL(this->get_logger(), "Did not detect any cameras!!");
    return;
  }

  if (!params_.device_user_id.empty()) {
    if (!registerCameraByUserId(params_.device_user_id)) {
      RCLCPP_FATAL_STREAM(this->get_logger(),
                          "Unable to find a camera with DeviceUserId\""
                              << params_.device_user_id << "\"");
      return;
    }
  } else if (!params_.serial_number.empty()) {
    if (!registerCameraBySerialNumber(params_.serial_number)) {
      RCLCPP_FATAL_STREAM(this->get_logger(),
                          "Unable to find a camera with Serial Number"
                              << params_.serial_number);
      return;
    }
  } else {
    RCLCPP_WARN(this->get_logger(),
                "Neither device_user_id nor serial_number supplied, attempting "
                "to autodetect");
    if (!registerCameraByAuto()) {
      RCLCPP_FATAL(this->get_logger(),
                   "Unable to find any cameras to register");
      return;
    }
  }

  // Validate that the camera is from Lucid
  //(otherwise the Arena SDK will segfault)
  const auto device_vendor_name = Arena::GetNodeValue<GenICam::gcstring>(
      pDevice_->GetNodeMap(), "DeviceVendorName");
  if (device_vendor_name != "Lucid Vision Labs") {
    RCLCPP_FATAL_STREAM(
        this->get_logger(),
        "Hm, this doesn't appear to be a Lucid Vision camera, got vendor name: "
            << device_vendor_name);
  }

  // ~~ Now define all of the camera parameters ~~
  if (!configureCamera()) {
    RCLCPP_FATAL(this->get_logger(), "Unable to configure camera");
  }

  // ~~ Callback to check parameters ~~
  // Don't like needing to poll these
  parameter_check_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&ArenaCameraBaseNode::checkParametersCb, this));
}

ArenaCameraBaseNode::~ArenaCameraBaseNode() {
  if (pDevice_ != nullptr) {
    pSystem_->DestroyDevice(pDevice_);
  }

  if (pSystem_ != nullptr) {
    Arena::CloseSystem(pSystem_);
  }
}

bool ArenaCameraBaseNode::registerCameraByUserId(
    const std::string &device_user_id_to_open) {
  rcpputils::assert_true(pSystem_);
  std::vector<Arena::DeviceInfo> deviceInfos = pSystem_->GetDevices();
  rcpputils::assert_true(deviceInfos.size() > 0);

  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Connecting to camera with DeviceUserId \""
                         << device_user_id_to_open << "\"");

  std::vector<Arena::DeviceInfo>::iterator it;

  for (auto &dev : deviceInfos) {
    const std::string device_user_id(dev.UserDefinedName());
    // Must be an exact match
    if (0 == device_user_id_to_open.compare(device_user_id)) {
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Found the desired camera with DeviceUserID \""
                             << device_user_id_to_open << "\"");

      pDevice_ = pSystem_->CreateDevice(dev);
      return true;
    }
  }

  RCLCPP_ERROR_STREAM(
      this->get_logger(),
      "Couldn't find the camera that matches the "
          << "given DeviceUserID: \"" << device_user_id_to_open << "\"! "
          << "Either the ID is wrong or the cam is not yet connected");
  return false;
}

bool ArenaCameraBaseNode::registerCameraBySerialNumber(
    const std::string &serial_number) {
  rcpputils::assert_true(pSystem_);
  std::vector<Arena::DeviceInfo> deviceInfos = pSystem_->GetDevices();
  rcpputils::assert_true(deviceInfos.size() > 0);

  RCLCPP_INFO_STREAM(
      this->get_logger(),
      "Connecting to camera with Serial Number " << serial_number);

  for (auto &dev : deviceInfos) {
    if (0 == serial_number.compare(dev.SerialNumber())) {
      RCLCPP_INFO_STREAM(
          this->get_logger(),
          "Found the desired camera with Serial Number " << serial_number);

      pDevice_ = pSystem_->CreateDevice(dev);
      return true;
    }
  }

  RCLCPP_ERROR_STREAM(
      this->get_logger(),
      "Couldn't find the camera that matches the "
          << "given Serial Number: " << serial_number << "! "
          << "Either the ID is wrong or the cam is not yet connected");
  return false;
}

bool ArenaCameraBaseNode::registerCameraByAuto() {
  rcpputils::assert_true(pSystem_);
  std::vector<Arena::DeviceInfo> deviceInfos = pSystem_->GetDevices();
  rcpputils::assert_true(deviceInfos.size() > 0);

  int i = 0;
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Found " << deviceInfos.size() << " cameras:");
  for (auto &dev : deviceInfos) {
    RCLCPP_INFO_STREAM(this->get_logger(),
                       i++ << ":  s/n: " << dev.SerialNumber()
                           << "  User id: " << dev.UserDefinedName()
                           << " vendor: " << dev.VendorName());
  }

  for (auto &dev : deviceInfos) {
    if (dev.VendorName() == "Lucid Vision Labs") {
      pDevice_ = pSystem_->CreateDevice(dev);
      RCLCPP_INFO_STREAM(
          this->get_logger(),
          "Connecting to first autodetected Lucid Vision camera: s/n "
              << dev.SerialNumber() << " User ID: " << dev.UserDefinedName());
      return true;
    }
  }

  return false;
}

bool ArenaCameraBaseNode::configureCamera() {
  auto pNodeMap = pDevice_->GetNodeMap();

  try {
    RCLCPP_INFO_STREAM(
        this->get_logger(),
        "   Device model: " << Arena::GetNodeValue<GenICam::gcstring>(
            pDevice_->GetNodeMap(), "DeviceModelName"));
    RCLCPP_INFO_STREAM(
        this->get_logger(),
        "Device firmware: " << Arena::GetNodeValue<GenICam::gcstring>(
            pDevice_->GetNodeMap(), "DeviceFirmwareVersion"));

    if (Arena::GetNodeValue<GenICam::gcstring>(
            pDevice_->GetNodeMap(), "DeviceTLType") == "GigEVision") {
      RCLCPP_INFO_STREAM(
          this->get_logger(),
          "GigE device, performing GigE specific configuration:");

      // Set Jumbo frames (this is only relevant for GigE cameras.)
      auto pPacketSize = pNodeMap->GetNode("DeviceStreamChannelPacketSize");
      if (GenApi::IsWritable(pPacketSize)) {
        RCLCPP_INFO_STREAM(this->get_logger(),
                           " -> Setting MTU to " << params_.gige.mtu_size);
        Arena::SetNodeValue<int64_t>(pNodeMap, "DeviceStreamChannelPacketSize",
                                     params_.gige.mtu_size);
      } else {
        RCLCPP_INFO_STREAM(this->get_logger(),
                           " -> Camera MTU is not writeable");
      }
    }

    auto payloadSize = Arena::GetNodeValue<int64_t>(pNodeMap, "PayloadSize");
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Expected payload size: " << payloadSize);

    // enable stream auto negotiate packet size
    RCLCPP_DEBUG_STREAM(this->get_logger(),
                        "Enabling auto-negotiation of packet size");
    Arena::SetNodeValue<bool>(pDevice_->GetTLStreamNodeMap(),
                              "StreamAutoNegotiatePacketSize", true);

    // enable stream packet resend
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Enabling packet resend");
    Arena::SetNodeValue<bool>(pDevice_->GetTLStreamNodeMap(),
                              "StreamPacketResendEnable", true);

    setImageEncoding(params_.image_encoding);
    if (encoding_conversions::isHDR(params_.image_encoding)) {
      hdr_metadata_pub_ =
          this->create_publisher<imaging_msgs::msg::HdrImagingMetadata>(
              "hdr_imaging_metadata", 1);
    }

    //  Initial setting of the CameraInfo-msg, assuming no calibration given
    camera_info_manager::CameraInfo initial_cam_info;
    camera_info_manager_->setCameraInfo(initial_cam_info);

    if (camera_info_manager_->validateURL(params_.camera_info_url) &&
        camera_info_manager_->loadCameraInfo(params_.camera_info_url)) {
      // This is weird but the above boolean expression is a lot cleaner.
      ;
    } else {
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "Unable to validate camera info URL \""
                              << params_.camera_info_url << "\"");
    }

    // ~~~ Start unimplemented features

    //     //
    //     // TRIGGER MODE
    //     //
    //     GenApi::CStringPtr pTriggerMode = pNodeMap->GetNode("TriggerMode");
    //     if (GenApi::IsWritable(pTriggerMode)) {
    //       Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "TriggerMode",
    //       "On"); Arena::SetNodeValue<GenICam::gcstring>(pNodeMap,
    //       "TriggerSource",
    //                                              "Software");
    //     }

    //     // LUT
    //     Node_INFO_STREAM(
    //         (arena_camera_parameter_set_.enable_lut_ ? "Enabling" :
    //         "Disabling")
    //         << " camera LUT");
    //     enableLUT(arena_camera_parameter_set_.enable_lut_);

    //     if (arena_camera_parameter_set_.binning_x_given_) {
    //       size_t reached_binning_x;
    //       if (setBinningX(arena_camera_parameter_set_.binning_x_,
    //                       reached_binning_x)) {
    //         Node_INFO_STREAM("Setting horizontal binning_x to "
    //                             << arena_camera_parameter_set_.binning_x_);
    //         Node_WARN_STREAM(
    //             "The image width of the camera_info-msg will "
    //             << "be adapted, so that the binning_x value in this msg
    //             remains 1");
    //       }
    //     }

    //     if (arena_camera_parameter_set_.binning_y_given_) {
    //       size_t reached_binning_y;
    //       if (setBinningY(arena_camera_parameter_set_.binning_y_,
    //                       reached_binning_y)) {
    //         Node_INFO_STREAM("Setting vertical binning_y to "
    //                             << arena_camera_parameter_set_.binning_y_);
    //         Node_WARN_STREAM(
    //             "The image height of the camera_info-msg will "
    //             << "be adapted, so that the binning_y value in this msg
    //             remains 1");
    //       }
    //     }

    // ~~~ End unimplemented features

    Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetTLStreamNodeMap(),
                                           "StreamBufferHandlingMode",
                                           "NewestOnly");

    // bool isTriggerArmed = false;
    // if (GenApi::IsWritable(pTriggerMode)) {
    //   do {
    //     isTriggerArmed = Arena::GetNodeValue<bool>(pNodeMap, "TriggerArmed");
    //   } while (isTriggerArmed == false);
    //   // Arena::ExecuteNode(pNodeMap, "TriggerSoftware");
    // }

    // ~~ Initial configuration of camera

    setFrameRate(params_.frame_rate);
    setGain(params_);
    setExposure(params_);

    setGamma(params_.gamma);

    setTargetBrightness(params_.target_brightness);

  } catch (GenICam::GenericException &e) {
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "Error while configuring camera: \r\n"
                            << e.GetDescription());
    return false;
  }

  return true;
}

void ArenaCameraBaseNode::startStreaming() {
  if (!is_streaming_) {
    pDevice_->StartStream();
    is_streaming_ = true;
  }
}

void ArenaCameraBaseNode::stopStreaming() {
  if (is_streaming_) {
    pDevice_->StopStream();
    is_streaming_ = false;
  }
}

//========================================================================
//
// Image encoding
//

std::string ArenaCameraBaseNode::currentROSEncoding() {
  std::string gen_api_encoding(Arena::GetNodeValue<GenICam::gcstring>(
      pDevice_->GetNodeMap(), "PixelFormat"));
  std::string ros_encoding;

  if (!encoding_conversions::genAPI2Ros(gen_api_encoding, ros_encoding)) {
    std::stringstream ss;
    ss << "No ROS equivalent to GenApi encoding '" << gen_api_encoding
       << "' found! This is bad because this case should never occur!";
    throw std::runtime_error(ss.str());
    return "NO_ENCODING";
  }

  return ros_encoding;
}

bool ArenaCameraBaseNode::setImageEncoding(const std::string &ros_encoding) {
  std::string gen_api_encoding;
  bool conversion_found =
      encoding_conversions::ros2GenAPI(ros_encoding, gen_api_encoding);

  if (ros_encoding.empty()) {
    return false;
  }

  auto node_map = pDevice_->GetNodeMap();

  if (!conversion_found) {
    std::string fallbackPixelFormat =
        Arena::GetNodeValue<GenICam::gcstring>(node_map, "PixelFormat").c_str();
    RCLCPP_ERROR_STREAM(
        this->get_logger(),

        "Can't convert ROS encoding '"
            << ros_encoding
            << "' to a corresponding GenAPI encoding! Will use current "
            << "pixel format ( " << fallbackPixelFormat << " ) as fallback!");
    return false;
  }

  try {
    GenApi::CEnumerationPtr pPixelFormat = node_map->GetNode("PixelFormat");

    if (GenApi::IsWritable(pPixelFormat)) {
      Arena::SetNodeValue<GenICam::gcstring>(node_map, "PixelFormat",
                                             gen_api_encoding.c_str());

      if (currentROSEncoding() == "16UC3" || currentROSEncoding() == "16UC4")
        RCLCPP_WARN(this->get_logger(),
                    "ROS grabbing image data from 3D pixel format, unable to "
                    "display in image viewer");
    }
  } catch (const GenICam::GenericException &e) {
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "An exception while setting image encoding to ROS '"
                            << ros_encoding << "' / GenICam '"
                            << gen_api_encoding
                            << "' occurred: " << e.GetDescription());
    return false;
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "Set image_encoding to ROS '"
                                             << ros_encoding << "' / GenICam '"
                                             << gen_api_encoding);

  if (arena_camera::encoding_conversions::isHDR(ros_encoding)) {
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "Requested HDR encoding \""
                            << ros_encoding
                            << "\", enabling HDR mode in camera");

    try {
      GenApi::CStringPtr pHDROutput =
          pDevice_->GetNodeMap()->GetNode("HDROutput");
      if (GenApi::IsWritable(pHDROutput)) {
        Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(),
                                               "HDROutput", "HDR");
      }

      // Enable HDR image enhancement
      Arena::SetNodeValue<bool>(node_map, "HDRImageEnhancementEnable", true);
      Arena::SetNodeValue<bool>(node_map, "HDRTuningEnable", false);

      Arena::SetNodeValue<GenICam::gcstring>(node_map, "HDROutput", "HDR");

    } catch (GenICam::GenericException &e) {
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "Error while configuring camera: "
                              << e.GetDescription()
                              << ", is this camera capable of HDR?");
      return false;
    }
  }

  return true;
}

//========================================================================
//
// Frame rate
//

double ArenaCameraBaseNode::currentFrameRate() const {
  const auto currentFrameRate = Arena::GetNodeValue<double>(
      pDevice_->GetNodeMap(), "AcquisitionFrameRate");

  return currentFrameRate;
}

// Note that streaming must be stopped before updating frame rate
bool ArenaCameraBaseNode::setFrameRate(float frame_rate) {
  try {
    auto pNodeMap = pDevice_->GetNodeMap();

    // const bool was_streaming = is_streaming_;
    // stopStreaming();

    auto currentFrameRate =
        Arena::GetNodeValue<double>(pNodeMap, "AcquisitionFrameRate");
    auto maximumFrameRate =
        GenApi::CFloatPtr(pNodeMap->GetNode("AcquisitionFrameRate"))->GetMax();

    // requested framerate larger than device max so we trancate it
    if (frame_rate >= maximumFrameRate) {
      RCLCPP_WARN(this->get_logger(),
                  "Desired framerate %.2f Hz (rounded) is higher than "
                  "max possible. "
                  "Will limit framerate device max : %.2f Hz (rounded)",
                  frame_rate, maximumFrameRate);

      frame_rate = maximumFrameRate;
    }
    // special case:
    // dues to inacurate float comparision we skip. If we set it it
    // might throw becase it could be a lil larger than the max avoid
    // the exception (double accuracy issue when setting the node)
    // request frame rate very close to device max
    else if (frame_rate == maximumFrameRate) {
      RCLCPP_INFO(this->get_logger(), "Framerate is %.2f Hz", frame_rate);
    }

    // requested max frame rate
    else if (frame_rate == -1)  // speacial for max frame rate available
    {
      RCLCPP_WARN(this->get_logger(),
                  "Framerate is set to device max : %.2f Hz", maximumFrameRate);
    }

    // requested framerate is valid so we set it to the device
    try {
      if (!Arena::GetNodeValue<bool>(pNodeMap, "AcquisitionFrameRateEnable")) {
        RCLCPP_INFO(this->get_logger(),
                    "\"AcquisitionFrameRateEnable\" not true, trying "
                    "to enable");
        Arena::SetNodeValue<bool>(pNodeMap, "AcquisitionFrameRateEnable", true);
      }
      Arena::SetNodeValue<double>(pNodeMap, "AcquisitionFrameRate", frame_rate);
    } catch (GenICam::GenericException &e) {
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Exception while changing frame rate: " << e.what());
    }
    RCLCPP_INFO_STREAM(
        this->get_logger(),
        "Framerate has been set to "
            << Arena::GetNodeValue<double>(pNodeMap, "AcquisitionFrameRate")
            << " Hz");

    // if (was_streaming) startStreaming();
  } catch (GenICam::GenericException &e) {
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Exception while changing frame rate: " << e.what());
    return false;
  }

  return true;
}

//========================================================================
//
// Exposure
//

bool ArenaCameraBaseNode::setExposure(
    ArenaCameraBaseNode::AutoExposureMode exp_mode, float exposure_ms) {
  auto pNodeMap = pDevice_->GetNodeMap();
  // exposure_auto_ will be already set to false if exposure_given_ is true
  // read params () solved the priority between them
  if (exp_mode == ArenaCameraBaseNode::AutoExposureMode::Off) {
    Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "ExposureAuto", "Off");

    GenApi::CFloatPtr pExposureTime =
        pDevice_->GetNodeMap()->GetNode("ExposureTime");

    float exposure_to_set = exposure_ms * 1000;
    if (exposure_to_set < pExposureTime->GetMin()) {
      RCLCPP_WARN_STREAM(this->get_logger(),
                         "Desired exposure ("
                             << exposure_to_set << ") "
                             << "time unreachable! Setting to lower limit: "
                             << pExposureTime->GetMin());
      exposure_to_set = pExposureTime->GetMin();
    } else if (exposure_to_set > pExposureTime->GetMax()) {
      RCLCPP_WARN_STREAM(this->get_logger(),
                         "Desired exposure ("
                             << exposure_to_set << ") "
                             << "time unreachable! Setting to upper limit: "
                             << pExposureTime->GetMax());
      exposure_to_set = pExposureTime->GetMax();
    }

    pExposureTime->SetValue(exposure_to_set);

    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Setting auto-exposure _off_ with exposure of "
                           << pExposureTime->GetValue() << " ms");
  } else {
    if (exp_mode == ArenaCameraBaseNode::AutoExposureMode::Once) {
      RCLCPP_INFO(this->get_logger(), "Setting auto-exposure to _on_ / Once");
      Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "ExposureAuto", "Once");
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Setting auto-exposure to _on_ / Continuous");
      Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "ExposureAuto",
                                             "Continuous");
    }

    if (exposure_ms > 0) {
      Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "ExposureAutoLimitAuto",
                                             "Off");
      GenApi::CFloatPtr pExposureUpperLimit =
          pDevice_->GetNodeMap()->GetNode("ExposureAutoUpperLimit");
      if (GenApi::IsWritable(pExposureUpperLimit)) {
        // The parameter in the camera is in us
        pExposureUpperLimit->SetValue(static_cast<int64_t>(exposure_ms) * 1000);
      } else {
        RCLCPP_INFO(this->get_logger(),
                    "ExposureAutoUpperLimit is not writeable");
      }

    } else {
      // Use automatic auto-exposure limits
      Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "ExposureAutoLimitAuto",
                                             "Continuous");
    }

    RCLCPP_INFO_STREAM(
        this->get_logger(),
        "Enabling autoexposure with limits "
            << Arena::GetNodeValue<double>(pNodeMap, "ExposureAutoLowerLimit")
            << " to "
            << Arena::GetNodeValue<double>(pNodeMap, "ExposureAutoUpperLimit"));
  }

  return true;
}

bool ArenaCameraBaseNode::setExposure(const arena_camera::Params &p) {
  if (p.auto_exposure) {
    return setExposure(ArenaCameraBaseNode::AutoExposureMode::Continuous,
                       p.auto_exposure_max_ms);
  } else {
    return setExposure(ArenaCameraBaseNode::AutoExposureMode::Off,
                       p.exposure_ms);
  }
}

float ArenaCameraBaseNode::currentExposure() {
  try {
    return Arena::GetNodeValue<double>(pDevice_->GetNodeMap(), "ExposureTime");
  } catch (const GenICam::GenericException &e) {
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "Unable to read exposure: " << e.GetDescription());
    return -1;
  }
}

//========================================================================
//
// Gain
//

bool ArenaCameraBaseNode::setGain(ArenaCameraBaseNode::AutoGainMode gain_mode,
                                  float target_gain) {
  try {
    auto pNodeMap = pDevice_->GetNodeMap();

    Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "GainAuto",
                                           "Off");

    if (gain_mode == ArenaCameraBaseNode::AutoGainMode::Off) {
      RCLCPP_INFO(this->get_logger(), "Setting auto-gain to _off_");
      Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "GainAuto", "Off");
      // todo update parameter on the server

      GenApi::CFloatPtr pGain = pDevice_->GetNodeMap()->GetNode("Gain");
      float truncated_gain = target_gain;

      if (truncated_gain < 0.0) {
        RCLCPP_WARN_STREAM(this->get_logger(),
                           "Desired gain ("
                               << target_gain
                               << ") out of range [0.0,1.0]! Setting to 0.0");
        target_gain = 0.0;
      } else if (truncated_gain > 1.0) {
        RCLCPP_WARN_STREAM(this->get_logger(),
                           "Desired gain ("
                               << target_gain
                               << ") out of range [0.0,1.0]! Setting to 1.0");
        target_gain = 1.0;
      }

      const float gain_min = pGain->GetMin(), gain_max = pGain->GetMax();
      float gain_to_set = gain_min + target_gain * (gain_max - gain_min);
      pGain->SetValue(gain_to_set);

    } else if (gain_mode == ArenaCameraBaseNode::AutoGainMode::Once) {
      Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "GainAuto", "Once");
      RCLCPP_INFO(this->get_logger(), "Setting auto-gain to _on_ / Once");

    } else if (gain_mode == ArenaCameraBaseNode::AutoGainMode::Continuous) {
      Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "GainAuto",
                                             "Continuous");
      RCLCPP_INFO(this->get_logger(), "Setting auto-gain to _on_ / Continuous");
    } else {
      // ??
    }

  } catch (const GenICam::GenericException &e) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "An exception while setting gain: "
                                                << e.GetDescription());
    return false;
  }
  return true;
}

bool ArenaCameraBaseNode::setGain(const arena_camera::Params &p) {
  if (p.auto_gain) {
    return setGain(ArenaCameraBaseNode::AutoGainMode::Continuous);
  } else {
    return setGain(ArenaCameraBaseNode::AutoGainMode::Off, p.gain);
  }
}

float ArenaCameraBaseNode::currentGain() {
  GenApi::CFloatPtr pGain = pDevice_->GetNodeMap()->GetNode("Gain");

  if (!pGain || !GenApi::IsReadable(pGain)) {
    RCLCPP_WARN(this->get_logger(), "No gain value");
    return -1.;
  } else {
    float gainValue = pGain->GetValue();
    return gainValue;
  }
}

//========================================================================
//
// Gamma
//

bool ArenaCameraBaseNode::setGamma(const float &target_gamma) {
  // for GigE cameras you have to enable gamma first

  GenApi::CBooleanPtr pGammaEnable =
      pDevice_->GetNodeMap()->GetNode("GammaEnable");
  if (GenApi::IsWritable(pGammaEnable)) {
    pGammaEnable->SetValue(true);
  }

  GenApi::CFloatPtr pGamma = pDevice_->GetNodeMap()->GetNode("Gamma");
  if (!pGamma || !GenApi::IsWritable(pGamma)) {
    RCLCPP_WARN(this->get_logger(), "Cannot set gamma, it is not writeable");
    return false;
  } else {
    try {
      float gamma_to_set = target_gamma;
      if (pGamma->GetMin() > gamma_to_set) {
        gamma_to_set = pGamma->GetMin();
        RCLCPP_WARN_STREAM(this->get_logger(),
                           "Desired gamma unreachable! Setting to lower limit: "
                               << gamma_to_set);
      } else if (pGamma->GetMax() < gamma_to_set) {
        gamma_to_set = pGamma->GetMax();
        RCLCPP_WARN_STREAM(this->get_logger(),
                           "Desired gamma unreachable! Setting to upper limit: "
                               << gamma_to_set);
      }

      RCLCPP_WARN_STREAM(this->get_logger(),
                         "Setting gamma to " << gamma_to_set);
      pGamma->SetValue(gamma_to_set);

    } catch (const GenICam::GenericException &e) {
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "An exception while setting target gamma to "
                              << target_gamma
                              << " occurred: " << e.GetDescription());
      return false;
    }
  }
  return true;
}

float ArenaCameraBaseNode::currentGamma() {
  GenApi::CFloatPtr pGamma = pDevice_->GetNodeMap()->GetNode("Gamma");

  if (!pGamma || !GenApi::IsReadable(pGamma)) {
    RCLCPP_WARN(this->get_logger(), "No gamma value, returning -1");
    return -1.;
  } else {
    return pGamma->GetValue();
  }
}

//========================================================================
//
// Target brightness
//

void ArenaCameraBaseNode::setTargetBrightness(unsigned int brightness) {
  try {
    GenApi::CIntegerPtr pTargetBrightness =
        pDevice_->GetNodeMap()->GetNode("TargetBrightness");

    if (GenApi::IsWritable(pTargetBrightness)) {
      if (brightness > 255) brightness = 255;
      pTargetBrightness->SetValue(brightness);

      RCLCPP_INFO_STREAM(
          this->get_logger(),
          "Set target brightness to " << pTargetBrightness->GetValue());
    } else {
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "TargetBrightness is not writeable; current value "
                             << pTargetBrightness->GetValue());
    }
  } catch (const GenICam::GenericException &e) {
    RCLCPP_ERROR_STREAM(
        this->get_logger(),
        "An exception while setting TargetBrightness: " << e.GetDescription());
  }
}

//========================================================================
//
// Per-channel HDR gain
//

float ArenaCameraBaseNode::currentHdrGain(int channel) {
  try {
    Arena::SetNodeValue<int64_t>(pDevice_->GetNodeMap(),
                                 "HDRTuningChannelSelector", channel);

    return Arena::GetNodeValue<double>(pDevice_->GetNodeMap(),
                                       "HDRChannelAnalogGain");
  } catch (const GenICam::GenericException &e) {
    RCLCPP_ERROR_STREAM(
        this->get_logger(),
        "Exception while querying HDR gain: " << e.GetDescription());
    return -1;
  }
}

float ArenaCameraBaseNode::currentHdrExposure(int channel) {
  try {
    Arena::SetNodeValue<int64_t>(pDevice_->GetNodeMap(),
                                 "HDRTuningChannelSelector", channel);

    return Arena::GetNodeValue<double>(pDevice_->GetNodeMap(),
                                       "HDRChannelExposureTime");
  } catch (const GenICam::GenericException &e) {
    RCLCPP_ERROR_STREAM(
        this->get_logger(),
        "Exception while querying HDR exposure time: " << e.GetDescription());
    return -1;
  }
}

//========================================================================
//
// Region of interest (ROI)
//

// bool ArenaCameraBaseNode::setROI(
//     const sensor_msgs::RegionOfInterest target_roi,
//     sensor_msgs::RegionOfInterest &reached_roi) {
//   boost::lock_guard<boost::recursive_mutex> lock(device_mutex_);
//   // TODO: set ROI
//   return true;
// }

// sensor_msgs::RegionOfInterest ArenaCameraBaseNode::currentROI() {
//   sensor_msgs::RegionOfInterest roi;
//   // \todo{amarburg}  Broke this by getting rid of a member pImage_
//   //                  Need to save as state?
//   // roi.width = pImage_->GetWidth();
//   // roi.height = pImage_->GetHeight();
//   // ;
//   // roi.x_offset = pImage_->GetOffsetX();
//   // roi.y_offset = pImage_->GetOffsetY();
//   return roi;
// }

//========================================================================
//
// Binning
//

// int64_t ArenaCameraBaseNode::currentBinningX() {
//   GenApi::CIntegerPtr BinningHorizontal =
//       pDevice_->GetNodeMap()->GetNode("BinningHorizontal");

//   if (!BinningHorizontal || !GenApi::IsReadable(BinningHorizontal)) {
//     Node_WARN_STREAM("No binningY value, returning -1");
//     return -1;
//   } else {
//     float binningXValue = BinningHorizontal->GetValue();
//     return binningXValue;
//   }
// }

// int64_t ArenaCameraBaseNode::currentBinningY() {
//   GenApi::CIntegerPtr BinningVertical =
//       pDevice_->GetNodeMap()->GetNode("BinningVertical");

//   if (!BinningVertical || !GenApi::IsReadable(BinningVertical)) {
//     Node_WARN_STREAM("No binningY value, returning -1");
//     return -1;
//   } else {
//     float binningYValue = BinningVertical->GetValue();
//     return binningYValue;
//   }
// }

// bool ArenaCameraBaseNode::setBinningXValue(const size_t &target_binning_x,
//                                               size_t &reached_binning_x) {
//   try {
//     GenApi::CIntegerPtr pBinningHorizontal =
//         pDevice_->GetNodeMap()->GetNode("BinningHorizontal");
//     if (GenApi::IsWritable(pBinningHorizontal)) {
//       size_t binning_x_to_set = target_binning_x;
//       if (binning_x_to_set < pBinningHorizontal->GetMin()) {
//         Node_WARN_STREAM("Desired horizontal binning_x factor("
//                             << binning_x_to_set
//                             << ") unreachable! Setting to lower "
//                             << "limit: " << pBinningHorizontal->GetMin());
//         binning_x_to_set = pBinningHorizontal->GetMin();
//       } else if (binning_x_to_set > pBinningHorizontal->GetMax()) {
//         Node_WARN_STREAM("Desired horizontal binning_x factor("
//                             << binning_x_to_set
//                             << ") unreachable! Setting to upper "
//                             << "limit: " << pBinningHorizontal->GetMax());
//         binning_x_to_set = pBinningHorizontal->GetMax();
//       }

//       pBinningHorizontal->SetValue(binning_x_to_set);
//       reached_binning_x = currentBinningX();
//     } else {
//       Node_WARN_STREAM("Camera does not support binning. Will keep the "
//                           << "current settings");
//       reached_binning_x = currentBinningX();
//     }
//   }

//   catch (const GenICam::GenericException &e) {
//     Node_ERROR_STREAM("An exception while setting target horizontal "
//                          << "binning_x factor to " << target_binning_x
//                          << " occurred: " << e.GetDescription());
//     return false;
//   }
//   return true;
// }

// bool ArenaCameraBaseNode::setBinningX(const size_t &target_binning_x,
//                                          size_t &reached_binning_x) {
//   boost::lock_guard<boost::recursive_mutex> lock(device_mutex_);

//   if (!setBinningXValue(target_binning_x, reached_binning_x)) {
//     // retry till timeout
//     ros::Rate r(10.0);
//     ros::Time timeout(ros::Time::now() + ros::Duration(2.0));
//     while (ros::ok()) {
//       if (setBinningXValue(target_binning_x, reached_binning_x)) {
//         break;
//       }
//       if (ros::Time::now() > timeout) {
//         Node_ERROR_STREAM("Error in setBinningX(): Unable to set target "
//                              << "binning_x factor before timeout");
//         CameraInfoPtr cam_info(
//             new CameraInfo(camera_info_manager_->getCameraInfo()));
//         cam_info->binning_x = currentBinningX();
//         camera_info_manager_->setCameraInfo(*cam_info);
//         //   img_raw_msg_.width = pImage_->GetWidth();
//         //  step = full row length in bytes, img_size = (step * rows),
//         //  imagePixelDepth already contains the number of channels
//         //  img_raw_msg_.step = img_raw_msg_.width *
//         //  (pImage_->GetBitsPerPixel() / 8);
//         return false;
//       }
//       r.sleep();
//     }
//   }

//   return true;
// }

// bool ArenaCameraBaseNode::setBinningYValue(const size_t &target_binning_y,
//                                               size_t &reached_binning_y) {
//   try {
//     GenApi::CIntegerPtr pBinningVertical =
//         pDevice_->GetNodeMap()->GetNode("BinningVertical");
//     if (GenApi::IsWritable(pBinningVertical)) {
//       size_t binning_y_to_set = target_binning_y;
//       if (binning_y_to_set < pBinningVertical->GetMin()) {
//         Node_WARN_STREAM("Desired horizontal binning_y factor("
//                             << binning_y_to_set
//                             << ") unreachable! Setting to lower "
//                             << "limit: " << pBinningVertical->GetMin());
//         binning_y_to_set = pBinningVertical->GetMin();
//       } else if (binning_y_to_set > pBinningVertical->GetMax()) {
//         Node_WARN_STREAM("Desired horizontal binning_y factor("
//                             << binning_y_to_set
//                             << ") unreachable! Setting to upper "
//                             << "limit: " << pBinningVertical->GetMax());
//         binning_y_to_set = pBinningVertical->GetMax();
//       }

//       pBinningVertical->SetValue(binning_y_to_set);
//       reached_binning_y = currentBinningY();
//     } else {
//       Node_WARN_STREAM("Camera does not support binning. Will keep the "
//                           << "current settings");
//       reached_binning_y = currentBinningY();
//     }
//   }

//   catch (const GenICam::GenericException &e) {
//     Node_ERROR_STREAM("An exception while setting target horizontal "
//                          << "binning_y factor to " << target_binning_y
//                          << " occurred: " << e.GetDescription());
//     return false;
//   }
//   return true;
// }

// bool ArenaCameraBaseNode::setBinningY(const size_t &target_binning_y,
//                                          size_t &reached_binning_y) {
//   boost::lock_guard<boost::recursive_mutex> lock(device_mutex_);

//   if (!setBinningYValue(target_binning_y, reached_binning_y)) {
//     // retry till timeout
//     ros::Rate r(10.0);
//     ros::Time timeout(ros::Time::now() + ros::Duration(2.0));
//     while (ros::ok()) {
//       if (setBinningYValue(target_binning_y, reached_binning_y)) {
//         break;
//       }
//       if (ros::Time::now() > timeout) {
//         Node_ERROR_STREAM("Error in setBinningY(): Unable to set target "
//                              << "binning_y factor before timeout");
//         CameraInfoPtr cam_info(
//             new CameraInfo(camera_info_manager_->getCameraInfo()));
//         cam_info->binning_y = currentBinningY();
//         camera_info_manager_->setCameraInfo(*cam_info);

//         // img_raw_msg_.width = pImage_->GetWidth();
//         // //  step = full row length in bytes, img_size = (step * rows),
//         // //  imagePixelDepth already contains the number of channels
//         // img_raw_msg_.step =
//         //     img_raw_msg_.width * (pImage_->GetBitsPerPixel() / 8);
//         return false;
//       }
//       r.sleep();
//     }
//   }

//   return true;
// }

// void ArenaCameraBaseNode::setupSamplingIndices(
//     std::vector<std::size_t> &indices, std::size_t rows, std::size_t cols,
//     int downsampling_factor) {
//   indices.clear();
//   std::size_t min_window_height =
//       static_cast<float>(rows) / static_cast<float>(downsampling_factor);
//   cv::Point2i start_pt(0, 0);
//   cv::Point2i end_pt(cols, rows);
//   // add the iamge center point only once
//   sampling_indices_.push_back(0.5 * rows * cols);
//   genSamplingIndicesRec(indices, min_window_height, start_pt, end_pt);
//   std::sort(indices.begin(), indices.end());
//   return;
// }

// void ArenaCameraBaseNode::genSamplingIndicesRec(
//     std::vector<std::size_t> &indices, const std::size_t &min_window_height,
//     const cv::Point2i &s,  // start
//     const cv::Point2i &e)  // end
// {
//   if (static_cast<std::size_t>(std::abs(e.y - s.y)) <= min_window_height) {
//     return;  // abort criteria -> shrinked window has the min_col_size
//   }
//   /*
//    * sampled img:      point:                             idx:
//    * s 0 0 0 0 0 0  a) [(e.x-s.x)*0.5, (e.y-s.y)*0.5]     a.x*a.y*0.5
//    * 0 0 0 d 0 0 0  b) [a.x,           1.5*a.y]           b.y*img_rows+b.x
//    * 0 0 0 0 0 0 0  c) [0.5*a.x,       a.y]               c.y*img_rows+c.x
//    * 0 c 0 a 0 f 0  d) [a.x,           0.5*a.y]           d.y*img_rows+d.x
//    * 0 0 0 0 0 0 0  f) [1.5*a.x,       a.y]               f.y*img_rows+f.x
//    * 0 0 0 b 0 0 0
//    * 0 0 0 0 0 0 e
//    */
//   cv::Point2i a, b, c, d, f, delta;
//   a = s + 0.5 * (e - s);  // center point
//   delta = 0.5 * (e - s);
//   b = s + cv::Point2i(delta.x, 1.5 * delta.y);
//   c = s + cv::Point2i(0.5 * delta.x, delta.y);
//   d = s + cv::Point2i(delta.x, 0.5 * delta.y);
//   f = s + cv::Point2i(1.5 * delta.x, delta.y);
//   // \todo{amarburg}  This broke when I made pImage_ a non-member
//   // indices.push_back(b.y * pImage_->GetWidth() + b.x);
//   // indices.push_back(c.y * pImage_->GetWidth() + c.x);
//   // indices.push_back(d.y * pImage_->GetWidth() + d.x);
//   // indices.push_back(f.y * pImage_->GetWidth() + f.x);
//   genSamplingIndicesRec(indices, min_window_height, s, a);
//   genSamplingIndicesRec(indices, min_window_height, a, e);
//   genSamplingIndicesRec(indices, min_window_height, cv::Point2i(s.x, a.y),
//                         cv::Point2i(a.x, e.y));
//   genSamplingIndicesRec(indices, min_window_height, cv::Point2i(a.x, s.y),
//                         cv::Point2i(e.x, a.y));
//   return;
// }

// float ArenaCameraBaseNode::calcCurrentBrightness() {
//   boost::lock_guard<boost::recursive_mutex> lock(device_mutex_);
//   if (img_raw_msg_.data.empty()) {
//     return 0.0;
//   }
//   float sum = 0.0;
//   if (sensor_msgs::image_encodings::isMono(img_raw_msg_.encoding)) {
//     // The mean brightness is calculated using a subset of all pixels
//     for (const std::size_t &idx : sampling_indices_) {
//       sum += img_raw_msg_.data.at(idx);
//     }
//     if (sum > 0.0) {
//       sum /= static_cast<float>(sampling_indices_.size());
//     }
//   } else {
//     // The mean brightness is calculated using all pixels and all channels
//     sum =
//         std::accumulate(img_raw_msg_.data.begin(), img_raw_msg_.data.end(),
//         0);
//     if (sum > 0.0) {
//       sum /= static_cast<float>(img_raw_msg_.data.size());
//     }
//   }
//   return sum;
// }

//-------------------------------------------------------------------
// Functions for dealing with LUT
//
// \todo{amarburg}  Very simple right now
//

void ArenaCameraBaseNode::enableLUT(bool enable) {
  try {
    Arena::SetNodeValue<bool>(pDevice_->GetNodeMap(), "LUTEnable", enable);
  } catch (const GenICam::GenericException &e) {
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "An exception while setting LUTEnable to "
                            << (enable ? "true" : "false")
                            << " occurred: " << e.GetDescription());
  }
}

//===================================================================
//  Periodic callback to check if parameters have changed
//

void ArenaCameraBaseNode::checkParametersCb() {
  if (!param_listener_->is_old(params_)) return;

  auto const new_params = param_listener_->get_params();

  // Ugh, why are we back to doing this manually?
  const bool frame_rate_changed = (params_.frame_rate != new_params.frame_rate);
  const bool need_to_stop_streaming = frame_rate_changed;

  if (need_to_stop_streaming) {
    const bool was_streaming = is_streaming_;
    RCLCPP_INFO(this->get_logger(), "Stopping sensor for reconfigure");
    stopStreaming();

    setFrameRate(new_params.frame_rate);

    if (was_streaming) {
      startStreaming();
    }
  }

  // ----- From this point, params which **don't** require stopping streaming
  // to set.

  if ((params_.auto_exposure != new_params.auto_exposure) ||
      (params_.auto_exposure_max_ms != new_params.auto_exposure_max_ms) ||
      (params_.exposure_ms != new_params.exposure_ms)) {
    setExposure(new_params);
  }

  if ((params_.gain != new_params.gain) ||
      (params_.auto_gain != new_params.auto_gain)) {
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Changing gain from "
                           << (params_.auto_gain ? "ON" : "OFF") << " to "
                           << (new_params.auto_gain ? "ON" : "OFF"));
    setGain(new_params);
  }

  if (params_.gamma != new_params.gamma) {
    setGamma(new_params.gamma);
  }

  if (params_.target_brightness != new_params.target_brightness) {
    setTargetBrightness(new_params.target_brightness);
  }

  params_ = new_params;
}

}  // namespace arena_camera
