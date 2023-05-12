/******************************************************************************
 * Software License Agreement (BSD License)
 *
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

// STD
#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>

// ROS
#include <sensor_msgs/RegionOfInterest.h>

#include "boost/multi_array.hpp"

// Arena
#include <ArenaApi.h>
#include <GenApi/GenApi.h>
#include <GenApiCustom.h>

// Arena node
#include "arena_camera/arena_camera_nodelet.h"
#include "arena_camera/encoding_conversions.h"

using diagnostic_msgs::DiagnosticStatus;

namespace arena_camera {

using sensor_msgs::CameraInfo;
using sensor_msgs::CameraInfoPtr;

ArenaCameraNodeletBase::ArenaCameraNodeletBase()
    : pSystem_(nullptr),
      pDevice_(nullptr),
      pNodeMap_(nullptr),
      arena_camera_parameter_set_(),
      set_user_output_srvs_(),
      it_(nullptr),
      img_raw_pub_(),
      pinhole_model_(),
      camera_info_manager_(nullptr),
      sampling_indices_(),
      brightness_exp_lut_(),
      is_sleeping_(false) {}

ArenaCameraNodeletBase::~ArenaCameraNodeletBase() {
  if (pDevice_ != nullptr) {
    pSystem_->DestroyDevice(pDevice_);
  }

  if (pSystem_ != nullptr) {
    Arena::CloseSystem(pSystem_);
  }
}

//
// Nodelet::onInit  function

void ArenaCameraNodeletBase::onInit() {
  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle pnh = getPrivateNodeHandle();

  set_binning_srv_ = nh.advertiseService(
      "set_binning", &ArenaCameraNodeletBase::setBinningCallback, this);
  set_roi_srv_ = nh.advertiseService(
      "set_roi", &ArenaCameraNodeletBase::setROICallback, this);
  set_exposure_srv_ = nh.advertiseService(
      "set_exposure", &ArenaCameraNodeletBase::setExposureCallback, this);
  set_gain_srv_ = nh.advertiseService(
      "set_gain", &ArenaCameraNodeletBase::setGainCallback, this);
  set_gamma_srv_ = nh.advertiseService(
      "set_gamma", &ArenaCameraNodeletBase::setGammaCallback, this);
  set_brightness_srv_ = nh.advertiseService(
      "set_brightness", &ArenaCameraNodeletBase::setBrightnessCallback, this);
  set_sleeping_srv_ = nh.advertiseService(
      "set_sleeping", &ArenaCameraNodeletBase::setSleepingCallback, this);

  it_.reset(new image_transport::ImageTransport(nh));
  img_raw_pub_ = it_->advertiseCamera("image_raw", 1);

  camera_info_manager_ = new camera_info_manager::CameraInfoManager(nh);

  diagnostics_updater_.setHardwareID("none");
  diagnostics_updater_.add("camera_availability", this,
                           &ArenaCameraNodeletBase::create_diagnostics);
  diagnostics_updater_.add(
      "intrinsic_calibration", this,
      &ArenaCameraNodeletBase::create_camera_info_diagnostics);
  diagnostics_trigger_ = nh.createTimer(
      ros::Duration(2), &ArenaCameraNodeletBase::diagnostics_timer_callback_,
      this);

  // Read all parameters from parameter server
  arena_camera_parameter_set_.readFromRosParameterServer(pnh);

  // setting the camera info URL to produce rectified image. Can substitute
  // any desired file path or comment out this line if only producing raw
  // images.
  //  arena_camera_parameter_set_.setCameraInfoURL(nh,
  //  "file://${ROS_HOME}/camera_info/camera.yaml");

  // Open the Arena SDK
  pSystem_ = Arena::OpenSystem();
  pSystem_->UpdateDevices(100);
  if (pSystem_->GetDevices().size() == 0) {
    NODELET_FATAL("Did not detect any cameras!!");
    return;
  }

  if (!arena_camera_parameter_set_.deviceUserID().empty()) {
    if (!registerCameraByUserId(arena_camera_parameter_set_.deviceUserID())) {
      NODELET_FATAL_STREAM("Unable to find a camera with DeviceUserId \""
                           << arena_camera_parameter_set_.deviceUserID()
                           << "\"");
      return;
    }
  } else if (!arena_camera_parameter_set_.serialNumber().empty()) {
    if (!registerCameraBySerialNumber(
            arena_camera_parameter_set_.serialNumber())) {
      NODELET_FATAL_STREAM("Unable to find a camera with Serial Number "
                           << arena_camera_parameter_set_.serialNumber());
      return;
    }
  } else {
    if (!registerCameraByAuto()) {
      NODELET_FATAL_STREAM("Unable to find any cameras to register");
      return;
    }
  }

  // starting the grabbing procedure with the desired image-settings
  if (!startGrabbing()) {
    NODELET_FATAL_STREAM("Unable to configure camera");
    return;
  }
}

bool ArenaCameraNodeletBase::registerCameraByUserId(
    const std::string &device_user_id_to_open) {
  ROS_ASSERT(pSystem_);
  std::vector<Arena::DeviceInfo> deviceInfos = pSystem_->GetDevices();
  ROS_ASSERT(deviceInfos.size() > 0);

  NODELET_INFO_STREAM("Connecting to camera with DeviceUserId"
                      << device_user_id_to_open);

  std::vector<Arena::DeviceInfo>::iterator it;

  for (auto &dev : deviceInfos) {
    const std::string device_user_id(dev.UserDefinedName());
    // Must be an exact match
    if (0 == device_user_id_to_open.compare(device_user_id)) {
      NODELET_INFO_STREAM("Found the desired camera with DeviceUserID "
                          << device_user_id_to_open << ": ");

      pDevice_ = pSystem_->CreateDevice(dev);
      return true;
    }
  }

  NODELET_ERROR_STREAM(
      "Couldn't find the camera that matches the "
      << "given DeviceUserID: \"" << device_user_id_to_open << "\"! "
      << "Either the ID is wrong or the cam is not yet connected");
  return false;
}

bool ArenaCameraNodeletBase::registerCameraBySerialNumber(
    const std::string &serial_number) {
  ROS_ASSERT(pSystem_);
  std::vector<Arena::DeviceInfo> deviceInfos = pSystem_->GetDevices();
  ROS_ASSERT(deviceInfos.size() > 0);

  NODELET_INFO_STREAM("Connecting to camera with Serial Number "
                      << serial_number);

  for (auto &dev : deviceInfos) {
    if (0 == serial_number.compare(dev.SerialNumber())) {
      NODELET_INFO_STREAM("Found the desired camera with Serial Number "
                          << serial_number << ": ");

      pDevice_ = pSystem_->CreateDevice(dev);
      return true;
    }
  }

  NODELET_ERROR_STREAM(
      "Couldn't find the camera that matches the "
      << "given Serial Number: " << serial_number << "! "
      << "Either the ID is wrong or the cam is not yet connected");
  return false;
}

bool ArenaCameraNodeletBase::registerCameraByAuto() {
  ROS_ASSERT(pSystem_);
  std::vector<Arena::DeviceInfo> deviceInfos = pSystem_->GetDevices();
  ROS_ASSERT(deviceInfos.size() > 0);

  int i = 0;
  NODELET_INFO_STREAM("Found " << deviceInfos.size() << " cameras");
  for (auto &dev : deviceInfos) {
    NODELET_INFO_STREAM(i++ << ":  " << dev.SerialNumber() << "  "
                            << dev.UserDefinedName());
  }

  NODELET_INFO_STREAM("Connecting to first autodetected camera: Serial Number "
                      << deviceInfos[0].SerialNumber()
                      << " ; User ID: " << deviceInfos[0].UserDefinedName());
  pDevice_ = pSystem_->CreateDevice(deviceInfos[0]);
  return true;
}

sensor_msgs::RegionOfInterest ArenaCameraNodeletBase::currentROI() {
  sensor_msgs::RegionOfInterest roi;
  // \todo{amarburg}  Broke this by getting ride of pImage_
  //                  Need to save as state?
  // roi.width = pImage_->GetWidth();
  // roi.height = pImage_->GetHeight();
  // ;
  // roi.x_offset = pImage_->GetOffsetX();
  // roi.y_offset = pImage_->GetOffsetY();
  return roi;
}

float ArenaCameraNodeletBase::currentGamma() {
  GenApi::CFloatPtr pGamma = pDevice_->GetNodeMap()->GetNode("Gamma");

  if (!pGamma || !GenApi::IsReadable(pGamma)) {
    NODELET_WARN_STREAM("No gamma value, returning -1");
    return -1.;
  } else {
    float gammaValue = pGamma->GetValue();
    return gammaValue;
  }
}

int64_t ArenaCameraNodeletBase::currentBinningX() {
  GenApi::CIntegerPtr BinningHorizontal =
      pDevice_->GetNodeMap()->GetNode("BinningHorizontal");

  if (!BinningHorizontal || !GenApi::IsReadable(BinningHorizontal)) {
    NODELET_WARN_STREAM("No binningY value, returning -1");
    return -1;
  } else {
    float binningXValue = BinningHorizontal->GetValue();
    return binningXValue;
  }
}

int64_t ArenaCameraNodeletBase::currentBinningY() {
  GenApi::CIntegerPtr BinningVertical =
      pDevice_->GetNodeMap()->GetNode("BinningVertical");

  if (!BinningVertical || !GenApi::IsReadable(BinningVertical)) {
    NODELET_WARN_STREAM("No binningY value, returning -1");
    return -1;
  } else {
    float binningYValue = BinningVertical->GetValue();
    return binningYValue;
  }
}

float ArenaCameraNodeletBase::currentGain() {
  GenApi::CFloatPtr pGain = pDevice_->GetNodeMap()->GetNode("Gain");

  if (!pGain || !GenApi::IsReadable(pGain)) {
    NODELET_WARN_STREAM("No gain value");
    return -1.;
  } else {
    float gainValue = pGain->GetValue();
    return gainValue;
  }
}

float ArenaCameraNodeletBase::currentExposure() {
  GenApi::CFloatPtr pExposureTime =
      pDevice_->GetNodeMap()->GetNode("ExposureTime");

  if (!pExposureTime || !GenApi::IsReadable(pExposureTime)) {
    NODELET_WARN_STREAM("No exposure time value, returning -1");
    return -1.;
  } else {
    float exposureValue = pExposureTime->GetValue();
    return exposureValue;
  }
}

std::string ArenaCameraNodeletBase::currentROSEncoding() {
  std::string gen_api_encoding(Arena::GetNodeValue<GenICam::gcstring>(
      pDevice_->GetNodeMap(), "PixelFormat"));
  std::string ros_encoding("");
  if (!encoding_conversions::genAPI2Ros(gen_api_encoding, ros_encoding)) {
    std::stringstream ss;
    ss << "No ROS equivalent to GenApi encoding '" << gen_api_encoding
       << "' found! This is bad because this case "
          "should never occur!";
    throw std::runtime_error(ss.str());
    return "NO_ENCODING";
  }
  return ros_encoding;
}

bool ArenaCameraNodeletBase::setImageEncoding(const std::string &ros_encoding) {
  std::string gen_api_encoding;
  bool conversion_found =
      encoding_conversions::ros2GenAPI(ros_encoding, gen_api_encoding);
  if (!conversion_found) {
    if (ros_encoding.empty()) {
      return false;
    } else {
      std::string fallbackPixelFormat =
          Arena::GetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(),
                                                 "PixelFormat")
              .c_str();
      NODELET_ERROR_STREAM(
          "Can't convert ROS encoding '"
          << ros_encoding
          << "' to a corresponding GenAPI encoding! Will use current "
          << "pixel format ( " << fallbackPixelFormat << " ) as fallback!");
      return false;
    }
  }
  try {
    GenApi::CEnumerationPtr pPixelFormat =
        pDevice_->GetNodeMap()->GetNode("PixelFormat");
    if (GenApi::IsWritable(pPixelFormat)) {
      Arena::SetNodeValue<GenICam::gcstring>(
          pDevice_->GetNodeMap(), "PixelFormat", gen_api_encoding.c_str());
      if (currentROSEncoding() == "16UC3" || currentROSEncoding() == "16UC4")
        NODELET_WARN_STREAM(
            "ROS grabbing image data from 3D pixel format, unable to display "
            "in image viewer");
    }
  } catch (const GenICam::GenericException &e) {
    NODELET_ERROR_STREAM("An exception while setting target image encoding to '"
                         << ros_encoding
                         << "' occurred: " << e.GetDescription());
    return false;
  }
  return true;
}

bool ArenaCameraNodeletBase::startGrabbing() {
  ros::NodeHandle nh = getNodeHandle();
  auto pNodeMap = pDevice_->GetNodeMap();

  try {
    //
    // Arena device prior streaming settings
    //

    if (Arena::GetNodeValue<GenICam::gcstring>(
            pDevice_->GetNodeMap(), "DeviceTLType") == "GigEVision") {
      NODELET_INFO("GigE device, performing GigE specific configuration");

      // Set Jumbo frames (this is only relevant for GigE cameras.)
      auto pPacketSize = pNodeMap->GetNode("DeviceStreamChannelPacketSize");
      if (GenApi::IsWritable(pPacketSize)) {
        NODELET_INFO_STREAM("Setting MTU to "
                            << arena_camera_parameter_set_.mtuSize());
        Arena::SetNodeValue<int64_t>(pNodeMap, "DeviceStreamChannelPacketSize",
                                     arena_camera_parameter_set_.mtuSize());
      } else {
        NODELET_INFO("Camera MTU is not writeable");
      }
    }

    auto payloadSize = Arena::GetNodeValue<int64_t>(pNodeMap, "PayloadSize");
    NODELET_INFO_STREAM("Expected payload size: " << payloadSize);

    // enable stream auto negotiate packet size
    Arena::SetNodeValue<bool>(pDevice_->GetTLStreamNodeMap(),
                              "StreamAutoNegotiatePacketSize", true);

    // enable stream packet resend
    Arena::SetNodeValue<bool>(pDevice_->GetTLStreamNodeMap(),
                              "StreamPacketResendEnable", true);

    //
    // PIXELFORMAT
    //
    setImageEncoding(arena_camera_parameter_set_.imageEncoding());

    //
    // TRIGGER MODE
    //
    GenApi::CStringPtr pTriggerMode = pNodeMap->GetNode("TriggerMode");
    if (GenApi::IsWritable(pTriggerMode)) {
      Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "TriggerMode", "On");
      Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "TriggerSource",
                                             "Software");
    }

    //
    // FRAMERATE
    //
    auto cmdlnParamFrameRate = arena_camera_parameter_set_.frameRate();
    auto currentFrameRate =
        Arena::GetNodeValue<double>(pNodeMap, "AcquisitionFrameRate");
    auto maximumFrameRate =
        GenApi::CFloatPtr(pNodeMap->GetNode("AcquisitionFrameRate"))->GetMax();

    // requested framerate larger than device max so we trancate it
    if (cmdlnParamFrameRate >= maximumFrameRate) {
      arena_camera_parameter_set_.setFrameRate(nh, maximumFrameRate);

      NODELET_WARN(
          "Desired framerate %.2f Hz (rounded) is higher than max possible. "
          "Will limit "
          "framerate device max : %.2f Hz (rounded)",
          cmdlnParamFrameRate, maximumFrameRate);
    }
    // special case:
    // dues to inacurate float comparision we skip. If we set it it might
    // throw becase it could be a lil larger than the max avoid the exception
    // (double accuracy issue when setting the node) request frame rate very
    // close to device max
    else if (cmdlnParamFrameRate == maximumFrameRate) {
      NODELET_INFO("Framerate is %.2f Hz", cmdlnParamFrameRate);
    }
    // requested max frame rate
    else if (cmdlnParamFrameRate ==
             -1)  // speacial for max frame rate available
    {
      arena_camera_parameter_set_.setFrameRate(nh, maximumFrameRate);

      NODELET_WARN("Framerate is set to device max : %.2f Hz",
                   maximumFrameRate);
    }
    // requested framerate is valid so we set it to the device
    else {
      Arena::SetNodeValue<bool>(pNodeMap, "AcquisitionFrameRateEnable", true);
      Arena::SetNodeValue<double>(pNodeMap, "AcquisitionFrameRate",
                                  cmdlnParamFrameRate);
      NODELET_INFO("Framerate is set to: %.2f Hz", cmdlnParamFrameRate);
    }

    //
    // EXPOSURE AUTO & EXPOSURE
    //

    // exposure_auto_ will be already set to false if exposure_given_ is true
    // read params () solved the priority between them
    if (arena_camera_parameter_set_.exposure_auto_) {
      Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "ExposureAuto",
                                             "Continuous");
      // todo update parameter on the server
      NODELET_INFO_STREAM("Settings Exposure to auto/Continuous");
    } else {
      Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "ExposureAuto", "Off");
      // todo update parameter on the server
      NODELET_INFO_STREAM("Settings Exposure to off/false");
    }

    if (arena_camera_parameter_set_.exposure_given_) {
      float reached_exposure;
      if (setExposure(arena_camera_parameter_set_.exposure_,
                      reached_exposure)) {
        // Note: ont update the ros param because it might keep
        // decreasing or incresing overtime when rerun
        NODELET_INFO_STREAM("Setting exposure to "
                            << arena_camera_parameter_set_.exposure_
                            << ", reached: " << reached_exposure);
      }
    }

    //
    // GAIN
    //

    // gain_auto_ will be already set to false if gain_given_ is true
    // read params () solved the priority between them
    if (arena_camera_parameter_set_.gain_auto_) {
      Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "GainAuto",
                                             "Continuous");
      // todo update parameter on the server
      NODELET_INFO_STREAM("Settings Gain to auto/Continuous");
    } else {
      Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "GainAuto", "Off");
      // todo update parameter on the server
      NODELET_INFO_STREAM("Settings Gain to off/false");
    }

    if (arena_camera_parameter_set_.gain_given_) {
      float reached_gain;
      if (setGain(arena_camera_parameter_set_.gain_, reached_gain)) {
        // Note: ont update the ros param because it might keep
        // decreasing or incresing overtime when rerun
        NODELET_INFO_STREAM("Setting gain to: "
                            << arena_camera_parameter_set_.gain_
                            << ", reached: " << reached_gain);
      }
    }

    //
    // GAMMA
    //
    if (arena_camera_parameter_set_.gamma_given_) {
      float reached_gamma;
      if (setGamma(arena_camera_parameter_set_.gamma_, reached_gamma)) {
        NODELET_INFO_STREAM("Setting gamma to "
                            << arena_camera_parameter_set_.gamma_
                            << ", reached: " << reached_gamma);
      }
    }

    // ------------------------------------------------------------------------

    //
    //  Initial setting of the CameraInfo-msg, assuming no calibration given
    CameraInfo initial_cam_info;
    setupInitialCameraInfo(initial_cam_info);
    camera_info_manager_->setCameraInfo(initial_cam_info);

    if (arena_camera_parameter_set_.cameraInfoURL().empty() ||
        !camera_info_manager_->validateURL(
            arena_camera_parameter_set_.cameraInfoURL())) {
      NODELET_INFO_STREAM("CameraInfoURL needed for rectification! ROS-Param: "
                          << "'" << nh.getNamespace() << "/camera_info_url' = '"
                          << arena_camera_parameter_set_.cameraInfoURL()
                          << "' is invalid!");
      NODELET_DEBUG_STREAM("CameraInfoURL should have following style: "
                           << "'file:///full/path/to/local/file.yaml' or "
                           << "'file://${ROS_HOME}/camera_info/${NAME}.yaml'");
      NODELET_WARN_STREAM("Will only provide distorted /image_raw images!");
    } else {
      // override initial camera info if the url is valid
      if (camera_info_manager_->loadCameraInfo(
              arena_camera_parameter_set_.cameraInfoURL())) {
        // setupRectification();
        // set the correct tf frame_id
        CameraInfoPtr cam_info(
            new CameraInfo(camera_info_manager_->getCameraInfo()));
        cam_info->header.frame_id = img_raw_msg_.header.frame_id;
        camera_info_manager_->setCameraInfo(*cam_info);
      } else {
        NODELET_WARN_STREAM("Will only provide distorted /image_raw images!");
      }
    }

    if (arena_camera_parameter_set_.binning_x_given_) {
      size_t reached_binning_x;
      if (setBinningX(arena_camera_parameter_set_.binning_x_,
                      reached_binning_x)) {
        NODELET_INFO_STREAM("Setting horizontal binning_x to "
                            << arena_camera_parameter_set_.binning_x_);
        NODELET_WARN_STREAM(
            "The image width of the camera_info-msg will "
            << "be adapted, so that the binning_x value in this msg remains 1");
      }
    }

    if (arena_camera_parameter_set_.binning_y_given_) {
      size_t reached_binning_y;
      if (setBinningY(arena_camera_parameter_set_.binning_y_,
                      reached_binning_y)) {
        NODELET_INFO_STREAM("Setting vertical binning_y to "
                            << arena_camera_parameter_set_.binning_y_);
        NODELET_WARN_STREAM(
            "The image height of the camera_info-msg will "
            << "be adapted, so that the binning_y value in this msg remains 1");
      }
    }

    // if (arena_camera_parameter_set_.image_encoding_given_)
    // {
    // 	float reached_image_encoding;
    // 	if (setImageEncoding(arena_camera_parameter_set_.image_encoding_))
    // 	{
    // 		NODELET_INFO_STREAM("Setting exposure to "
    // 						<<
    // arena_camera_parameter_set_.image_encoding_);
    // 	}
    // }

    Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetTLStreamNodeMap(),
                                           "StreamBufferHandlingMode",
                                           "NewestOnly");

    //
    // Trigger Image
    //

    pDevice_->StartStream();
    bool isTriggerArmed = false;

    // if (GenApi::IsWritable(pTriggerMode)) {
    //   do {
    //     isTriggerArmed = Arena::GetNodeValue<bool>(pNodeMap, "TriggerArmed");
    //   } while (isTriggerArmed == false);
    //   // Arena::ExecuteNode(pNodeMap, "TriggerSoftware");
    // }

    // pImage_ = pDevice_->GetImage(5000);
    // pData_ = pImage_->GetData();

    // img_raw_msg_.data.resize(img_raw_msg_.height * img_raw_msg_.step);
    // memcpy(&img_raw_msg_.data[0], pImage_->GetData(),
    //        img_raw_msg_.height * img_raw_msg_.step);
  } catch (GenICam::GenericException &e) {
    NODELET_ERROR_STREAM("Error while grabbing first image occurred: \r\n"
                         << e.GetDescription());
    return false;
  }

  // --------------------------------------------------------------------------

  img_raw_msg_.header.frame_id = arena_camera_parameter_set_.cameraFrame();
  // Encoding of pixels -- channel meaning, ordering, size
  // taken from the list of strings in include/sensor_msgs/image_encodings.h
  // img_raw_msg_.height = pImage_->GetHeight();
  // img_raw_msg_.width = pImage_->GetWidth();
  // step = full row length in bytes, img_size = (step * rows), imagePixelDepth
  // already contains the number of channels
  // img_raw_msg_.step = img_raw_msg_.width * (pImage_->GetBitsPerPixel() / 8);

  if (!camera_info_manager_->setCameraName(std::string(
          Arena::GetNodeValue<GenICam::gcstring>(pNodeMap, "DeviceUserID")
              .c_str()))) {
    // valid name contains only alphanumeric signs and '_'
    NODELET_WARN_STREAM("["
                        << std::string(Arena::GetNodeValue<GenICam::gcstring>(
                                           pNodeMap, "DeviceUserID")
                                           .c_str())
                        << "] name not valid for camera_info_manager");
  }

  NODELET_INFO_STREAM("Startup settings: "
                      << "encoding = '" << currentROSEncoding() << "', "
                      << "binning = [" << currentBinningX() << ", "
                      << currentBinningY() << "], "
                      << "exposure = " << currentExposure() << ", "
                      << "gain = " << currentGain() << ", "
                      << "gamma = " << currentGamma() << ", "
                      << "shutter mode = "
                      << arena_camera_parameter_set_.shutterModeString());

  // pDevice_->RequeueBuffer(pImage_);
  return true;
}

// struct CameraPublisherImpl {
//   image_transport::Publisher image_pub_;
//   ros::Publisher info_pub_;
//   bool unadvertised_;
//   // double constructed_;
// };

// class CameraPublisherLocal {
// public:
//   struct Impl;
//   typedef boost::shared_ptr<Impl> ImplPtr;
//   typedef boost::weak_ptr<Impl> ImplWPtr;

//   CameraPublisherImpl *impl_;
// };

bool ArenaCameraNodeletBase::triggerImage() {
  boost::lock_guard<boost::recursive_mutex> lock(device_mutex_);
  bool retval = false;

  try {
    GenApi::CStringPtr pTriggerMode =
        pDevice_->GetNodeMap()->GetNode("TriggerMode");

    if (GenApi::IsWritable(pTriggerMode)) {
      bool isTriggerArmed = false;

      do {
        isTriggerArmed =
            Arena::GetNodeValue<bool>(pDevice_->GetNodeMap(), "TriggerArmed");
      } while (isTriggerArmed == false);
      Arena::ExecuteNode(pDevice_->GetNodeMap(), "TriggerSoftware");
    }
  } catch (GenICam::GenericException &e) {
    ;
  }

  return true;
}

bool ArenaCameraNodeletBase::grabImage() {
  boost::lock_guard<boost::recursive_mutex> lock(device_mutex_);
  bool retval = false;

  try {
    GenApi::CStringPtr pTriggerMode =
        pDevice_->GetNodeMap()->GetNode("TriggerMode");

    if (GenApi::IsWritable(pTriggerMode)) {
      bool isTriggerArmed = false;

      do {
        isTriggerArmed =
            Arena::GetNodeValue<bool>(pDevice_->GetNodeMap(), "TriggerArmed");
      } while (isTriggerArmed == false);
      Arena::ExecuteNode(pDevice_->GetNodeMap(), "TriggerSoftware");
    }

    Arena::IBuffer *pBuffer = pDevice_->GetBuffer(250);

    if (pBuffer->HasImageData()) {
      Arena::IImage *pImage = dynamic_cast<Arena::IImage *>(pBuffer);

      if (pImage->IsIncomplete()) {
        auto node_map = pDevice_->GetNodeMap();
        const auto payload_size =
            Arena::GetNodeValue<int64_t>(node_map, "PayloadSize");
        if (pBuffer->DataLargerThanBuffer()) {
          NODELET_WARN_STREAM("Image incomplete: data larger than buffer;  "
                              << pBuffer->GetSizeOfBuffer() << " > "
                              << payload_size);
        } else {
          NODELET_WARN_STREAM("Image incomplete: Payload size = "
                              << pBuffer->GetSizeFilled() << " , buffer size = "
                              << pBuffer->GetSizeOfBuffer() << " , expected "
                              << payload_size);
        }

        goto out;
      }

      img_raw_msg_.header.stamp = ros::Time::now();

      // Will return false if PixelEndiannessUnknown
      img_raw_msg_.is_bigendian =
          (pImage->GetPixelEndianness() == Arena::PixelEndiannessBig);

      img_raw_msg_.encoding = currentROSEncoding();
      img_raw_msg_.height = pImage->GetHeight();
      img_raw_msg_.width = pImage->GetWidth();

      const unsigned int bytes_per_pixel = pImage->GetBitsPerPixel() / 8;
      img_raw_msg_.step = img_raw_msg_.width * bytes_per_pixel;

      const unsigned int data_size = img_raw_msg_.height * img_raw_msg_.step;

      // \todo{amarburg} Compare to Buffer/Image payload size

      img_raw_msg_.data.resize(data_size);
      memcpy(&img_raw_msg_.data[0], pImage->GetData(), data_size);

      retval = true;
    }

  out:
    pDevice_->RequeueBuffer(pBuffer);
  } catch (GenICam::GenericException &e) {
    ;
  }

  return retval;
}

bool ArenaCameraNodeletBase::setUserOutputCB(
    const int output_id, camera_control_msgs::SetBool::Request &req,
    camera_control_msgs::SetBool::Response &res) {
  //  res.success = arena_camera_->setUserOutput(output_id, req.data);
  return true;
}

bool ArenaCameraNodeletBase::setAutoflash(
    const int output_id, camera_control_msgs::SetBool::Request &req,
    camera_control_msgs::SetBool::Response &res) {
  NODELET_INFO("AutoFlashCB: %i -> %i", output_id, req.data);
  std::map<int, bool> auto_flashs;
  auto_flashs[output_id] = req.data;
  //    arena_camera_->setAutoflash(auto_flashs);
  res.success = true;
  return true;
}

const double &ArenaCameraNodeletBase::frameRate() const {
  return arena_camera_parameter_set_.frameRate();
}

const std::string &ArenaCameraNodeletBase::cameraFrame() const {
  return arena_camera_parameter_set_.cameraFrame();
}

uint32_t ArenaCameraNodeletBase::getNumSubscribers() const {
  return img_raw_pub_.getNumSubscribers();
}

void ArenaCameraNodeletBase::setupInitialCameraInfo(
    sensor_msgs::CameraInfo &cam_info_msg) {
  std_msgs::Header header;
  header.frame_id = arena_camera_parameter_set_.cameraFrame();
  header.stamp = ros::Time::now();

  // http://www.ros.org/reps/rep-0104.html
  // If the camera is uncalibrated, the matrices D, K, R, P should be left
  // zeroed out. In particular, clients may assume that K[0] == 0.0
  // indicates an uncalibrated camera.
  cam_info_msg.header = header;

  // The image dimensions with which the camera was calibrated. Normally
  // this will be the full camera resolution in pixels. They remain fix,
  // even if binning is applied rows and colums
  cam_info_msg.height =
      Arena::GetNodeValue<int64_t>(pDevice_->GetNodeMap(), "Height");
  cam_info_msg.width =
      Arena::GetNodeValue<int64_t>(pDevice_->GetNodeMap(), "Width");

  // The distortion model used. Supported models are listed in
  // sensor_msgs/distortion_models.h. For most cameras, "plumb_bob" - a
  // simple model of radial and tangential distortion - is sufficient.
  // Empty D and distortion_model indicate that the CameraInfo cannot be
  // used to rectify points or images, either because the camera is not
  // calibrated or because the rectified image was produced using an
  // unsupported distortion model, e.g. the proprietary one used by
  // Bumblebee cameras [http://www.ros.org/reps/rep-0104.html].
  cam_info_msg.distortion_model = "";

  // The distortion parameters, size depending on the distortion model.
  // For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3) ->
  // float64[] D.
  cam_info_msg.D = std::vector<double>(5, 0.);

  // Intrinsic camera matrix for the raw (distorted) images.
  //     [fx  0 cx]
  // K = [ 0 fy cy]  --> 3x3 row-major matrix
  //     [ 0  0  1]
  // Projects 3D points in the camera coordinate frame to 2D pixel
  // coordinates using the focal lengths (fx, fy) and principal point (cx,
  // cy).
  cam_info_msg.K.assign(0.0);

  // Rectification matrix (stereo cameras only)
  // A rotation matrix aligning the camera coordinate system to the ideal
  // stereo image plane so that epipolar lines in both stereo images are
  // parallel.
  cam_info_msg.R.assign(0.0);

  // Projection/camera matrix
  //     [fx'  0  cx' Tx]
  // P = [ 0  fy' cy' Ty]  --> # 3x4 row-major matrix
  //     [ 0   0   1   0]
  // By convention, this matrix specifies the intrinsic (camera) matrix of
  // the processed (rectified) image. That is, the left 3x3 portion is the
  // normal camera intrinsic matrix for the rectified image. It projects 3D
  // points in the camera coordinate frame to 2D pixel coordinates using the
  // focal lengths (fx', fy') and principal point (cx', cy') - these may
  // differ from the values in K. For monocular cameras, Tx = Ty = 0.
  // Normally, monocular cameras will also have R = the identity and
  // P[1:3,1:3] = K. For a stereo pair, the fourth column [Tx Ty 0]' is
  // related to the position of the optical center of the second camera in
  // the first camera's frame. We assume Tz = 0 so both cameras are in the
  // same stereo image plane. The first camera always has Tx = Ty = 0. For
  // the right (second) camera of a horizontal stereo pair, Ty = 0 and Tx =
  // -fx' * B, where B is the baseline between the cameras. Given a 3D point
  // [X Y Z]', the projection (x, y) of the point onto the rectified image
  // is given by: [u v w]' = P * [X Y Z 1]'
  //        x = u / w
  //        y = v / w
  //  This holds for both images of a stereo pair.
  cam_info_msg.P.assign(0.0);

  // Binning refers here to any camera setting which combines rectangular
  // neighborhoods of pixels into larger "super-pixels." It reduces the
  // resolution of the output image to (width / binning_x) x (height /
  // binning_y). The default values binning_x = binning_y = 0 is considered
  // the same as binning_x = binning_y = 1 (no subsampling).
  //  cam_info_msg.binning_x = currentBinningX();
  //  cam_info_msg.binning_y = currentBinningY();

  // Region of interest (subwindow of full camera resolution), given in full
  // resolution (unbinned) image coordinates. A particular ROI always
  // denotes the same window of pixels on the camera sensor, regardless of
  // binning settings. The default setting of roi (all values 0) is
  // considered the same as full resolution (roi.width = width, roi.height =
  // height).

  // todo? do these has ti be set via
  // Arena::GetNodeValue<int64_t>(pDevice_->GetNodeMap(), "OffsetX"); or so
  // ?
  cam_info_msg.roi.x_offset = cam_info_msg.roi.y_offset = 0;
  cam_info_msg.roi.height = cam_info_msg.roi.width = 0;
}

bool ArenaCameraNodeletBase::setROI(
    const sensor_msgs::RegionOfInterest target_roi,
    sensor_msgs::RegionOfInterest &reached_roi) {
  boost::lock_guard<boost::recursive_mutex> lock(device_mutex_);
  // TODO: set ROI
  return true;
}

bool ArenaCameraNodeletBase::setBinningXValue(const size_t &target_binning_x,
                                              size_t &reached_binning_x) {
  try {
    GenApi::CIntegerPtr pBinningHorizontal =
        pDevice_->GetNodeMap()->GetNode("BinningHorizontal");
    if (GenApi::IsWritable(pBinningHorizontal)) {
      size_t binning_x_to_set = target_binning_x;
      if (binning_x_to_set < pBinningHorizontal->GetMin()) {
        NODELET_WARN_STREAM("Desired horizontal binning_x factor("
                            << binning_x_to_set
                            << ") unreachable! Setting to lower "
                            << "limit: " << pBinningHorizontal->GetMin());
        binning_x_to_set = pBinningHorizontal->GetMin();
      } else if (binning_x_to_set > pBinningHorizontal->GetMax()) {
        NODELET_WARN_STREAM("Desired horizontal binning_x factor("
                            << binning_x_to_set
                            << ") unreachable! Setting to upper "
                            << "limit: " << pBinningHorizontal->GetMax());
        binning_x_to_set = pBinningHorizontal->GetMax();
      }

      pBinningHorizontal->SetValue(binning_x_to_set);
      reached_binning_x = currentBinningX();
    } else {
      NODELET_WARN_STREAM("Camera does not support binning. Will keep the "
                          << "current settings");
      reached_binning_x = currentBinningX();
    }
  }

  catch (const GenICam::GenericException &e) {
    NODELET_ERROR_STREAM("An exception while setting target horizontal "
                         << "binning_x factor to " << target_binning_x
                         << " occurred: " << e.GetDescription());
    return false;
  }
  return true;
}

bool ArenaCameraNodeletBase::setBinningX(const size_t &target_binning_x,
                                         size_t &reached_binning_x) {
  boost::lock_guard<boost::recursive_mutex> lock(device_mutex_);

  if (!setBinningXValue(target_binning_x, reached_binning_x)) {
    // retry till timeout
    ros::Rate r(10.0);
    ros::Time timeout(ros::Time::now() + ros::Duration(2.0));
    while (ros::ok()) {
      if (setBinningXValue(target_binning_x, reached_binning_x)) {
        break;
      }
      if (ros::Time::now() > timeout) {
        NODELET_ERROR_STREAM("Error in setBinningX(): Unable to set target "
                             << "binning_x factor before timeout");
        CameraInfoPtr cam_info(
            new CameraInfo(camera_info_manager_->getCameraInfo()));
        cam_info->binning_x = currentBinningX();
        camera_info_manager_->setCameraInfo(*cam_info);
        //   img_raw_msg_.width = pImage_->GetWidth();
        //  step = full row length in bytes, img_size = (step * rows),
        //  imagePixelDepth already contains the number of channels
        //  img_raw_msg_.step = img_raw_msg_.width *
        //  (pImage_->GetBitsPerPixel() / 8);
        return false;
      }
      r.sleep();
    }
  }

  return true;
}

bool ArenaCameraNodeletBase::setBinningYValue(const size_t &target_binning_y,
                                              size_t &reached_binning_y) {
  try {
    GenApi::CIntegerPtr pBinningVertical =
        pDevice_->GetNodeMap()->GetNode("BinningVertical");
    if (GenApi::IsWritable(pBinningVertical)) {
      size_t binning_y_to_set = target_binning_y;
      if (binning_y_to_set < pBinningVertical->GetMin()) {
        NODELET_WARN_STREAM("Desired horizontal binning_y factor("
                            << binning_y_to_set
                            << ") unreachable! Setting to lower "
                            << "limit: " << pBinningVertical->GetMin());
        binning_y_to_set = pBinningVertical->GetMin();
      } else if (binning_y_to_set > pBinningVertical->GetMax()) {
        NODELET_WARN_STREAM("Desired horizontal binning_y factor("
                            << binning_y_to_set
                            << ") unreachable! Setting to upper "
                            << "limit: " << pBinningVertical->GetMax());
        binning_y_to_set = pBinningVertical->GetMax();
      }

      pBinningVertical->SetValue(binning_y_to_set);
      reached_binning_y = currentBinningY();
    } else {
      NODELET_WARN_STREAM("Camera does not support binning. Will keep the "
                          << "current settings");
      reached_binning_y = currentBinningY();
    }
  }

  catch (const GenICam::GenericException &e) {
    NODELET_ERROR_STREAM("An exception while setting target horizontal "
                         << "binning_y factor to " << target_binning_y
                         << " occurred: " << e.GetDescription());
    return false;
  }
  return true;
}

bool ArenaCameraNodeletBase::setBinningY(const size_t &target_binning_y,
                                         size_t &reached_binning_y) {
  boost::lock_guard<boost::recursive_mutex> lock(device_mutex_);

  if (!setBinningYValue(target_binning_y, reached_binning_y)) {
    // retry till timeout
    ros::Rate r(10.0);
    ros::Time timeout(ros::Time::now() + ros::Duration(2.0));
    while (ros::ok()) {
      if (setBinningYValue(target_binning_y, reached_binning_y)) {
        break;
      }
      if (ros::Time::now() > timeout) {
        NODELET_ERROR_STREAM("Error in setBinningY(): Unable to set target "
                             << "binning_y factor before timeout");
        CameraInfoPtr cam_info(
            new CameraInfo(camera_info_manager_->getCameraInfo()));
        cam_info->binning_y = currentBinningY();
        camera_info_manager_->setCameraInfo(*cam_info);

        // img_raw_msg_.width = pImage_->GetWidth();
        // //  step = full row length in bytes, img_size = (step * rows),
        // //  imagePixelDepth already contains the number of channels
        // img_raw_msg_.step =
        //     img_raw_msg_.width * (pImage_->GetBitsPerPixel() / 8);
        return false;
      }
      r.sleep();
    }
  }

  return true;
}

bool ArenaCameraNodeletBase::setBinningCallback(
    camera_control_msgs::SetBinning::Request &req,
    camera_control_msgs::SetBinning::Response &res) {
  size_t reached_binning_x, reached_binning_y;
  bool success_x = setBinningX(req.target_binning_x, reached_binning_x);
  bool success_y = setBinningY(req.target_binning_y, reached_binning_y);
  res.reached_binning_x = static_cast<uint32_t>(reached_binning_x);
  res.reached_binning_y = static_cast<uint32_t>(reached_binning_y);
  res.success = success_x && success_y;
  return true;
}

bool ArenaCameraNodeletBase::setROICallback(
    camera_control_msgs::SetROI::Request &req,
    camera_control_msgs::SetROI::Response &res) {
  res.success = setROI(req.target_roi, res.reached_roi);
  return true;
}

bool ArenaCameraNodeletBase::setExposureValue(const float &target_exposure,
                                              float &reached_exposure) {
  try {
    Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(),
                                           "ExposureAuto", "Off");

    GenApi::CFloatPtr pExposureTime =
        pDevice_->GetNodeMap()->GetNode("ExposureTime");

    float exposure_to_set = target_exposure;
    if (exposure_to_set < pExposureTime->GetMin()) {
      NODELET_WARN_STREAM("Desired exposure ("
                          << exposure_to_set << ") "
                          << "time unreachable! Setting to lower limit: "
                          << pExposureTime->GetMin());
      exposure_to_set = pExposureTime->GetMin();
    } else if (exposure_to_set > pExposureTime->GetMax()) {
      NODELET_WARN_STREAM("Desired exposure ("
                          << exposure_to_set << ") "
                          << "time unreachable! Setting to upper limit: "
                          << pExposureTime->GetMax());
      exposure_to_set = pExposureTime->GetMax();
    }

    pExposureTime->SetValue(exposure_to_set);
    reached_exposure = pExposureTime->GetValue();

    // if ( std::fabs(reached_exposure - exposure_to_set) > exposureStep() )
    // {
    //     // no success if the delta between target and reached exposure
    //     // is greater then the exposure step in ms
    //     return false;
    // }
  } catch (const GenICam::GenericException &e) {
    NODELET_ERROR_STREAM("An exception while setting target exposure to "
                         << target_exposure
                         << " occurred:" << e.GetDescription());
    return false;
  }
  return true;
}

bool ArenaCameraNodeletBase::setExposure(const float &target_exposure,
                                         float &reached_exposure) {
  boost::lock_guard<boost::recursive_mutex> lock(device_mutex_);
  // if ( !pylon_camera_->isReady() )
  // {
  //     NODELET_WARN("Error in setExposure(): pylon_camera_ is not
  //     ready!"); return false;
  // }

  if (ArenaCameraNodeletBase::setExposureValue(target_exposure,
                                               reached_exposure)) {
    // success if the delta is smaller then the exposure step
    return true;
  } else  // retry till timeout
  {
    // wait for max 5s till the cam has updated the exposure
    ros::Rate r(10.0);
    ros::Time timeout(ros::Time::now() + ros::Duration(5.0));
    while (ros::ok()) {
      if (ArenaCameraNodeletBase::setExposureValue(target_exposure,
                                                   reached_exposure)) {
        // success if the delta is smaller then the exposure step
        return true;
      }

      if (ros::Time::now() > timeout) {
        break;
      }
      r.sleep();
    }
    NODELET_ERROR_STREAM("Error in setExposure(): Unable to set target"
                         << " exposure before timeout");
    return false;
  }
}

bool ArenaCameraNodeletBase::setExposureCallback(
    camera_control_msgs::SetExposure::Request &req,
    camera_control_msgs::SetExposure::Response &res) {
  res.success = setExposure(req.target_exposure, res.reached_exposure);
  return true;
}

bool ArenaCameraNodeletBase::setGainValue(const float &target_gain,
                                          float &reached_gain) {
  try {
    Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "GainAuto",
                                           "Off");

    GenApi::CFloatPtr pGain = pDevice_->GetNodeMap()->GetNode("Gain");
    float truncated_gain = target_gain;
    if (truncated_gain < pGain->GetMin()) {
      NODELET_WARN_STREAM(
          "Desired gain ("
          << target_gain << ") in "
          << "percent out of range [0.0 - 1.0]! Setting to lower "
          << "limit: 0.0");
      truncated_gain = pGain->GetMin();
    } else if (truncated_gain > pGain->GetMax()) {
      NODELET_WARN_STREAM(
          "Desired gain ("
          << target_gain << ") in "
          << "percent out of range [0.0 - 1.0]! Setting to upper "
          << "limit: 1.0");
      truncated_gain = pGain->GetMax();
    }

    float gain_to_set =
        pGain->GetMin() + truncated_gain * (pGain->GetMax() - pGain->GetMin());
    pGain->SetValue(gain_to_set);
    reached_gain = currentGain();
  } catch (const GenICam::GenericException &e) {
    NODELET_ERROR_STREAM("An exception while setting target gain to "
                         << target_gain << " occurred: " << e.GetDescription());
    return false;
  }
  return true;
}

bool ArenaCameraNodeletBase::setGain(const float &target_gain,
                                     float &reached_gain) {
  boost::lock_guard<boost::recursive_mutex> lock(device_mutex_);
  // if ( !arena_camera_->isReady() )
  // {
  //         NODELET_WARN("Error in setGain(): arena_camera_ is not
  //         ready!"); return false;
  // }
  //
  if (ArenaCameraNodeletBase::setGainValue(target_gain, reached_gain)) {
    return true;
  } else  // retry till timeout
  {
    // wait for max 5s till the cam has updated the exposure
    ros::Rate r(10.0);
    ros::Time timeout(ros::Time::now() + ros::Duration(5.0));
    while (ros::ok()) {
      if (ArenaCameraNodeletBase::setGainValue(target_gain, reached_gain)) {
        return true;
      }

      if (ros::Time::now() > timeout) {
        break;
      }
      r.sleep();
    }
    NODELET_ERROR_STREAM("Error in setGain(): Unable to set target "
                         << "gain before timeout");
    return false;
  }
}

bool ArenaCameraNodeletBase::setGainCallback(
    camera_control_msgs::SetGain::Request &req,
    camera_control_msgs::SetGain::Response &res) {
  res.success = setGain(req.target_gain, res.reached_gain);
  return true;
}

void ArenaCameraNodeletBase::disableAllRunningAutoBrightessFunctions() {
  GenApi::CStringPtr pExposureAuto = pNodeMap_->GetNode("ExposureAuto");
  GenApi::CStringPtr pGainAuto = pNodeMap_->GetNode("GainAuto");
  if (!pExposureAuto || !GenApi::IsWritable(pExposureAuto) || !pGainAuto ||
      !GenApi::IsWritable(pGainAuto)) {
    NODELET_WARN_STREAM("Unable to disable auto brightness");
    return;
  }

  else {
    Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(),
                                           "ExposureAuto", "Off");
    Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "GainAuto",
                                           "Off");
  }
}

bool ArenaCameraNodeletBase::setGammaValue(const float &target_gamma,
                                           float &reached_gamma) {
  // for GigE cameras you have to enable gamma first

  GenApi::CBooleanPtr pGammaEnable =
      pDevice_->GetNodeMap()->GetNode("GammaEnable");
  if (GenApi::IsWritable(pGammaEnable)) {
    pGammaEnable->SetValue(true);
  }

  GenApi::CFloatPtr pGamma = pDevice_->GetNodeMap()->GetNode("Gamma");
  if (!pGamma || !GenApi::IsWritable(pGamma)) {
    reached_gamma = -1;
    return true;
  } else {
    try {
      float gamma_to_set = target_gamma;
      if (pGamma->GetMin() > gamma_to_set) {
        gamma_to_set = pGamma->GetMin();
        NODELET_WARN_STREAM(
            "Desired gamma unreachable! Setting to lower limit: "
            << gamma_to_set);
      } else if (pGamma->GetMax() < gamma_to_set) {
        gamma_to_set = pGamma->GetMax();
        NODELET_WARN_STREAM(
            "Desired gamma unreachable! Setting to upper limit: "
            << gamma_to_set);
      }
      pGamma->SetValue(gamma_to_set);
      reached_gamma = currentGamma();
    } catch (const GenICam::GenericException &e) {
      NODELET_ERROR_STREAM("An exception while setting target gamma to "
                           << target_gamma
                           << " occurred: " << e.GetDescription());
      return false;
    }
  }
  return true;
}

bool ArenaCameraNodeletBase::setGamma(const float &target_gamma,
                                      float &reached_gamma) {
  boost::lock_guard<boost::recursive_mutex> lock(device_mutex_);
  if (ArenaCameraNodeletBase::setGammaValue(target_gamma, reached_gamma)) {
    return true;
  } else  // retry till timeout
  {
    // wait for max 5s till the cam has updated the gamma value
    ros::Rate r(10.0);
    ros::Time timeout(ros::Time::now() + ros::Duration(5.0));
    while (ros::ok()) {
      if (ArenaCameraNodeletBase::setGammaValue(target_gamma, reached_gamma)) {
        return true;
      }

      if (ros::Time::now() > timeout) {
        break;
      }
      r.sleep();
    }
    NODELET_ERROR_STREAM("Error in setGamma(): Unable to set target "
                         << "gamma before timeout");
    return false;
  }
  return true;
}

bool ArenaCameraNodeletBase::setGammaCallback(
    camera_control_msgs::SetGamma::Request &req,
    camera_control_msgs::SetGamma::Response &res) {
  res.success = setGamma(req.target_gamma, res.reached_gamma);
  return true;
}

bool ArenaCameraNodeletBase::setBrightness(const int &target_brightness,
                                           int &reached_brightness,
                                           const bool &exposure_auto,
                                           const bool &gain_auto) {
  boost::lock_guard<boost::recursive_mutex> lock(device_mutex_);
  ros::Time begin =
      ros::Time::now();  // time measurement for the exposure search

  // brightness service can only work, if an image has already been grabbed,
  // because it calculates the mean on the current image. The interface is
  // ready if the grab-result-pointer of the first acquisition contains
  // valid data
  int target_brightness_co = std::min(255, target_brightness);
  // smart brightness search initially sets the last rememberd exposure time
  if (brightness_exp_lut_.at(target_brightness_co) != 0.0) {
    float reached_exp;
    if (!setExposure(brightness_exp_lut_.at(target_brightness_co),
                     reached_exp)) {
      NODELET_WARN_STREAM("Tried to speed-up exposure search with initial"
                          << " guess, but setting the exposure failed!");
    } else {
      NODELET_DEBUG_STREAM("Speed-up exposure search with initial exposure"
                           << " guess of " << reached_exp);
    }
  }

  // get actual image -> fills img_raw_msg_.data vector
  if (!grabImage()) {
    NODELET_ERROR("Failed to grab image, can't calculate current brightness!");
    return false;
  }

  // calculates current brightness by generating the mean over all pixels
  // stored in img_raw_msg_.data vector
  float current_brightness = calcCurrentBrightness();

  NODELET_DEBUG_STREAM("New brightness request for target brightness "
                       << target_brightness_co
                       << ", current brightness = " << current_brightness);

  if (std::fabs(current_brightness -
                static_cast<float>(target_brightness_co)) <= 1.0) {
    reached_brightness = static_cast<int>(current_brightness);
    ros::Time end = ros::Time::now();
    NODELET_DEBUG_STREAM(
        "Brightness reached without exposure search, duration: "
        << (end - begin).toSec());
    return true;  // target brightness already reached
  }

  // initially cancel all running exposure search by deactivating
  // ExposureAuto & AutoGain
  disableAllRunningAutoBrightessFunctions();

  if (target_brightness_co <= 50) {
    // own binary-exp search: we need to have the upper bound -> ArenaAuto
    // exposure to a initial start value of 50 provides it
    if (brightness_exp_lut_.at(50) != 0.0) {
      float reached_exp;
      if (!setExposure(brightness_exp_lut_.at(50), reached_exp)) {
        NODELET_WARN_STREAM("Tried to speed-up exposure search with initial"
                            << " guess, but setting the exposure failed!");
      } else {
        NODELET_DEBUG_STREAM("Speed-up exposure search with initial exposure"
                             << " guess of " << reached_exp);
      }
    }
  }

  if (!exposure_auto && !gain_auto) {
    NODELET_WARN_STREAM(
        "Neither Auto Exposure Time ('exposure_auto') nor Auto "
        << "Gain ('gain_auto') are enabled! Hence gain and exposure time "
        << "are assumed to be fix and the target brightness ("
        << target_brightness_co << ") can not be reached!");
    return false;
  }

  bool is_brightness_reached = false;
  size_t fail_safe_ctr = 0;
  size_t fail_safe_ctr_limit = 10;

  float last_brightness = std::numeric_limits<float>::max();

  // timeout for the exposure search -> need more time for great exposure
  // values
  ros::Time start_time = ros::Time::now();
  ros::Time timeout = start_time;
  if (target_brightness_co < 205) {
    timeout +=
        ros::Duration(arena_camera_parameter_set_.exposure_search_timeout_);
  } else {
    timeout += ros::Duration(
        10.0 + arena_camera_parameter_set_.exposure_search_timeout_);
  }

  while (ros::ok()) {
    // calling setBrightness in every cycle would not be necessary for the
    // arena auto brightness search. But for the case that the target
    // brightness is out of the arena range which is from [50 - 205] a
    // binary exposure search will be executed where we have to update the
    // search parameter in every cycle if (
    // !arena_camera_->setBrightness(target_brightness_co,
    //                                    current_brightness,
    //                                    exposure_auto,
    //                                    gain_auto) )
    // {
    //         disableAllRunningAutoBrightessFunctions();
    //         break;
    // }

    if (!grabImage()) {
      return false;
    }

    // if ( arena_camera_->isArenaAutoBrightnessFunctionRunning() )
    // {
    //         // do nothing if the arena auto function is running, we need
    //         to
    //         // wait till it's finished
    //         /*
    //            NODELET_DEBUG_STREAM("ArenaAutoBrightnessFunction still
    //            running! "
    //             << " Current brightness: " << calcCurrentBrightness()
    //             << ", current exposure: " << currentExposure());
    //          */
    //         continue;
    // }

    current_brightness = calcCurrentBrightness();
    // is_brightness_reached = fabs(current_brightness -
    // static_cast<float>(target_brightness_co))
    //                         < arena_camera_->maxBrightnessTolerance();
    //
    // if ( is_brightness_reached )
    // {
    //         disableAllRunningAutoBrightessFunctions();
    //         break;
    // }

    if (std::fabs(last_brightness - current_brightness) <= 1.0) {
      fail_safe_ctr++;
    } else {
      fail_safe_ctr = 0;
    }

    last_brightness = current_brightness;

    if ((fail_safe_ctr > fail_safe_ctr_limit) && !is_brightness_reached) {
      NODELET_WARN_STREAM("Seems like the desired brightness ("
                          << target_brightness_co
                          << ") is not reachable! Stuck at brightness "
                          << current_brightness << " and exposure "
                          << currentExposure() << "us");
      disableAllRunningAutoBrightessFunctions();
      reached_brightness = static_cast<int>(current_brightness);
      return false;
    }

    if (ros::Time::now() > timeout) {
      // cancel all running brightness search by deactivating ExposureAuto
      disableAllRunningAutoBrightessFunctions();
      NODELET_WARN_STREAM("Did not reach the target brightness before "
                          << "timeout of " << (timeout - start_time).sec
                          << " sec! Stuck at brightness " << current_brightness
                          << " and exposure " << currentExposure() << "us");
      reached_brightness = static_cast<int>(current_brightness);
      return false;
    }
  }

  NODELET_DEBUG_STREAM("Finally reached brightness: " << current_brightness);
  reached_brightness = static_cast<int>(current_brightness);

  // store reached brightness - exposure tuple for next times search
  if (is_brightness_reached) {
    if (brightness_exp_lut_.at(reached_brightness) == 0.0) {
      brightness_exp_lut_.at(reached_brightness) = currentExposure();
    } else {
      brightness_exp_lut_.at(reached_brightness) += currentExposure();
      brightness_exp_lut_.at(reached_brightness) *= 0.5;
    }
    if (brightness_exp_lut_.at(target_brightness_co) == 0.0) {
      brightness_exp_lut_.at(target_brightness_co) = currentExposure();
    } else {
      brightness_exp_lut_.at(target_brightness_co) += currentExposure();
      brightness_exp_lut_.at(target_brightness_co) *= 0.5;
    }
  }
  ros::Time end = ros::Time::now();
  NODELET_DEBUG_STREAM("Brightness search duration: " << (end - begin).toSec());
  return is_brightness_reached;
}

bool ArenaCameraNodeletBase::setBrightnessCallback(
    camera_control_msgs::SetBrightness::Request &req,
    camera_control_msgs::SetBrightness::Response &res) {
  res.success = setBrightness(req.target_brightness, res.reached_brightness,
                              req.exposure_auto, req.gain_auto);
  if (req.brightness_continuous) {
    if (req.exposure_auto) {
      //  arena_camera_->enableContinuousAutoExposure();
    }
    if (req.gain_auto) {
      //    arena_camera_->enableContinuousAutoGain();
    }
  }
  res.reached_exposure_time = currentExposure();
  res.reached_gain_value = currentGain();
  return true;
}

void ArenaCameraNodeletBase::setupSamplingIndices(
    std::vector<std::size_t> &indices, std::size_t rows, std::size_t cols,
    int downsampling_factor) {
  indices.clear();
  std::size_t min_window_height =
      static_cast<float>(rows) / static_cast<float>(downsampling_factor);
  cv::Point2i start_pt(0, 0);
  cv::Point2i end_pt(cols, rows);
  // add the iamge center point only once
  sampling_indices_.push_back(0.5 * rows * cols);
  genSamplingIndicesRec(indices, min_window_height, start_pt, end_pt);
  std::sort(indices.begin(), indices.end());
  return;
}

void ArenaCameraNodeletBase::genSamplingIndicesRec(
    std::vector<std::size_t> &indices, const std::size_t &min_window_height,
    const cv::Point2i &s,  // start
    const cv::Point2i &e)  // end
{
  if (static_cast<std::size_t>(std::abs(e.y - s.y)) <= min_window_height) {
    return;  // abort criteria -> shrinked window has the min_col_size
  }
  /*
   * sampled img:      point:                             idx:
   * s 0 0 0 0 0 0  a) [(e.x-s.x)*0.5, (e.y-s.y)*0.5]     a.x*a.y*0.5
   * 0 0 0 d 0 0 0  b) [a.x,           1.5*a.y]           b.y*img_rows+b.x
   * 0 0 0 0 0 0 0  c) [0.5*a.x,       a.y]               c.y*img_rows+c.x
   * 0 c 0 a 0 f 0  d) [a.x,           0.5*a.y]           d.y*img_rows+d.x
   * 0 0 0 0 0 0 0  f) [1.5*a.x,       a.y]               f.y*img_rows+f.x
   * 0 0 0 b 0 0 0
   * 0 0 0 0 0 0 e
   */
  cv::Point2i a, b, c, d, f, delta;
  a = s + 0.5 * (e - s);  // center point
  delta = 0.5 * (e - s);
  b = s + cv::Point2i(delta.x, 1.5 * delta.y);
  c = s + cv::Point2i(0.5 * delta.x, delta.y);
  d = s + cv::Point2i(delta.x, 0.5 * delta.y);
  f = s + cv::Point2i(1.5 * delta.x, delta.y);
  // \todo{amarburg}  This broke when I made pImage_ a non-member
  // indices.push_back(b.y * pImage_->GetWidth() + b.x);
  // indices.push_back(c.y * pImage_->GetWidth() + c.x);
  // indices.push_back(d.y * pImage_->GetWidth() + d.x);
  // indices.push_back(f.y * pImage_->GetWidth() + f.x);
  genSamplingIndicesRec(indices, min_window_height, s, a);
  genSamplingIndicesRec(indices, min_window_height, a, e);
  genSamplingIndicesRec(indices, min_window_height, cv::Point2i(s.x, a.y),
                        cv::Point2i(a.x, e.y));
  genSamplingIndicesRec(indices, min_window_height, cv::Point2i(a.x, s.y),
                        cv::Point2i(e.x, a.y));
  return;
}

float ArenaCameraNodeletBase::calcCurrentBrightness() {
  boost::lock_guard<boost::recursive_mutex> lock(device_mutex_);
  if (img_raw_msg_.data.empty()) {
    return 0.0;
  }
  float sum = 0.0;
  if (sensor_msgs::image_encodings::isMono(img_raw_msg_.encoding)) {
    // The mean brightness is calculated using a subset of all pixels
    for (const std::size_t &idx : sampling_indices_) {
      sum += img_raw_msg_.data.at(idx);
    }
    if (sum > 0.0) {
      sum /= static_cast<float>(sampling_indices_.size());
    }
  } else {
    // The mean brightness is calculated using all pixels and all channels
    sum =
        std::accumulate(img_raw_msg_.data.begin(), img_raw_msg_.data.end(), 0);
    if (sum > 0.0) {
      sum /= static_cast<float>(img_raw_msg_.data.size());
    }
  }
  return sum;
}

bool ArenaCameraNodeletBase::setSleepingCallback(
    camera_control_msgs::SetSleeping::Request &req,
    camera_control_msgs::SetSleeping::Response &res) {
  is_sleeping_ = req.set_sleeping;

  if (is_sleeping_) {
    NODELET_INFO("Setting Arena Camera Node to sleep...");
  } else {
    NODELET_INFO("Arena Camera Node continues grabbing");
  }

  res.success = true;
  return true;
}

//------------------------------------------------------------------------
//  ROS Disagnostics callbacks
//

void ArenaCameraNodeletBase::create_diagnostics(
    diagnostic_updater::DiagnosticStatusWrapper &stat) {}

void ArenaCameraNodeletBase::create_camera_info_diagnostics(
    diagnostic_updater::DiagnosticStatusWrapper &stat) {
  if (camera_info_manager_->isCalibrated()) {
    stat.summaryf(DiagnosticStatus::OK, "Intrinsic calibration found");
  } else {
    stat.summaryf(DiagnosticStatus::ERROR, "No intrinsic calibration found");
  }
}

void ArenaCameraNodeletBase::diagnostics_timer_callback_(
    const ros::TimerEvent &) {
  diagnostics_updater_.update();
}

}  // namespace arena_camera
