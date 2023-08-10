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

// STD
// #include <algorithm>
// #include <boost/multi_array.hpp>
// #include <cmath>
// #include <cstring>
// #include <string>
// #include <vector>

// ROS2
#include <rcpputils/asserts.hpp>
// #include <dynamic_reconfigure/SensorLevels.h>
// #include <sensor_msgs/RegionOfInterest.h>

// Arena
#include <ArenaApi.h>
#include <GenApi/GenApi.h>
#include <GenApiCustom.h>

// Arena node
#include "arena_camera/arena_camera_nodes.h"
#include "arena_camera/encoding_conversions.h"

// using diagnostic_msgs::DiagnosticStatus;

namespace arena_camera {

// using sensor_msgs::CameraInfo;
// using sensor_msgs::CameraInfoPtr;

ArenaCameraBaseNode::ArenaCameraBaseNode(const std::string &node_name,
                                         const rclcpp::NodeOptions &options)
    : Node(node_name, options),
      pSystem_(nullptr),
      pDevice_(nullptr),
      pNodeMap_(nullptr),
      is_streaming_(false)
// arena_camera_parameter_set_(),
// set_user_output_srvs_(),
// it_(nullptr),
// img_raw_pub_(),
// pinhole_model_(),
// camera_info_manager_(nullptr),
// sampling_indices_()
{
  //   metadata_pub_ =
  //       nh.advertise<imaging_msgs::ImagingMetadata>("imaging_metadata",
  //       1);

  //   it_.reset(new image_transport::ImageTransport(nh));
  //   img_raw_pub_ = it_->advertiseCamera("image_raw", 1);

  //   camera_info_manager_ = new
  //   camera_info_manager::CameraInfoManager(nh);

  //   diagnostics_updater_.setHardwareID("none");
  //   diagnostics_updater_.add("camera_availability", this,
  //                            &ArenaCameraBaseNode::create_diagnostics);
  //   diagnostics_updater_.add(
  //       "intrinsic_calibration", this,
  //       &ArenaCameraBaseNode::create_camera_info_diagnostics);
  //   diagnostics_trigger_ = nh.createTimer(
  //       ros::Duration(2),
  //       &ArenaCameraBaseNode::diagnostics_timer_callback_,
  //       this);

  //   // Initialize parameters from parameter server
  //   arena_camera_parameter_set_.readFromRosParameterServer(pnh);

  this->declare_parameter("device_user_id", "");
  this->declare_parameter("serial_number", "");

  std::string device_user_id, serial_number;

  // Open the Arena SDK
  pSystem_ = Arena::OpenSystem();
  pSystem_->UpdateDevices(100);
  if (pSystem_->GetDevices().size() == 0) {
    RCLCPP_FATAL(this->get_logger(), "Did not detect any cameras!!");
    return;
  }

  if (this->get_parameter("device_user_id", device_user_id)) {
    if (!registerCameraByUserId(device_user_id)) {
      RCLCPP_FATAL_STREAM(this->get_logger(),
                          "Unable to find a camera with DeviceUserId\""
                              << device_user_id << "\"");
      return;
    }
  } else if (this->get_parameter("serial_number", serial_number)) {
    if (!registerCameraBySerialNumber(serial_number)) {
      RCLCPP_FATAL_STREAM(
          this->get_logger(),
          "Unable to find a camera with Serial Number" << serial_number);
      return;
    }
  } else {
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

  //   if (!configureCamera()) {
  //     Node_FATAL_STREAM("Unable to configure camera");
  //     return;
  //   }

  //   _dynReconfigureServer =
  //   std::make_shared<DynReconfigureServer>(pnh);
  //   _dynReconfigureServer->setCallback(boost::bind(
  //       &ArenaCameraBaseNode::reconfigureCallbackWrapper, this, _1,
  //       _2));
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

  RCLCPP_INFO_STREAM(
      this->get_logger(),
      "Connecting to camera with DeviceUserId" << device_user_id_to_open);

  std::vector<Arena::DeviceInfo>::iterator it;

  for (auto &dev : deviceInfos) {
    const std::string device_user_id(dev.UserDefinedName());
    // Must be an exact match
    if (0 == device_user_id_to_open.compare(device_user_id)) {
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Found the desired camera with DeviceUserID "
                             << device_user_id_to_open << ": ");

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
  RCLCPP_DEBUG_STREAM(this->get_logger(),
                      "Found " << deviceInfos.size() << " cameras");
  for (auto &dev : deviceInfos) {
    RCLCPP_INFO_STREAM(this->get_logger(), i++ << ":  " << dev.SerialNumber()
                                               << "  "
                                               << dev.UserDefinedName());
  }

  for (auto &dev : deviceInfos) {
    if (dev.VendorName() == "Lucid Vision Labs") {
      pDevice_ = pSystem_->CreateDevice(dev);
      RCLCPP_INFO_STREAM(
          this->get_logger(),
          "Connecting to first autodetected Lucid Vision camera: Serial Number "
              << dev.SerialNumber() << " ; User ID: " << dev.UserDefinedName());
      return true;
    }
  }

  return false;
}

// bool ArenaCameraBaseNode::configureCamera() {
//   ros::NodeHandle nh = getNodeHandle();
//   auto pNodeMap = pDevice_->GetNodeMap();

//   // **NOTE** only configuration which is not also accessible through
//   // dynamic_reconfigure.  Those will be handled in the callback
//   // when it is called for the first time at node startup.

//   try {
//     Node_INFO_STREAM(
//         "   Device model: " << Arena::GetNodeValue<GenICam::gcstring>(
//             pDevice_->GetNodeMap(), "DeviceModelName"));
//     Node_INFO_STREAM(
//         "Device firmware: " << Arena::GetNodeValue<GenICam::gcstring>(
//             pDevice_->GetNodeMap(), "DeviceFirmwareVersion"));

//     //
//     // Arena device prior streaming settings
//     //

//     if (Arena::GetNodeValue<GenICam::gcstring>(
//             pDevice_->GetNodeMap(), "DeviceTLType") == "GigEVision") {
//       Node_INFO("GigE device, performing GigE specific configuration:");

//       // Set Jumbo frames (this is only relevant for GigE cameras.)
//       auto pPacketSize = pNodeMap->GetNode("DeviceStreamChannelPacketSize");
//       if (GenApi::IsWritable(pPacketSize)) {
//         Node_INFO_STREAM(" -> Setting MTU to "
//                             << arena_camera_parameter_set_.mtuSize());
//         Arena::SetNodeValue<int64_t>(pNodeMap,
//         "DeviceStreamChannelPacketSize",
//                                      arena_camera_parameter_set_.mtuSize());
//       } else {
//         Node_INFO(" -> Camera MTU is not writeable");
//       }
//     }

//     auto payloadSize = Arena::GetNodeValue<int64_t>(pNodeMap, "PayloadSize");
//     Node_INFO_STREAM("Expected payload size: " << payloadSize);

//     // enable stream auto negotiate packet size
//     Node_DEBUG("Enabling auto-negotiation of packet size");
//     Arena::SetNodeValue<bool>(pDevice_->GetTLStreamNodeMap(),
//                               "StreamAutoNegotiatePacketSize", true);

//     // enable stream packet resend
//     Node_DEBUG("Enabling packet resend");
//     Arena::SetNodeValue<bool>(pDevice_->GetTLStreamNodeMap(),
//                               "StreamPacketResendEnable", true);

//     //
//     // PIXELFORMAT
//     //
//     setImageEncoding(arena_camera_parameter_set_.imageEncoding());

//     if (encoding_conversions::isHDR(
//             arena_camera_parameter_set_.imageEncoding())) {
//       hdr_metadata_pub_ = nh.advertise<imaging_msgs::HdrImagingMetadata>(
//           "hdr_imaging_metadata", 1);
//     }

//     //
//     // TRIGGER MODE
//     //
//     GenApi::CStringPtr pTriggerMode = pNodeMap->GetNode("TriggerMode");
//     if (GenApi::IsWritable(pTriggerMode)) {
//       Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "TriggerMode", "On");
//       Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "TriggerSource",
//                                              "Software");
//     }

//     //!! Parameters controlled by param / dynamic reonfigure are not set here
//     //!! Assume there will be an immediate call from dynamic reconfigure

//     // LUT
//     Node_INFO_STREAM(
//         (arena_camera_parameter_set_.enable_lut_ ? "Enabling" : "Disabling")
//         << " camera LUT");
//     enableLUT(arena_camera_parameter_set_.enable_lut_);

//     //
//     ------------------------------------------------------------------------

//     //
//     //  Initial setting of the CameraInfo-msg, assuming no calibration given
//     CameraInfo initial_cam_info;
//     // initializeCameraInfo(initial_cam_info);
//     camera_info_manager_->setCameraInfo(initial_cam_info);

//     if (arena_camera_parameter_set_.cameraInfoURL().empty() ||
//         !camera_info_manager_->validateURL(
//             arena_camera_parameter_set_.cameraInfoURL())) {
//       Node_INFO_STREAM("CameraInfoURL needed for rectification! ROS-Param: "
//                           << "'" << nh.getNamespace() << "/camera_info_url' =
//                           '"
//                           << arena_camera_parameter_set_.cameraInfoURL()
//                           << "' is invalid!");
//       Node_DEBUG_STREAM("CameraInfoURL should have following style: "
//                            << "'file:///full/path/to/local/file.yaml' or "
//                            <<
//                            "'file://${ROS_HOME}/camera_info/${NAME}.yaml'");
//       Node_WARN_STREAM("Will only provide distorted /image_raw images!");
//     } else {
//       // override initial camera info if the url is valid
//       if (camera_info_manager_->loadCameraInfo(
//               arena_camera_parameter_set_.cameraInfoURL())) {
//         // setupRectification();
//         // set the correct tf frame_id
//         CameraInfoPtr cam_info(
//             new CameraInfo(camera_info_manager_->getCameraInfo()));
//         cam_info->header.frame_id = img_raw_msg_.header.frame_id;
//         camera_info_manager_->setCameraInfo(*cam_info);
//       } else {
//         Node_WARN_STREAM("Will only provide distorted /image_raw images!");
//       }
//     }

//     if (arena_camera_parameter_set_.binning_x_given_) {
//       size_t reached_binning_x;
//       if (setBinningX(arena_camera_parameter_set_.binning_x_,
//                       reached_binning_x)) {
//         Node_INFO_STREAM("Setting horizontal binning_x to "
//                             << arena_camera_parameter_set_.binning_x_);
//         Node_WARN_STREAM(
//             "The image width of the camera_info-msg will "
//             << "be adapted, so that the binning_x value in this msg remains
//             1");
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
//             << "be adapted, so that the binning_y value in this msg remains
//             1");
//       }
//     }

//     // if (arena_camera_parameter_set_.image_encoding_given_)
//     // {
//     // 	float reached_image_encoding;
//     // 	if
//     (setImageEncoding(arena_camera_parameter_set_.image_encoding_))
//     // 	{
//     // 		Node_INFO_STREAM("Setting encoding to "
//     // 						<<
//     // arena_camera_parameter_set_.image_encoding_);
//     // 	}
//     // }

//     Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetTLStreamNodeMap(),
//                                            "StreamBufferHandlingMode",
//                                            "NewestOnly");

//     // bool isTriggerArmed = false;

//     // if (GenApi::IsWritable(pTriggerMode)) {
//     //   do {
//     //     isTriggerArmed = Arena::GetNodeValue<bool>(pNodeMap,
//     "TriggerArmed");
//     //   } while (isTriggerArmed == false);
//     //   // Arena::ExecuteNode(pNodeMap, "TriggerSoftware");
//     // }

//     // pImage_ = pDevice_->GetImage(5000);
//     // pData_ = pImage_->GetData();

//     // img_raw_msg_.data.resize(img_raw_msg_.height * img_raw_msg_.step);
//     // memcpy(&img_raw_msg_.data[0], pImage_->GetData(),
//     //        img_raw_msg_.height * img_raw_msg_.step);
//   } catch (GenICam::GenericException &e) {
//     Node_ERROR_STREAM("Error while configuring camera: \r\n"
//                          << e.GetDescription());
//     return false;
//   }

//   //
//   --------------------------------------------------------------------------

//   img_raw_msg_.header.frame_id = arena_camera_parameter_set_.cameraFrame();
//   // Encoding of pixels -- channel meaning, ordering, size
//   // taken from the list of strings in include/sensor_msgs/image_encodings.h
//   // img_raw_msg_.height = pImage_->GetHeight();
//   // img_raw_msg_.width = pImage_->GetWidth();
//   // step = full row length in bytes, img_size = (step * rows),
//   imagePixelDepth
//   // already contains the number of channels
//   // img_raw_msg_.step = img_raw_msg_.width * (pImage_->GetBitsPerPixel() /
//   8);

//   if (!camera_info_manager_->setCameraName(std::string(
//           Arena::GetNodeValue<GenICam::gcstring>(pNodeMap, "DeviceUserID")
//               .c_str()))) {
//     // valid name contains only alphanumeric signs and '_'
//     Node_WARN_STREAM("["
//                         <<
//                         std::string(Arena::GetNodeValue<GenICam::gcstring>(
//                                            pNodeMap, "DeviceUserID")
//                                            .c_str())
//                         << "] name not valid for camera_info_manager");
//   }

//   // Node_INFO("=== Startup settings ===");
//   // Node_INFO_STREAM("encoding = " << currentROSEncoding());
//   // Node_INFO_STREAM("binning = [" << currentBinningX() << " x "
//   //                                   << currentBinningY() << "]");
//   // Node_INFO_STREAM("exposure = " << currentExposure() << " us");
//   // Node_INFO_STREAM("gain = " << currentGain());
//   // Node_INFO_STREAM("gamma = " << currentGamma());
//   // Node_INFO_STREAM(
//   //     "shutter mode = " <<
//   arena_camera_parameter_set_.shutterModeString());
//   // Node_INFO("========================");

//   // pDevice_->RequeueBuffer(pImage_);
//   return true;
// }

// void ArenaCameraBaseNode::startStreaming() {
//   if (!is_streaming_) {
//     pDevice_->StartStream();
//     is_streaming_ = true;
//   }
// }

// void ArenaCameraBaseNode::stopStreaming() {
//   if (is_streaming_) {
//     pDevice_->StopStream();
//     is_streaming_ = false;
//   }
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

// float ArenaCameraBaseNode::currentGamma() {
//   GenApi::CFloatPtr pGamma = pDevice_->GetNodeMap()->GetNode("Gamma");

//   if (!pGamma || !GenApi::IsReadable(pGamma)) {
//     Node_WARN_STREAM("No gamma value, returning -1");
//     return -1.;
//   } else {
//     float gammaValue = pGamma->GetValue();
//     return gammaValue;
//   }
// }

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

// float ArenaCameraBaseNode::currentGain() {
//   GenApi::CFloatPtr pGain = pDevice_->GetNodeMap()->GetNode("Gain");

//   if (!pGain || !GenApi::IsReadable(pGain)) {
//     Node_WARN_STREAM("No gain value");
//     return -1.;
//   } else {
//     float gainValue = pGain->GetValue();
//     return gainValue;
//   }
// }

// std::string ArenaCameraBaseNode::currentROSEncoding() {
//   std::string gen_api_encoding(Arena::GetNodeValue<GenICam::gcstring>(
//       pDevice_->GetNodeMap(), "PixelFormat"));
//   std::string ros_encoding("");
//   if (!encoding_conversions::genAPI2Ros(gen_api_encoding, ros_encoding)) {
//     std::stringstream ss;
//     ss << "No ROS equivalent to GenApi encoding '" << gen_api_encoding
//        << "' found! This is bad because this case "
//           "should never occur!";
//     throw std::runtime_error(ss.str());
//     return "NO_ENCODING";
//   }
//   return ros_encoding;
// }

// bool ArenaCameraBaseNode::setImageEncoding(const std::string &ros_encoding) {
//   std::string gen_api_encoding;
//   bool conversion_found =
//       encoding_conversions::ros2GenAPI(ros_encoding, gen_api_encoding);
//   if (!conversion_found) {
//     if (ros_encoding.empty()) {
//       return false;
//     } else {
//       std::string fallbackPixelFormat =
//           Arena::GetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(),
//                                                  "PixelFormat")
//               .c_str();
//       Node_ERROR_STREAM(
//           "Can't convert ROS encoding '"
//           << ros_encoding
//           << "' to a corresponding GenAPI encoding! Will use current "
//           << "pixel format ( " << fallbackPixelFormat << " ) as fallback!");
//       return false;
//     }
//   }
//   try {
//     GenApi::CEnumerationPtr pPixelFormat =
//         pDevice_->GetNodeMap()->GetNode("PixelFormat");
//     if (GenApi::IsWritable(pPixelFormat)) {
//       Arena::SetNodeValue<GenICam::gcstring>(
//           pDevice_->GetNodeMap(), "PixelFormat", gen_api_encoding.c_str());
//       if (currentROSEncoding() == "16UC3" || currentROSEncoding() == "16UC4")
//         Node_WARN_STREAM(
//             "ROS grabbing image data from 3D pixel format, unable to display
//             " "in image viewer");
//     }
//   } catch (const GenICam::GenericException &e) {
//     Node_ERROR_STREAM("An exception while setting target image encoding to '"
//                          << ros_encoding
//                          << "' occurred: " << e.GetDescription());
//     return false;
//   }

//   Node_INFO("Have configured image_encoding");

//   if (arena_camera::encoding_conversions::isHDR(ros_encoding)) {
//     Node_INFO_STREAM("Requested HDR encoding \""
//                         << ros_encoding << "\", enabling HDR mode in
//                         camera");

//     try {
//       auto pNodeMap = pDevice_->GetNodeMap();

//       // GenApi::CStringPtr pHDROutput = pNodeMap->GetNode("HDROutput");
//       // if (GenApi::IsWritable(pHDROutput)) {
//       //   Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "HDROutput",
//       //   "HDR");
//       // }

//       // Enable HDR image enhancement
//       Arena::SetNodeValue<bool>(pNodeMap, "HDRImageEnhancementEnable", true);
//       Arena::SetNodeValue<bool>(pNodeMap, "HDRTuningEnable", false);

//       Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "HDROutput", "HDR");

//     } catch (GenICam::GenericException &e) {
//       Node_ERROR_STREAM("Error while configuring camera: "
//                            << e.GetDescription()
//                            << ", is this camera capable of HDR?");
//       return false;
//     }
//   }

//   return true;
// }

// // Note that streaming must be stopped before updating frame rate
// void ArenaCameraBaseNode::updateFrameRate() {
//   try {
//     ros::NodeHandle nh = getNodeHandle();
//     auto pNodeMap = pDevice_->GetNodeMap();

//     // const bool was_streaming = is_streaming_;
//     // stopStreaming();

//     auto cmdlnParamFrameRate = arena_camera_parameter_set_.frameRate();
//     auto currentFrameRate =
//         Arena::GetNodeValue<double>(pNodeMap, "AcquisitionFrameRate");
//     auto maximumFrameRate =
//         GenApi::CFloatPtr(pNodeMap->GetNode("AcquisitionFrameRate"))->GetMax();

//     // requested framerate larger than device max so we trancate it
//     if (cmdlnParamFrameRate >= maximumFrameRate) {
//       arena_camera_parameter_set_.setFrameRate(maximumFrameRate);

//       Node_WARN(
//           "Desired framerate %.2f Hz (rounded) is higher than max possible. "
//           "Will limit "
//           "framerate device max : %.2f Hz (rounded)",
//           cmdlnParamFrameRate, maximumFrameRate);
//     }
//     // special case:
//     // dues to inacurate float comparision we skip. If we set it it might
//     // throw becase it could be a lil larger than the max avoid the exception
//     // (double accuracy issue when setting the node) request frame rate very
//     // close to device max
//     else if (cmdlnParamFrameRate == maximumFrameRate) {
//       Node_INFO("Framerate is %.2f Hz", cmdlnParamFrameRate);
//     }
//     // requested max frame rate
//     else if (cmdlnParamFrameRate ==
//              -1)  // speacial for max frame rate available
//     {
//       arena_camera_parameter_set_.setFrameRate(maximumFrameRate);

//       Node_WARN("Framerate is set to device max : %.2f Hz",
//                    maximumFrameRate);
//     }

//     // requested framerate is valid so we set it to the device
//     try {
//       if (!Arena::GetNodeValue<bool>(pNodeMap, "AcquisitionFrameRateEnable"))
//       {
//         Node_INFO(
//             "\"AcquisitionFrameRateEnable\" not true, trying to enable");
//         Arena::SetNodeValue<bool>(pNodeMap, "AcquisitionFrameRateEnable",
//         true);
//       }
//       Arena::SetNodeValue<double>(pNodeMap, "AcquisitionFrameRate",
//                                   arena_camera_parameter_set_.frameRate());
//     } catch (GenICam::GenericException &e) {
//       Node_INFO_STREAM("Exception while changing frame rate: " << e.what());
//     }
//     Node_INFO_STREAM(
//         "Framerate has been set to "
//         << Arena::GetNodeValue<double>(pNodeMap, "AcquisitionFrameRate")
//         << " Hz");

//     // if (was_streaming) startStreaming();

//   } catch (GenICam::GenericException &e) {
//     Node_INFO_STREAM("Exception while changing frame rate: " << e.what());
//   }
// }

// // void ArenaCameraBaseNode::initializeCameraInfo(
// //     sensor_msgs::CameraInfo &cam_info_msg) {
// //   // http://www.ros.org/reps/rep-0104.html
// //   // If the camera is uncalibrated, the matrices D, K, R, P should be left
// //   // zeroed out. In particular, clients may assume that K[0] == 0.0
// //   // indicates an uncalibrated camera.
// //   cam_info_msg.header.frame_id = cameraFrame();
// //   cam_info_msg.header.stamp = ros::Time::now();

// //   // The image dimensions with which the camera was calibrated. Normally
// //   // this will be the full camera resolution in pixels. They remain fix,
// //   // even if binning is applied rows and colums
// //   cam_info_msg.height =
// //       Arena::GetNodeValue<int64_t>(pDevice_->GetNodeMap(), "Height");
// //   cam_info_msg.width =
// //       Arena::GetNodeValue<int64_t>(pDevice_->GetNodeMap(), "Width");

// //   // The distortion model used. Supported models are listed in
// //   // sensor_msgs/distortion_models.h. For most cameras, "plumb_bob" - a
// //   // simple model of radial and tangential distortion - is sufficient.
// //   // Empty D and distortion_model indicate that the CameraInfo cannot be
// //   // used to rectify points or images, either because the camera is not
// //   // calibrated or because the rectified image was produced using an
// //   // unsupported distortion model, e.g. the proprietary one used by
// //   // Bumblebee cameras [http://www.ros.org/reps/rep-0104.html].
// //   cam_info_msg.distortion_model = "";

// //   // The distortion parameters, size depending on the distortion model.
// //   // For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3) ->
// //   // float64[] D.
// //   cam_info_msg.D = std::vector<double>(5, 0.);

// //   // Intrinsic camera matrix for the raw (distorted) images.
// //   //     [fx  0 cx]
// //   // K = [ 0 fy cy]  --> 3x3 row-major matrix
// //   //     [ 0  0  1]
// //   // Projects 3D points in the camera coordinate frame to 2D pixel
// //   // coordinates using the focal lengths (fx, fy) and principal point (cx,
// //   // cy).
// //   cam_info_msg.K.assign(0.0);

// //   // Rectification matrix (stereo cameras only)
// //   // A rotation matrix aligning the camera coordinate system to the ideal
// //   // stereo image plane so that epipolar lines in both stereo images are
// //   // parallel.
// //   cam_info_msg.R.assign(0.0);

// //   // Projection/camera matrix
// //   //     [fx'  0  cx' Tx]
// //   // P = [ 0  fy' cy' Ty]  --> # 3x4 row-major matrix
// //   //     [ 0   0   1   0]
// //   // By convention, this matrix specifies the intrinsic (camera) matrix of
// //   // the processed (rectified) image. That is, the left 3x3 portion is the
// //   // normal camera intrinsic matrix for the rectified image. It projects
// 3D
// //   // points in the camera coordinate frame to 2D pixel coordinates using
// the
// //   // focal lengths (fx', fy') and principal point (cx', cy') - these may
// //   // differ from the values in K. For monocular cameras, Tx = Ty = 0.
// //   // Normally, monocular cameras will also have R = the identity and
// //   // P[1:3,1:3] = K. For a stereo pair, the fourth column [Tx Ty 0]' is
// //   // related to the position of the optical center of the second camera in
// //   // the first camera's frame. We assume Tz = 0 so both cameras are in the
// //   // same stereo image plane. The first camera always has Tx = Ty = 0. For
// //   // the right (second) camera of a horizontal stereo pair, Ty = 0 and Tx
// =
// //   // -fx' * B, where B is the baseline between the cameras. Given a 3D
// point
// //   // [X Y Z]', the projection (x, y) of the point onto the rectified image
// //   // is given by: [u v w]' = P * [X Y Z 1]'
// //   //        x = u / w
// //   //        y = v / w
// //   //  This holds for both images of a stereo pair.
// //   cam_info_msg.P.assign(0.0);

// //   // Binning refers here to any camera setting which combines rectangular
// //   // neighborhoods of pixels into larger "super-pixels." It reduces the
// //   // resolution of the output image to (width / binning_x) x (height /
// //   // binning_y). The default values binning_x = binning_y = 0 is
// considered
// //   // the same as binning_x = binning_y = 1 (no subsampling).
// //   //  cam_info_msg.binning_x = currentBinningX();
// //   //  cam_info_msg.binning_y = currentBinningY();

// //   // Region of interest (subwindow of full camera resolution), given in
// full
// //   // resolution (unbinned) image coordinates. A particular ROI always
// //   // denotes the same window of pixels on the camera sensor, regardless of
// //   // binning settings. The default setting of roi (all values 0) is
// //   // considered the same as full resolution (roi.width = width, roi.height
// =
// //   // height).

// //   // todo? do these has ti be set via
// //   // Arena::GetNodeValue<int64_t>(pDevice_->GetNodeMap(), "OffsetX"); or
// so
// //   // ?
// //   cam_info_msg.roi.x_offset = cam_info_msg.roi.y_offset = 0;
// //   cam_info_msg.roi.height = cam_info_msg.roi.width = 0;
// // }

// //~~ Binning / ROI ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// bool ArenaCameraBaseNode::setROI(
//     const sensor_msgs::RegionOfInterest target_roi,
//     sensor_msgs::RegionOfInterest &reached_roi) {
//   boost::lock_guard<boost::recursive_mutex> lock(device_mutex_);
//   // TODO: set ROI
//   return true;
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

// //~~ Brightness / auto controls ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// void ArenaCameraBaseNode::setTargetBrightness(unsigned int brightness) {
//   // const bool was_streaming = is_streaming_;
//   // stopStreaming();

//   try {
//     GenApi::CIntegerPtr pTargetBrightness =
//         pDevice_->GetNodeMap()->GetNode("TargetBrightness");

//     if (GenApi::IsWritable(pTargetBrightness)) {
//       if (brightness > 255) brightness = 255;
//       pTargetBrightness->SetValue(brightness);

//       Node_INFO_STREAM("Set target brightness to "
//                           << pTargetBrightness->GetValue());
//     } else {
//       Node_INFO_STREAM("TargetBrightness is not writeable; current value "
//                           << pTargetBrightness->GetValue());
//     }
//   } catch (const GenICam::GenericException &e) {
//     Node_ERROR_STREAM(
//         "An exception while setting TargetBrightness: " <<
//         e.GetDescription());
//   }

//   // if (was_streaming)
//   //   startStreaming();
// }

// //~~ Exposure ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// float ArenaCameraBaseNode::currentExposure() {
//   GenApi::CFloatPtr pExposureTime =
//       pDevice_->GetNodeMap()->GetNode("ExposureTime");

//   if (!pExposureTime || !GenApi::IsReadable(pExposureTime)) {
//     Node_WARN_STREAM("No exposure time value, returning -1");
//     return -1.;
//   } else {
//     float exposureValue = pExposureTime->GetValue();
//     return exposureValue;
//   }
// }

// void ArenaCameraBaseNode::setExposure(
//     ArenaCameraBaseNode::AutoExposureMode exp_mode, float exposure_ms) {
//   auto pNodeMap = pDevice_->GetNodeMap();
//   // exposure_auto_ will be already set to false if exposure_given_ is true
//   // read params () solved the priority between them
//   if (exp_mode == ArenaCameraBaseNode::AutoExposureMode::Off) {
//     Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "ExposureAuto", "Off");

//     GenApi::CFloatPtr pExposureTime =
//         pDevice_->GetNodeMap()->GetNode("ExposureTime");

//     float exposure_to_set = exposure_ms * 1000;
//     if (exposure_to_set < pExposureTime->GetMin()) {
//       Node_WARN_STREAM("Desired exposure ("
//                           << exposure_to_set << ") "
//                           << "time unreachable! Setting to lower limit: "
//                           << pExposureTime->GetMin());
//       exposure_to_set = pExposureTime->GetMin();
//     } else if (exposure_to_set > pExposureTime->GetMax()) {
//       Node_WARN_STREAM("Desired exposure ("
//                           << exposure_to_set << ") "
//                           << "time unreachable! Setting to upper limit: "
//                           << pExposureTime->GetMax());
//       exposure_to_set = pExposureTime->GetMax();
//     }

//     pExposureTime->SetValue(exposure_to_set);

//     Node_INFO_STREAM("Setting auto-exposure _off_ with exposure of "
//                         << pExposureTime->GetValue() << " ms");
//   } else {
//     if (exp_mode == ArenaCameraBaseNode::AutoExposureMode::Once) {
//       Node_INFO_STREAM("Setting auto-exposure to _on_ / Once");
//       Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "ExposureAuto",
//       "Once");
//     } else {
//       Node_INFO_STREAM("Setting auto-exposure to _on_ / Continuous");
//       Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "ExposureAuto",
//                                              "Continuous");
//     }

//     if (exposure_ms > 0) {
//       Arena::SetNodeValue<GenICam::gcstring>(pNodeMap,
//       "ExposureAutoLimitAuto",
//                                              "Off");
//       GenApi::CFloatPtr pExposureUpperLimit =
//           pDevice_->GetNodeMap()->GetNode("ExposureAutoUpperLimit");
//       if (GenApi::IsWritable(pExposureUpperLimit)) {
//         // The parameter in the camera is in us
//         pExposureUpperLimit->SetValue(static_cast<int64_t>(exposure_ms) *
//         1000);
//       } else {
//         Node_INFO("ExposureAutoUpperLimit is not writeable");
//       }

//     } else {
//       // Use automatic auto-exposure limits
//       Arena::SetNodeValue<GenICam::gcstring>(pNodeMap,
//       "ExposureAutoLimitAuto",
//                                              "Continuous");
//     }

//     Node_INFO_STREAM(
//         "Enabling autoexposure with limits "
//         << Arena::GetNodeValue<double>(pNodeMap, "ExposureAutoLowerLimit")
//         << " to "
//         << Arena::GetNodeValue<double>(pNodeMap, "ExposureAutoUpperLimit"));
//   }
// }

// //~~ Gain ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// bool ArenaCameraBaseNode::setGain(
//     ArenaCameraBaseNode::AutoGainMode gain_mode, float target_gain) {
//   try {
//     auto pNodeMap = pDevice_->GetNodeMap();

//     Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(),
//     "GainAuto",
//                                            "Off");

//     if (gain_mode == ArenaCameraBaseNode::AutoGainMode::Off) {
//       Node_INFO_STREAM("Setting auto-gain to _off_");
//       Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "GainAuto", "Off");
//       // todo update parameter on the server

//       GenApi::CFloatPtr pGain = pDevice_->GetNodeMap()->GetNode("Gain");
//       float truncated_gain = target_gain;

//       if (truncated_gain < 0.0) {
//         Node_WARN_STREAM("Desired gain ("
//                             << target_gain
//                             << ") out of range [0.0,1.0]! Setting to 0.0");
//         target_gain = 0.0;
//       } else if (truncated_gain > 1.0) {
//         Node_WARN_STREAM("Desired gain ("
//                             << target_gain
//                             << ") out of range [0.0,1.0]! Setting to 1.0");
//         target_gain = 1.0;
//       }

//       const float gain_min = pGain->GetMin(), gain_max = pGain->GetMax();
//       float gain_to_set = gain_min + target_gain * (gain_max - gain_min);
//       pGain->SetValue(gain_to_set);

//     } else if (gain_mode == ArenaCameraBaseNode::AutoGainMode::Once) {
//       Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "GainAuto", "Once");
//       Node_INFO_STREAM("Setting auto-gain to _on_ / Once");

//     } else if (gain_mode == ArenaCameraBaseNode::AutoGainMode::Continuous) {
//       Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "GainAuto",
//                                              "Continuous");
//       Node_INFO_STREAM("Setting auto-gain to _on_ / Continuous");
//     } else {
//     }

//   } catch (const GenICam::GenericException &e) {
//     Node_ERROR_STREAM(
//         "An exception while setting gain: " << e.GetDescription());
//     return false;
//   }
//   return true;
// }

// void ArenaCameraBaseNode::disableAllRunningAutoBrightessFunctions() {
//   GenApi::CStringPtr pExposureAuto = pNodeMap_->GetNode("ExposureAuto");
//   GenApi::CStringPtr pGainAuto = pNodeMap_->GetNode("GainAuto");

//   if (!pExposureAuto || !GenApi::IsWritable(pExposureAuto) || !pGainAuto ||
//       !GenApi::IsWritable(pGainAuto)) {
//     Node_WARN_STREAM("Unable to disable auto gain & exposure");
//     return;
//   }

//   else {
//     Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(),
//                                            "ExposureAuto", "Off");
//     Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(),
//     "GainAuto",
//                                            "Off");
//   }
// }

// //~~ Gamma ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// bool ArenaCameraBaseNode::setGamma(const float &target_gamma) {
//   // for GigE cameras you have to enable gamma first

//   GenApi::CBooleanPtr pGammaEnable =
//       pDevice_->GetNodeMap()->GetNode("GammaEnable");
//   if (GenApi::IsWritable(pGammaEnable)) {
//     pGammaEnable->SetValue(true);
//   }

//   GenApi::CFloatPtr pGamma = pDevice_->GetNodeMap()->GetNode("Gamma");
//   if (!pGamma || !GenApi::IsWritable(pGamma)) {
//     Node_WARN("Cannot set gamma, it is not writeable");
//     return false;
//   } else {
//     try {
//       float gamma_to_set = target_gamma;
//       if (pGamma->GetMin() > gamma_to_set) {
//         gamma_to_set = pGamma->GetMin();
//         Node_WARN_STREAM(
//             "Desired gamma unreachable! Setting to lower limit: "
//             << gamma_to_set);
//       } else if (pGamma->GetMax() < gamma_to_set) {
//         gamma_to_set = pGamma->GetMax();
//         Node_WARN_STREAM(
//             "Desired gamma unreachable! Setting to upper limit: "
//             << gamma_to_set);
//       }

//       Node_INFO_STREAM("Setting gamma to " << gamma_to_set);
//       pGamma->SetValue(gamma_to_set);

//     } catch (const GenICam::GenericException &e) {
//       Node_ERROR_STREAM("An exception while setting target gamma to "
//                            << target_gamma
//                            << " occurred: " << e.GetDescription());
//       return false;
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

// //-------------------------------------------------------------------
// //
// // HDR Channel Query and set functions
// //
// float ArenaCameraBaseNode::currentHdrGain(int channel) {
//   try {
//     Arena::SetNodeValue<int64_t>(pDevice_->GetNodeMap(),
//                                  "HDRTuningChannelSelector", channel);

//     return Arena::GetNodeValue<double>(pDevice_->GetNodeMap(),
//                                        "HDRChannelAnalogGain");
//   } catch (const GenICam::GenericException &e) {
//     Node_ERROR_STREAM(
//         "Exception while querying HDR gain: " << e.GetDescription());
//     return -1;
//   }
// }

// float ArenaCameraBaseNode::currentHdrExposure(int channel) {
//   try {
//     Arena::SetNodeValue<int64_t>(pDevice_->GetNodeMap(),
//                                  "HDRTuningChannelSelector", channel);

//     return Arena::GetNodeValue<double>(pDevice_->GetNodeMap(),
//                                        "HDRChannelExposureTime");
//   } catch (const GenICam::GenericException &e) {
//     Node_ERROR_STREAM(
//         "Exception while querying HDR exposure time: " <<
//         e.GetDescription());
//     return -1;
//   }
// }

// //-------------------------------------------------------------------
// // Functions for dealing with LUT
// //
// // \todo{amarburg}  Very simple right now
// //

// void ArenaCameraBaseNode::enableLUT(bool enable) {
//   try {
//     Arena::SetNodeValue<bool>(pDevice_->GetNodeMap(), "LUTEnable", enable);
//   } catch (const GenICam::GenericException &e) {
//     Node_ERROR_STREAM("An exception while setting LUTEnable to "
//                          << (enable ? "true" : "false")
//                          << " occurred: " << e.GetDescription());
//   }
// }

// //------------------------------------------------------------------------
// //  ROS Reconfigure callback
// //

// void ArenaCameraBaseNode::reconfigureCallback(ArenaCameraConfig &config,
//                                                  uint32_t level) {
//   const auto stop_level =
//       (uint32_t)dynamic_reconfigure::SensorLevels::RECONFIGURE_STOP;

//   const bool was_streaming = is_streaming_;
//   if (level >= stop_level) {
//     ROS_INFO("Stopping sensor for reconfigure");
//     stopStreaming();
//   }

//   Node_INFO_STREAM("In reconfigureCallback");

//   // -- The following params require stopping streaming, only set if needed
//   -- if (config.frame_rate != previous_config_.frame_rate) {
//     arena_camera_parameter_set_.setFrameRate(config.frame_rate);
//     updateFrameRate();
//   }

//   // if (config.target_brightness != previous_config_.target_brightness) {
//   setTargetBrightness(config.target_brightness);
//   //}

//   // -- The following can be set while streaming, just set them every time

//   // if ((config.auto_exposure != previous_config_.auto_exposure) ||
//   //     (config.auto_exposure_max_ms !=
//   previous_config_.auto_exposure_max_ms)
//   //     || (config.exposure_ms != previous_config_.exposure_ms)) {
//   arena_camera_parameter_set_.exposure_auto_ = config.auto_exposure;
//   arena_camera_parameter_set_.exposure_ms_ = config.exposure_ms;
//   arena_camera_parameter_set_.auto_exposure_max_ms_ =
//       config.auto_exposure_max_ms;

//   if (config.auto_exposure) {
//     setExposure(ArenaCameraBaseNode::AutoExposureMode::Continuous,
//                 config.auto_exposure_max_ms);
//   } else {
//     setExposure(ArenaCameraBaseNode::AutoExposureMode::Off,
//                 config.exposure_ms);
//   }
//   // }

//   // if ((config.auto_gain != previous_config_.auto_gain)) {
//   arena_camera_parameter_set_.gain_auto_ = config.auto_gain;

//   if (arena_camera_parameter_set_.gain_auto_) {
//     setGain(ArenaCameraBaseNode::AutoGainMode::Continuous);
//   } else {
//     setGain(ArenaCameraBaseNode::AutoGainMode::Off, config.gain);
//   }
//   // }

//   arena_camera_parameter_set_.gamma_ = config.gamma;
//   setGamma(arena_camera_parameter_set_.gamma_);

//   if ((level >= stop_level) && was_streaming) {
//     startStreaming();
//   }

//   // Save config
//   previous_config_ = config;
// }

// //------------------------------------------------------------------------
// //  ROS Disagnostics callbacks
// //

// void ArenaCameraBaseNode::create_diagnostics(
//     diagnostic_updater::DiagnosticStatusWrapper &stat) {}

// void ArenaCameraBaseNode::create_camera_info_diagnostics(
//     diagnostic_updater::DiagnosticStatusWrapper &stat) {
//   if (camera_info_manager_->isCalibrated()) {
//     stat.summaryf(DiagnosticStatus::OK, "Intrinsic calibration found");
//   } else {
//     stat.summaryf(DiagnosticStatus::ERROR, "No intrinsic calibration found");
//   }
// }

// void ArenaCameraBaseNode::diagnostics_timer_callback_(
//     const ros::TimerEvent &) {
//   diagnostics_updater_.update();
// }

}  // namespace arena_camera
