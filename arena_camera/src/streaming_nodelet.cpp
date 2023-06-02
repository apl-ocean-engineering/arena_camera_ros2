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

#include <pluginlib/class_list_macros.h>

#include "arena_camera/arena_camera_nodelet.h"

namespace arena_camera {

using sensor_msgs::CameraInfo;
using sensor_msgs::CameraInfoPtr;

ArenaCameraStreamingNodelet::ArenaCameraStreamingNodelet()
    : ArenaCameraNodeletBase(),
      image_callback_obj_(std::bind(&ArenaCameraStreamingNodelet::imageCallback,
                                    this, std::placeholders::_1)) {}

ArenaCameraStreamingNodelet::~ArenaCameraStreamingNodelet() {
  pDevice_->DeregisterImageCallback(&image_callback_obj_);
}

//
// Nodelet::onInit  function

void ArenaCameraStreamingNodelet::onInit() {
  // Call parent initiallzer
  ArenaCameraNodeletBase::onInit();

  try {
    pDevice_->StartStream();
  } catch (GenICam::GenericException &e) {
    NODELET_ERROR_STREAM("Error while configuring camera: \r\n"
                         << e.GetDescription());
    return;
  }

  pDevice_->RegisterImageCallback(&image_callback_obj_);

  // ros::NodeHandle nh = getNodeHandle();
  // image_timer_ =
  //     nh.createTimer(ros::Duration(1.0 / frameRate()),
  //                    &ArenaCameraStreamingNodelet::timerCallback, this);
}

// void ArenaCameraStreamingNodelet::spin() {
// void ArenaCameraStreamingNodelet::timerCallback(const ros::TimerEvent &) {
// NODELET_INFO("Timer callback");
// return;

//   if (camera_info_manager_->isCalibrated()) {
//     NODELET_INFO_ONCE("Camera is calibrated");
//   } else {
//     NODELET_INFO_ONCE("Camera not calibrated");
//   }

//   if (pDevice_->IsConnected() == false) {
//     NODELET_ERROR("Arena camera has disconnected, giving up!");
//     image_timer_.stop();
//     return;
//   }

//   if (!isSleeping() && (img_raw_pub_.getNumSubscribers())) {
//     NODELET_DEBUG("Triggering image...");
//     sendSoftwareTrigger();
//   }
// }

void ArenaCameraStreamingNodelet::imageCallback(Arena::IImage *pImage) {
  // Is this always true if the ImageCallback is called?
  Arena::IBuffer *pBuffer = dynamic_cast<Arena::IBuffer *>(pImage);
  if (pBuffer->HasImageData()) {
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
                            << pBuffer->GetSizeFilled()
                            << " , buffer size = " << pBuffer->GetSizeOfBuffer()
                            << " , expected " << payload_size);
      }

      return;
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

    // \todo{amarburg} Validate image by comparing calculated image
    //  size to actual Buffer/Image payload size
    img_raw_msg_.data.resize(data_size);
    memcpy(&img_raw_msg_.data[0], pImage->GetData(), data_size);

    if (img_raw_pub_.getNumSubscribers() > 0) {
      // Create a new cam_info-object in every frame, because it might have
      // changed due to a 'set_camera_info'-service call
      sensor_msgs::CameraInfo cam_info =
          sensor_msgs::CameraInfo(camera_info_manager_->getCameraInfo());
      cam_info.header.stamp = img_raw_msg_.header.stamp;

      img_raw_pub_.publish(img_raw_msg_, cam_info);
    }
  }
}

}  // namespace arena_camera

PLUGINLIB_EXPORT_CLASS(arena_camera::ArenaCameraStreamingNodelet,
                       nodelet::Nodelet)
