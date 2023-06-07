/******************************************************************************
 * Software License Agreement (BSD 3-Clause License)
 *
 * Copyright (C) 2023 University of Washington. All rights reserved.
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

#include "arena_imx490/arena_imx490_nodelet.h"

#include <pluginlib/class_list_macros.h>

namespace arena_imx490 {

using arena_camera::ArenaCameraStreamingNodelet;

ArenaIMX490Nodelet::ArenaIMX490Nodelet() : ArenaCameraStreamingNodelet() {}

ArenaIMX490Nodelet::~ArenaIMX490Nodelet() {}

void ArenaIMX490Nodelet::onInit() {
  // Skip the ArenaCameraStreamingNodelet version and go straight
  // to the parent
  ArenaCameraNodeletBase::onInit();

  if (!configureHDR()) {
    NODELET_ERROR(
        "Unable to configure HDR features!  Is this an IMX490-based camera?");
  }

  try {
    pDevice_->StartStream();
  } catch (GenICam::GenericException &e) {
    NODELET_ERROR_STREAM("Error while configuring camera: \r\n"
                         << e.GetDescription());
    return;
  }

  pDevice_->RegisterImageCallback(&image_callback_obj_);
}

bool ArenaIMX490Nodelet::configureHDR() {
  auto pNodeMap = pDevice_->GetNodeMap();

  NODELET_INFO("Enabling HDR mode in camera");

  try {
    // GenApi::CStringPtr pHDROutput = pNodeMap->GetNode("HDROutput");
    // if (GenApi::IsWritable(pHDROutput)) {
    //   Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "HDROutput", "HDR");
    // }

    // // Enable HDR image enhancement
    Arena::SetNodeValue<bool>(pNodeMap, "HDRImageEnhancementEnable", true);
    Arena::SetNodeValue<bool>(pNodeMap, "HDRTuningEnable", false);

    Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "HDROutput", "HDR");

  } catch (GenICam::GenericException &e) {
    NODELET_ERROR_STREAM(
        "Error while configuring camera: " << e.GetDescription());
    return false;
  }

  return true;
}

}  // namespace arena_imx490

PLUGINLIB_EXPORT_CLASS(arena_imx490::ArenaIMX490Nodelet, nodelet::Nodelet)
