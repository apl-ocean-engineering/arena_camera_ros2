/******************************************************************************
 * Software License Agreement (BSD 3-Clause License)
 *
 * Copyright (C) 2023 University of Washington. All rights reserved.
 *
 * Based on the original arena_camera_ros as follows:
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

#pragma once

#include <memory>
#include <string>

// ROS system headers
#include <camera_info_manager/camera_info_manager.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>

// LucidVision Arena SDK
#include <ArenaApi.h>

#include <imaging_msgs/msg/hdr_imaging_metadata.hpp>
#include <imaging_msgs/msg/imaging_metadata.hpp>

// Auto-generated parameters file
#include <arena_camera_parameters.hpp>

namespace arena_camera {

/// Base class for both types of Nodes
class ArenaCameraBaseNode : public rclcpp::Node {
 public:
  ArenaCameraBaseNode(const std::string &node_name,
                      const rclcpp::NodeOptions &options);
  virtual ~ArenaCameraBaseNode();

  /// Command the camera to start streaming.
  ///
  void startStreaming();

  /// Command the camera to stop streaming.
  ///
  void stopStreaming();

 protected:
  /**
   * Creates the camera instance either by UserDeviceId, SerialNumber,
   * or taking the first auto-detected camera.
   * @return false if an error occurred
   */
  bool registerCameraByUserId(const std::string &device_id);
  bool registerCameraBySerialNumber(const std::string &serial_number);
  bool registerCameraByAuto();

  /// Start the camera and initialize the messages
  /// @return true if successful, false on an error
  ///
  bool configureCamera();

  //======================================================================
  // Camera property setters
  //

  /// Getter for the tf frame.
  /// @return the camera frame.
  ///
  const std::string &cameraFrame() const { return params_.camera_frame; }

  // ~~~ Image encoding ~~~

  bool setImageEncoding(const std::string &ros_encoding);

  std::string currentROSEncoding();

  // ~~~ Frame rate ~~~

  /// Getter for the current frame rate
  /// @param frame_rate  Desired frame rate in Hz
  /// @return True on success, false on error
  ///
  bool setFrameRate(float frame_rate);

  /// Getter for the current frame rate
  /// @return the camera's current frame rate.
  ///
  double currentFrameRate() const;

  // ~~~ Target brightness ~~~

  /**
   * Sets the target brightness which is the intensity-mean over all pixels.
   * If the target exposure time is not in the range of Arena's auto target
   * brightness range the extended brightness search is started.
   * The Auto function of the Arena-API supports values from [50 - 205].
   * Using a binary search, this range will be extended up to [1 - 255].
   * @param target_brightness is the desired brightness. Range is [1...255].
   * @param current_brightness is the current brightness with the given
   * settings.
   * @param exposure_auto flag which indicates if the target_brightness
   *                      should be reached adapting the exposure time
   * @param gain_auto flag which indicates if the target_brightness should be
   *                      reached adapting the gain.
   * @return true if the brightness could be reached or false otherwise.
   */
  // bool setBrightness(const int &target_brightness, int &reached_brightness,
  //                    const bool &exposure_auto, const bool &gain_auto);

  void setTargetBrightness(unsigned int brightness);

  // ~~~ Exposure ~~~

  enum class AutoExposureMode : int { Off = 0, Once = 1, Continuous = 2 };

  // Update exposure based on arena_camera_parameter_set
  //
  //  If exp_mode == Off, exposure_ms is the **fixed exposure** set in the
  //  camera If exp_mode == Once or Continuous, exposure_ms is the **max
  //  exposure** allowed
  //                   for the auto-exposure algorithm
  bool setExposure(AutoExposureMode exp_mode, float exposure_ms);

  bool setExposure(const arena_camera::Params &params);

  float currentExposure();

  // ~~~ Gain ~~~

  enum class AutoGainMode : int { Off = 0, Once = 1, Continuous = 2 };

  ///
  /// Update the gain from the camera to a target gain in percent
  /// @param gain_mode   Request gain mode
  /// @param target_gain the targeted gain in percent.  Ignored if gain_mode
  /// isn't "Off"
  ///
  /// @return  true if the requested gain settings were accepted by the camera,
  /// false on error
  ///
  bool setGain(AutoGainMode gain_mode, float target_gain = 0.0);

  /// Convenience wrapper which reads directly from a Params struct.
  /// @param params An arena_camera::Params struct
  /// @return  true if the requested gain settings were accepted by the camera,
  /// false on error
  bool setGain(const arena_camera::Params &params);

  /// Retrieve the current  gain from the camera
  /// @return Gain
  ///
  float currentGain();

  // ~~~ Gamma ~~~

  /// Update the gamma from the camera to a target gamma correction value
  /// @param target_gamma the targeted gamma
  /// @return true if the targeted gamma could be reached
  ///
  bool setGamma(const float &target_gamma);

  /// Retrieve the current gamma from the camera
  /// @return Gamma from camera
  ///
  float currentGamma();

  // ~~~ Camera Lookup Tables ~~~

  ///  Enable/disable lookup table (LUT) in camera.
  /// @param enable Whether to enable/disable the camera LUT
  ///
  void enableLUT(bool enable);

  // ~~~ HDR metadata (IMX490 only) ~~~

  float currentHdrGain(int channel);

  float currentHdrExposure(int channel);

  /**
   * Update area of interest in the camera image
   * @param target_roi the target roi
   * @param reached_roi the roi that could be set
   * @return true if the targeted roi could be reached
   */
  // bool setROI(const sensor_msgs::RegionOfInterest target_roi,
  //             sensor_msgs::RegionOfInterest &reached_roi);

  // /**
  //  * Update the horizontal binning_x factor to get downsampled images
  //  * @param target_binning_x the target horizontal binning_x factor
  //  * @param reached_binning_x the horizontal binning_x factor that could be
  //  *        reached
  //  * @return true if the targeted binning could be reached
  //  */
  // bool setBinningX(const size_t &target_binning_x, size_t
  // &reached_binning_x);

  // /**
  //  * Update the vertical binning_y factor to get downsampled images
  //  * @param target_binning_y the target vertical binning_y factor
  //  * @param reached_binning_y the vertical binning_y factor that could be
  //  *        reached
  //  * @return true if the targeted binning could be reached
  //  */
  // bool setBinningY(const size_t &target_binning_y, size_t
  // &reached_binning_y);

  // //===== Functions for querying HDR channels (IMX490 only)

  // /**
  //  * Generates the subset of points on which the brightness search will be
  //  * executed in order to speed it up. The subset are the indices of the
  //  * one-dimensional image_raw data vector. The base generation is done in a
  //  * recursive manner, by calling genSamplingIndicesRec
  //  * @return indices describing the subset of points
  //  */
  // void setupSamplingIndices(std::vector<std::size_t> &indices, std::size_t
  // rows,
  //                           std::size_t cols, int downsampling_factor);

  // /**
  //  * This function will recursively be called from above
  //  setupSamplingIndices()
  //  * to generate the indices of pixels given the actual ROI.
  //  * @return indices describing the subset of points
  //  */
  // void genSamplingIndicesRec(std::vector<std::size_t> &indices,
  //                            const std::size_t &min_window_height,
  //                            const cv::Point2i &start, const cv::Point2i
  //                            &end);

  // /**
  //  * Calculates the mean brightness of the image based on the subset indices
  //  * @return the mean brightness of the image
  //  */
  // float calcCurrentBrightness();

 protected:
  /// @brief
  /// @param cam_info_msg
  // void initializeCameraInfo(sensor_msgs::CameraInfo &cam_info_msg);

  Arena::ISystem *pSystem_;
  Arena::IDevice *pDevice_;
  GenApi::INodeMap *pNodeMap_;

  bool is_streaming_;

  // ~~~ Publishers ~~~
  image_transport::CameraPublisher img_raw_pub_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;

  rclcpp::Publisher<imaging_msgs::msg::ImagingMetadata>::SharedPtr
      metadata_pub_;

  rclcpp::Publisher<imaging_msgs::msg::HdrImagingMetadata>::SharedPtr
      hdr_metadata_pub_;

  // ~~~ Parameters ~~
  std::shared_ptr<arena_camera::ParamListener> param_listener_;
  arena_camera::Params params_;

  rclcpp::TimerBase::SharedPtr parameter_check_timer_;
  void checkParametersCb(void);

  // Hardware accessor functions
  // These might have originally been in arena_camera.h?
  // sensor_msgs::RegionOfInterest currentROI();
  // int64_t currentBinningX();
  // int64_t currentBinningY();

  // bool setBinningXValue(const size_t &target_binning_x,
  //                       size_t &reached_binning_x);
  // bool setBinningYValue(const size_t &target_binning_y,
  //                       size_t &reached_binning_y);
  // void disableAllRunningAutoBrightessFunctions();

  // std::vector<std::size_t> sampling_indices_;
  // // std::array<float, 256> brightness_exp_lut_;

  /// diagnostics:
  std::shared_ptr<diagnostic_updater::Updater> diagnostics_updater_;

  void create_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void create_camera_info_diagnostics(
      diagnostic_updater::DiagnosticStatusWrapper &stat);
};

class ArenaCameraStreamingNode : public ArenaCameraBaseNode {
 public:
  ArenaCameraStreamingNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  virtual ~ArenaCameraStreamingNode();

  // void onInit() override;

 protected:
  typedef std::function<void(Arena::IImage *pImage)> ImageCallback_t;

  // The ArenaSDK takes a class derived from Arena::IImageCallback
  // for the new-image callback.
  //
  // This class ImageCallback meets the Arena requirement, and wraps
  // a functional callback (which can then be **back** into
  // ArenaCameraStreamingNode)
  //
  class ImageCallback : public Arena::IImageCallback {
   public:
    ImageCallback(ImageCallback_t cb) : image_callback_(cb) {}
    ~ImageCallback() {}

    void OnImage(Arena::IImage *pImage) { image_callback_(pImage); }

   private:
    ImageCallback_t image_callback_;

    ImageCallback() = delete;
    ImageCallback(const ImageCallback &) = delete;
    ImageCallback operator=(const ImageCallback &) = delete;
  };

  ImageCallback image_callback_obj_;

  void newImageCb(Arena::IImage *pImage);
};

/// ArenaCameraPolledNode is not currently implemented.
class ArenaCameraPolledNode : public ArenaCameraBaseNode {
 public:
  ArenaCameraPolledNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  virtual ~ArenaCameraPolledNode();

  /**
   * Callback for the grab images action
   * @param goal the goal
   */
  // void grabImagesRawActionExecuteCB(
  //     const camera_control_msgs::GrabImagesGoal::ConstPtr &goal);

  // /**
  //  * This function can also be called from the derived
  //  ArenaCameraOpenCV-Class
  //  */
  // camera_control_msgs::GrabImagesResult grabImagesRaw(
  //     const camera_control_msgs::GrabImagesGoal::ConstPtr &goal,
  //     GrabImagesAS *action_server);

 protected:
  // virtual bool sendSoftwareTrigger();

  // /// Grabs an image and stores the image in img_raw_msg_
  // /// @return false if an error occurred.
  // virtual bool grabImage();

  // std::unique_ptr<GrabImagesAS> grab_imgs_raw_as_;
};

}  // namespace arena_camera
