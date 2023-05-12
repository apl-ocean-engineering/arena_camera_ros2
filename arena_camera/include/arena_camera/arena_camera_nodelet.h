/******************************************************************************
 * Software License Agreement (BSD 3-Clause License)
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

// STD
#include <memory>
#include <string>

// ROS sys dep
#include <boost/thread.hpp>

// ROS
#include <actionlib/server/simple_action_server.h>
#include <camera_control_msgs/GrabImagesAction.h>
#include <camera_control_msgs/SetBinning.h>
#include <camera_control_msgs/SetBool.h>
#include <camera_control_msgs/SetBrightness.h>
#include <camera_control_msgs/SetExposure.h>
#include <camera_control_msgs/SetGain.h>
#include <camera_control_msgs/SetGamma.h>
#include <camera_control_msgs/SetROI.h>
#include <camera_control_msgs/SetSleeping.h>
#include <camera_info_manager/camera_info_manager.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

// Arena
#include <ArenaApi.h>
// #include <arena_camera/arena_camera.h>
#include <arena_camera/arena_camera_parameter.h>

namespace arena_camera {
typedef actionlib::SimpleActionServer<camera_control_msgs::GrabImagesAction>
    GrabImagesAS;

/// Base class for both types of nodelets
class ArenaCameraNodeletBase : public nodelet::Nodelet {
 public:
  ArenaCameraNodeletBase();
  virtual ~ArenaCameraNodeletBase();

  /**
   * initialize the camera and the ros node.
   * calls ros::shutdown if an error occurs.
   */
  void onInit() override;

  /**
   * Getter for the frame rate set by the launch script or from the ros
   * parameter server
   * @return the desired frame rate.
   */
  const double& frameRate() const;

  /**
   * Getter for the tf frame.
   * @return the camera frame.
   */
  const std::string& cameraFrame() const;

 protected:
  /**
   * Creates the camera instance either by UserDeviceId, SerialNumber,
   * or taking the first auto-detected camera.
   * @return false if an error occurred
   */
  bool registerCameraByUserId(const std::string& device_id);
  bool registerCameraBySerialNumber(const std::string& serial_number);
  bool registerCameraByAuto();

  /**
   * Start the camera and initialize the messages
   * @return
   */
  bool startGrabbing();

  bool setImageEncoding(const std::string& ros_encoding);

  /**
   * Returns the total number of subscribers on any advertised image topic.
   */
  uint32_t getNumSubscribers() const;

  virtual bool triggerImage();

  /**
   * Grabs an image and stores the image in img_raw_msg_
   * @return false if an error occurred.
   */
  virtual bool grabImage();

  /**
   * Fills the ros CameraInfo-Object with the image dimensions
   */
  virtual void setupInitialCameraInfo(sensor_msgs::CameraInfo& cam_info_msg);

  /**
   * Update area of interest in the camera image
   * @param target_roi the target roi
   * @param reached_roi the roi that could be set
   * @return true if the targeted roi could be reached
   */
  bool setROI(const sensor_msgs::RegionOfInterest target_roi,
              sensor_msgs::RegionOfInterest& reached_roi);

  /**
   * Update the horizontal binning_x factor to get downsampled images
   * @param target_binning_x the target horizontal binning_x factor
   * @param reached_binning_x the horizontal binning_x factor that could be
   *        reached
   * @return true if the targeted binning could be reached
   */
  bool setBinningX(const size_t& target_binning_x, size_t& reached_binning_x);

  /**
   * Update the vertical binning_y factor to get downsampled images
   * @param target_binning_y the target vertical binning_y factor
   * @param reached_binning_y the vertical binning_y factor that could be
   *        reached
   * @return true if the targeted binning could be reached
   */
  bool setBinningY(const size_t& target_binning_y, size_t& reached_binning_y);

  /**
   * Service callback for updating the cameras binning setting
   * @param req request
   * @param res response
   * @return true on success
   */
  bool setBinningCallback(camera_control_msgs::SetBinning::Request& req,
                          camera_control_msgs::SetBinning::Response& res);

  /**
   * Service callback for updating the cameras roi setting
   * @param req request
   * @param res response
   * @return true on success
   */
  bool setROICallback(camera_control_msgs::SetROI::Request& req,
                      camera_control_msgs::SetROI::Response& res);

  bool setExposureValue(const float& target_exposure, float& reached_exposure);

  /**
   * Update the exposure value on the camera
   * @param target_exposure the targeted exposure
   * @param reached_exposure the exposure that could be reached
   * @return true if the targeted exposure could be reached
   */
  bool setExposure(const float& target_exposure, float& reached_exposure);

  /**
   * Service callback for setting the exposure
   * @param req request
   * @param res response
   * @return true on success
   */
  bool setExposureCallback(camera_control_msgs::SetExposure::Request& req,
                           camera_control_msgs::SetExposure::Response& res);

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
  bool setBrightness(const int& target_brightness, int& reached_brightness,
                     const bool& exposure_auto, const bool& gain_auto);

  /**
   * Service callback for setting the brightness
   * @param req request
   * @param res response
   * @return true on success
   */
  bool setBrightnessCallback(camera_control_msgs::SetBrightness::Request& req,
                             camera_control_msgs::SetBrightness::Response& res);

  bool setGainValue(const float& target_gain, float& reached_gain);

  /**
   * Update the gain from the camera to a target gain in percent
   * @param target_gain the targeted gain in percent
   * @param reached_gain the gain that could be reached
   * @return true if the targeted gain could be reached
   */
  bool setGain(const float& target_gain, float& reached_gain);

  /**
   * Service callback for setting the desired gain in percent
   * @param req request
   * @param res response
   * @return true on success
   */
  bool setGainCallback(camera_control_msgs::SetGain::Request& req,
                       camera_control_msgs::SetGain::Response& res);

  bool setGammaValue(const float& target_gamma, float& reached_gamma);

  /**
   * Update the gamma from the camera to a target gamma correction value
   * @param target_gamma the targeted gamma
   * @param reached_gamma the gamma that could be reached
   * @return true if the targeted gamma could be reached
   */
  bool setGamma(const float& target_gamma, float& reached_gamma);

  /**
   * Service callback for setting the desired gamma correction value
   * @param req request
   * @param res response
   * @return true on success
   */
  bool setGammaCallback(camera_control_msgs::SetGamma::Request& req,
                        camera_control_msgs::SetGamma::Response& res);

  /**
   * Callback that puts the camera to sleep
   * @param req request
   * @param res response
   * @return true on success
   */
  bool setSleepingCallback(camera_control_msgs::SetSleeping::Request& req,
                           camera_control_msgs::SetSleeping::Response& res);

  /**
   * Returns true if the camera was put into sleep mode
   * @return true if in sleep mode
   */
  bool isSleeping() const { return is_sleeping_; }

  /**
   * Generates the subset of points on which the brightness search will be
   * executed in order to speed it up. The subset are the indices of the
   * one-dimensional image_raw data vector. The base generation is done in a
   * recursive manner, by calling genSamplingIndicesRec
   * @return indices describing the subset of points
   */
  void setupSamplingIndices(std::vector<std::size_t>& indices, std::size_t rows,
                            std::size_t cols, int downsampling_factor);

  /**
   * This function will recursively be called from above setupSamplingIndices()
   * to generate the indices of pixels given the actual ROI.
   * @return indices describing the subset of points
   */
  void genSamplingIndicesRec(std::vector<std::size_t>& indices,
                             const std::size_t& min_window_height,
                             const cv::Point2i& start, const cv::Point2i& end);

  /**
   * Calculates the mean brightness of the image based on the subset indices
   * @return the mean brightness of the image
   */
  float calcCurrentBrightness();

  void initCalibrationMatrices(sensor_msgs::CameraInfo& info, const cv::Mat& D,
                               const cv::Mat& K);

  /**
   * Callback that sets the digital user output
   * @param output_id the ID of the user output to set
   * @param req request
   * @param res response
   * @return true on success
   */
  bool setUserOutputCB(int output_id,
                       camera_control_msgs::SetBool::Request& req,
                       camera_control_msgs::SetBool::Response& res);

  /**
  * Callback that activates the digital user output to
  be used as autoflash
  * @param output_id the ID of the user output to set
  * @param req request
  * @param res response
  * @return true on success
  */
  bool setAutoflash(const int output_id,
                    camera_control_msgs::SetBool::Request& req,
                    camera_control_msgs::SetBool::Response& res);

  Arena::ISystem* pSystem_;
  Arena::IDevice* pDevice_;
  GenApi::INodeMap* pNodeMap_;

  // Hardware accessor functions
  // These might have originally been in arena_camera.h?
  sensor_msgs::RegionOfInterest currentROI();
  float currentGamma();
  int64_t currentBinningX();
  int64_t currentBinningY();
  float currentGain();
  float currentExposure();
  std::string currentROSEncoding();
  bool setBinningXValue(const size_t& target_binning_x,
                        size_t& reached_binning_x);
  bool setBinningYValue(const size_t& target_binning_y,
                        size_t& reached_binning_y);
  void disableAllRunningAutoBrightessFunctions();

  ArenaCameraParameter arena_camera_parameter_set_;
  ros::ServiceServer set_binning_srv_;
  ros::ServiceServer set_roi_srv_;
  ros::ServiceServer set_exposure_srv_;
  ros::ServiceServer set_gain_srv_;
  ros::ServiceServer set_gamma_srv_;
  ros::ServiceServer set_brightness_srv_;
  ros::ServiceServer set_sleeping_srv_;
  std::vector<ros::ServiceServer> set_user_output_srvs_;

  std::unique_ptr<image_transport::ImageTransport> it_;
  image_transport::CameraPublisher img_raw_pub_;

  image_geometry::PinholeCameraModel pinhole_model_;

  //  GrabImagesAS* grab_imgs_raw_as_;

  // Don't like using this global member
  sensor_msgs::Image img_raw_msg_;

  camera_info_manager::CameraInfoManager* camera_info_manager_;

  std::vector<std::size_t> sampling_indices_;
  std::array<float, 256> brightness_exp_lut_;

  bool is_sleeping_;

  boost::recursive_mutex device_mutex_;

  /// diagnostics:
  diagnostic_updater::Updater diagnostics_updater_;
  void diagnostics_timer_callback_(const ros::TimerEvent&);
  ros::Timer diagnostics_trigger_;
  void create_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
  void create_camera_info_diagnostics(
      diagnostic_updater::DiagnosticStatusWrapper& stat);
};

class ArenaCameraStreamingNodelet : public ArenaCameraNodeletBase {
 public:
  ArenaCameraStreamingNodelet();
  virtual ~ArenaCameraStreamingNodelet();

  /**
   * initialize the camera and the ros node.
   * calls ros::shutdown if an error occurs.
   */
  virtual void onInit();

  /**
   * Take one image
   */
  void timerCallback(const ros::TimerEvent&);

 protected:
  ros::Timer image_timer_;

  typedef std::function<void(Arena::IImage* pImage)> ImageCallback_t;

  class ImageCallback : public Arena::IImageCallback {
   public:
    ImageCallback(ImageCallback_t cb) : image_callback_(cb) {}

    ~ImageCallback() {}

    void OnImage(Arena::IImage* pImage) { image_callback_(pImage); }

   private:
    ImageCallback_t image_callback_;
  } image_callback_obj_;

  void imageCallback(Arena::IImage* pImage);
};

class ArenaCameraPolledNodelet : public ArenaCameraNodeletBase {
 public:
  ArenaCameraPolledNodelet();
  virtual ~ArenaCameraPolledNodelet();

  /**
   * initialize the camera and the ros node.
   * calls ros::shutdown if an error occurs.
   */
  virtual void onInit();

  /**
   * Callback for the grab images action
   * @param goal the goal
   */
  void grabImagesRawActionExecuteCB(
      const camera_control_msgs::GrabImagesGoal::ConstPtr& goal);

  /**
   * This function can also be called from the derived ArenaCameraOpenCV-Class
   */
  camera_control_msgs::GrabImagesResult grabImagesRaw(
      const camera_control_msgs::GrabImagesGoal::ConstPtr& goal,
      GrabImagesAS* action_server);

 protected:
  std::unique_ptr<GrabImagesAS> grab_imgs_raw_as_;
};

}  // namespace arena_camera
