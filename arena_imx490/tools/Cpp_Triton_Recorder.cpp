//
// Based on the Cpp_Triton_Recorder example from:
// https://support.thinklucid.com/knowledgebase/tonemapping-images-tri054s-imx490/
//
/***************************************************************************************
***  Copyright (c) 2018, Lucid Vision Labs, Inc.
***  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
***  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
***  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
***  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
***  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
***  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
***  DEALINGS IN THE SOFTWARE.
***************************************************************************************/

#include <math.h>
#include <time.h>

#include <iomanip>
#include <sstream>

#include "ArenaApi.h"
#include "SaveApi.h"
#include "Utility.h"
#include "opencv2\opencv.hpp"

using namespace cv;
using namespace std;
using namespace Arena;

#define TAB1 "  "
#define TAB2 "    "

#define BUFFER_HANDLING_MODE "OldestFirst"

size_t srcWidth = 2880;
size_t srcHeight = 1860;
size_t srcBytesPerPixel = 3;
PfncFormat srcPixelFormat = PfncFormat::BayerRG24;
int tonemap_index = 2;

std::string tonemap_name = "";

std::stringstream scaled_stream;
std::stringstream tonemap_stream;

bool showTextOverlay = true;

bool isRecording = false;
uint64_t imagesRecorded = 0;

cv::ColorConversionCodes cvBayerPattern = cv::COLOR_BayerBG2BGR;

// Tonemap parameters for the OpenCV controls
float gamma = 1.0f;
float contrast = 4.0f;
float saturation = 1.0f;
float bias = 0.85f;
float scale = 0.7f;
float intensity = 0.0f;
float light_adapt = 1.0f;
float color_adapt = 0.0f;
float sigma_color = 2.0f;
float sigma_space = 2.0f;

// Slider parameters for the OpenCV controls
const int gamma_slider_max = 30;
int gamma_slider = 10;
const int contrast_slider_max = 80;
int contrast_slider = 40;
const int saturation_slider_max = 20;
int saturation_slider = 10;
const int bias_slider_max = 20;
int bias_slider = 17;
const int scale_slider_max = 10;
int scale_slider = 7;
const int intensity_slider_max = 160;
int intensity_slider = 80;
const int light_adapt_slider_max = 10;
int light_adapt_slider = 10;
const int color_adapt_slider_max = 10;
int color_adapt_slider = 0;
const int sigma_color_slider_max = 40;
int sigma_color_slider = 20;
const int sigma_space_slider_max = 40;
int sigma_space_slider = 20;

// Slider callbacks for the OpenCV controls
static void on_gamma(int, void*) { gamma = (float)gamma_slider / 10.0f; }
static void on_contrast(int, void*) {
  contrast = (float)contrast_slider / 10.0f;
}
static void on_saturation(int, void*) {
  saturation = (float)saturation_slider / 10.0f;
}
static void on_bias(int, void*) { bias = (float)bias_slider / 20.0f; }
static void on_scale(int, void*) { scale = (float)scale_slider / 10.0f; }
static void on_intensity(int, void*) {
  intensity = (float)(intensity_slider - 80) / 10.0f;
}
static void on_light_adapt(int, void*) {
  light_adapt = (float)light_adapt_slider / 10.0f;
}
static void on_color_adapt(int, void*) {
  color_adapt = (float)color_adapt_slider / 10.0f;
}
static void on_sigma_color(int, void*) {
  sigma_color = (float)sigma_color_slider / 10.0f;
}
static void on_sigma_space(int, void*) {
  sigma_space = (float)sigma_space_slider / 10.0f;
}

// Bilinear algorithm reworked from
// https://github.com/jdthomas/bayer2rgb/blob/master/bayer.c
void bayer_Bilinear_uint32(const uint32_t* bayer, uint32_t* rgb, int sx, int sy,
                           std::string tile = "BG") {
  int bayerStep = sx;
  int rgbStep = 3 * sx;
  int width = sx;
  int height = sy;

  // From https://github.com/jdthomas/bayer2rgb/blob/master/bayer.c Bayer tile
  // decision:
  //
  // int blue = tile == DC1394_COLOR_FILTER_BGGR || tile ==
  // DC1394_COLOR_FILTER_GBRG ? -1 : 1; int start_with_green = tile ==
  // DC1394_COLOR_FILTER_GBRG || tile == DC1394_COLOR_FILTER_GRBG;
  //
  // Assume Bayer tile pattern is RGGB:
  // int blue = -1
  // int start_with_green = 0;

  int blue = tile == "BG" || tile == "GB" ? -1 : 1;
  int start_with_green = tile == "GB" || tile == "GR";

  rgb += rgbStep + 3 + 1;
  height -= 2;
  width -= 2;

  for (; height--; bayer += bayerStep, rgb += rgbStep) {
    int t0, t1;
    const uint32_t* bayerEnd = bayer + width;

    if (start_with_green) {
      /* OpenCV has a bug in the next line, which was
      t0 = (bayer[0] + bayer[bayerStep * 2] + 1) >> 1; */
      t0 = (bayer[1] + bayer[bayerStep * 2 + 1] + 1) >> 1;
      t1 = (bayer[bayerStep] + bayer[bayerStep + 2] + 1) >> 1;
      rgb[-blue] = (uint32_t)t0;
      rgb[0] = bayer[bayerStep + 1];
      rgb[blue] = (uint32_t)t1;
      bayer++;
      rgb += 3;
    }

    if (blue > 0) {
      for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
        t0 = (bayer[0] + bayer[2] + bayer[bayerStep * 2] +
              bayer[bayerStep * 2 + 2] + 2) >>
             2;
        t1 = (bayer[1] + bayer[bayerStep] + bayer[bayerStep + 2] +
              bayer[bayerStep * 2 + 1] + 2) >>
             2;
        rgb[-1] = (uint32_t)t0;
        rgb[0] = (uint32_t)t1;
        rgb[1] = bayer[bayerStep + 1];

        t0 = (bayer[2] + bayer[bayerStep * 2 + 2] + 1) >> 1;
        t1 = (bayer[bayerStep + 1] + bayer[bayerStep + 3] + 1) >> 1;
        rgb[2] = (uint32_t)t0;
        rgb[3] = bayer[bayerStep + 2];
        rgb[4] = (uint32_t)t1;
      }
    } else {
      for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
        t0 = (bayer[0] + bayer[2] + bayer[bayerStep * 2] +
              bayer[bayerStep * 2 + 2] + 2) >>
             2;
        t1 = (bayer[1] + bayer[bayerStep] + bayer[bayerStep + 2] +
              bayer[bayerStep * 2 + 1] + 2) >>
             2;
        rgb[1] = (uint32_t)t0;
        rgb[0] = (uint32_t)t1;
        rgb[-1] = bayer[bayerStep + 1];

        t0 = (bayer[2] + bayer[bayerStep * 2 + 2] + 1) >> 1;
        t1 = (bayer[bayerStep + 1] + bayer[bayerStep + 3] + 1) >> 1;
        rgb[4] = (uint32_t)t0;
        rgb[3] = bayer[bayerStep + 2];
        rgb[2] = (uint32_t)t1;
      }
    }

    if (bayer < bayerEnd) {
      t0 = (bayer[0] + bayer[2] + bayer[bayerStep * 2] +
            bayer[bayerStep * 2 + 2] + 2) >>
           2;
      t1 = (bayer[1] + bayer[bayerStep] + bayer[bayerStep + 2] +
            bayer[bayerStep * 2 + 1] + 2) >>
           2;
      rgb[-blue] = (uint32_t)t0;
      rgb[0] = (uint32_t)t1;
      rgb[blue] = bayer[bayerStep + 1];
      bayer++;
      rgb += 3;
    }

    bayer -= width;
    rgb -= width * 3;

    blue = -blue;
    start_with_green = !start_with_green;
  }
}

cv::Mat LoadImageIntoOpenCV(Arena::IImage* pImage, size_t imgWidth = srcWidth,
                            size_t imgHeight = srcHeight,
                            size_t bytesPerPixel = srcBytesPerPixel,
                            std::string tile = "BG") {
  const uint8_t* input_data;
  size_t n_pixels = imgWidth * imgHeight;

  input_data = pImage->GetData();
  uint32_t* bayer_data = new uint32_t[imgWidth * imgHeight];

  // std::cout << "Unpack 3 bytes into 1 output pixel" << std::endl;
  for (size_t i = 0; i < n_pixels; i++) {
    if (bytesPerPixel == 3) {
      // Unpack 3 bytes into 1 output pixel
      bayer_data[i] = ((uint32_t)input_data[0]) +
                      (((uint32_t)input_data[1]) << 8) +
                      (((uint32_t)input_data[2]) << 16);
      input_data += 3;
    } else if (bytesPerPixel == 2) {
      bayer_data[i] =
          ((uint32_t)input_data[0]) + (((uint32_t)input_data[1]) << 8);
      input_data += 2;
    } else if (bytesPerPixel == 1) {
      bayer_data[i] = ((uint32_t)input_data[0]);
      input_data += 3;
    }
  }

  uint32_t* rgb_data = new uint32_t[imgWidth * imgHeight * 3];
  bayer_Bilinear_uint32(bayer_data, rgb_data, static_cast<int>(imgWidth),
                        static_cast<int>(imgHeight), tile);

  Mat image_s32_rgb(static_cast<int>(imgHeight), static_cast<int>(imgWidth),
                    CV_32SC3);
  image_s32_rgb.data = (uchar*)rgb_data;

  // Convert into 32-bit float for OpenCV
  Mat image_f32_rgb(static_cast<int>(imgHeight), static_cast<int>(imgWidth),
                    CV_32FC3, Scalar::all(0));
  image_s32_rgb.convertTo(image_f32_rgb, CV_32FC3);

  image_s32_rgb.release();
  delete[] bayer_data;
  delete[] rgb_data;

  return image_f32_rgb;
}

cv::Mat ProcessImageInOpenCV(cv::Mat input, float gamma = 1.0f) {
  Mat tonemap_output;

  std::string newfilename;

  switch (tonemap_index) {
    case 0: {
      // std::cout << "Using Tonemap" << std::endl;
      tonemap_name = "Tonemap";
      Ptr<Tonemap> tonemap = createTonemap(gamma);
      tonemap->process(input, tonemap_output);
      break;
    }
    case 1: {
      // std::cout << "Using TonemapDrago" << std::endl;
      tonemap_name = "TonemapDrago";
      Ptr<TonemapDrago> tonemap_drago = createTonemapDrago(gamma);
      tonemap_drago->process(input, tonemap_output);
      break;
    }
    case 2: {
      // std::cout << "Using TonemapDurand" << std::endl;
      tonemap_name = "TonemapDurand";
      Ptr<TonemapDurand> tonemap_durand = createTonemapDurand(gamma);
      tonemap_durand->process(input, tonemap_output);
      break;
    }
    case 3: {
      // std::cout << "Using TonemapMantiuk" << std::endl;
      tonemap_name = "TonemapMantiuk";
      Ptr<TonemapMantiuk> tonemap_mantiuk = createTonemapMantiuk();
      tonemap_mantiuk->process(input, tonemap_output);
      break;
    }
    case 4: {
      // std::cout << "Using TonemapReinhard" << std::endl;
      tonemap_name = "TonemapReinhard";
      Ptr<TonemapReinhard> tonemap_reinhard = createTonemapReinhard();
      tonemap_reinhard->process(input, tonemap_output);
      break;
    }
  }

  return tonemap_output;
}

void GrabLoop(Arena::IDevice* pDevice, GenICam::gcstring pixelFormat) {
  Arena::IImage* pImage = nullptr;

  Mat img;
  Mat processed_img;
  Mat display_img;
  cv::Size cv_windowSize = cv::Size(1440, 928);

  int key = -1;
  uint64_t frameId = 0;
  uint64_t imgWidth = 0;
  uint64_t imgHeight = 0;
  size_t imgBpp = 8;
  size_t imgBytesPerPixel = 1;

  std::string window_scaled_title = "Scaled Image";
  std::string window_title = "OpenCV Tonemapping";
  std::string window_control = "OpenCV Tonemapping Control Panel";

  Arena::IImage* pScaled = nullptr;
  Mat mScaled;
  Mat mScaledProcessed;

  Save::VideoRecorder recorder;
  Save::VideoParams params;

  double bitRangeMax = 23.0;
  double bpp = 23.0;
  double offset = 23.0;
  double bitRangeHigh = 23.0;
  double bitRangeLow = 16.0;

  if (pixelFormat == "BayerRG8" || pixelFormat == "BayerGR8" ||
      pixelFormat == "BayerGB8" || pixelFormat == "BayerBG8" ||
      pixelFormat == "Mono8") {
    bitRangeMax = 7.0;
    bpp = 7.0;
    offset = 7.0;
    bitRangeHigh = 7.0;
    bitRangeLow = 0.0;

    imgBytesPerPixel = 1;
  } else if (pixelFormat == "BayerRG12p" || pixelFormat == "BayerRG12Packed" ||
             pixelFormat == "BayerGR12p" || pixelFormat == "BayerGR12Packed" ||
             pixelFormat == "BayerGB12p" || pixelFormat == "BayerGB12Packed" ||
             pixelFormat == "BayerBG12p" || pixelFormat == "BayerBG12Packed" ||
             pixelFormat == "Mono12p" || pixelFormat == "Mono12Packed") {
    bitRangeMax = 11.0;
    bpp = 11.0;
    offset = 11.0;
    bitRangeHigh = 11.0;
    bitRangeLow = 4.0;
  } else if (pixelFormat == "BayerRG12" || pixelFormat == "BayerRG16" ||
             pixelFormat == "BayerGR12" || pixelFormat == "BayerGR16" ||
             pixelFormat == "BayerGB12" || pixelFormat == "BayerGB16" ||
             pixelFormat == "BayerBG12" || pixelFormat == "BayerBG16" ||
             pixelFormat == "Mono12" || pixelFormat == "Mono16") {
    bitRangeMax = 15.0;
    bpp = 15.0;
    offset = 15.0;
    bitRangeHigh = 15.0;
    bitRangeLow = 8.0;

    imgBytesPerPixel = 2;
  } else if (pixelFormat == "BayerRG24" || pixelFormat == "BayerGR24" ||
             pixelFormat == "BayerGB24" || pixelFormat == "BayerGB24" ||
             pixelFormat == "Mono24") {
    bitRangeMax = 23.0;
    bpp = 23.0;
    offset = 23.0;
    bitRangeHigh = 23.0;
    bitRangeLow = 16.0;

    imgBytesPerPixel = 3;
  }

  // This identifies the color processing mode depending on the pixel format
  std::string bayerPatternString = pixelFormat.substr(0, 7);
  std::string cvTile = "BG";

  if (bayerPatternString == "BayerRG") {
    cvBayerPattern = cv::COLOR_BayerBG2BGR;
    cvTile = "BG";
  } else if (bayerPatternString == "BayerGR") {
    cvBayerPattern = cv::COLOR_BayerGB2BGR;
    cvTile = "GB";
  } else if (bayerPatternString == "BayerGB") {
    cvBayerPattern = cv::COLOR_BayerGR2BGR;
    cvTile = "GR";
  } else if (bayerPatternString == "BayerBG") {
    cvBayerPattern = cv::COLOR_BayerRG2BGR;
    cvTile = "RG";
  } else {
    // Assume Mono pixel format
    cvBayerPattern = cv::COLOR_BayerBG2GRAY;
  }

  GenICam::gcstring modelName = Arena::GetNodeValue<GenICam::gcstring>(
      pDevice->GetNodeMap(), "DeviceModelName");

  time_t rawtime;
  struct tm timeinfo;

  std::ostringstream logFiletime;
  std::string recorder_filename = "";

  std::ostringstream fps;
  fps.str("");
  fps << std::fixed << std::setprecision(2)
      << Arena::GetNodeValue<double>(pDevice->GetNodeMap(),
                                     "AcquisitionFrameRate");

  pDevice->StartStream(20);

  while (true) {
    pImage = pDevice->GetImage(5000);

    if (!pImage->IsIncomplete()) {
      frameId = pImage->GetFrameId();
      imgWidth = pImage->GetWidth();
      imgHeight = pImage->GetHeight();
      imgBpp = pImage->GetBitsPerPixel();
      GenICam::gcstring imgPixelFormat =
          GetPixelFormatName(static_cast<PfncFormat>(pImage->GetPixelFormat()));

      // std::cout << "Got FrameId " << frameId << std::endl;
      if (isRecording == false) {
        if (imgBpp > 8) {
          img = LoadImageIntoOpenCV(pImage, imgWidth, imgHeight,
                                    imgBytesPerPixel, cvTile);
          processed_img = ProcessImageInOpenCV(img, gamma);
          processed_img.convertTo(display_img, CV_8UC3, 255);

          // Scaled 8-bits
          pScaled =
              Arena::ImageFactory::SelectBitsAndScale(pImage, 8, bpp - offset);
          mScaled = cv::Mat((int)imgHeight, (int)imgWidth, CV_8UC1,
                            (void*)pScaled->GetData());
          cv::cvtColor(mScaled, mScaledProcessed, cvBayerPattern);
          Arena::ImageFactory::Destroy(pScaled);
        } else {
          mScaled = cv::Mat((int)pImage->GetHeight(), (int)pImage->GetWidth(),
                            CV_8UC1, (void*)pImage->GetData());
          cv::cvtColor(mScaled, mScaledProcessed, cvBayerPattern);
        }

        tonemap_stream.str("");
        switch (tonemap_index) {
          case 0: {
            tonemap_stream << tonemap_name << "(" << std::fixed
                           << std::setprecision(1) << gamma << ")";
            break;
          }
          case 1: {
            tonemap_stream << tonemap_name << "(" << std::fixed
                           << std::setprecision(1) << gamma << " " << saturation
                           << " " << std::setprecision(2) << bias << ")";
            break;
          }
          case 2: {
            tonemap_stream << tonemap_name << "(" << std::fixed
                           << std::setprecision(1) << gamma << " " << contrast
                           << " " << saturation << " " << sigma_space << " "
                           << sigma_color << ")";
            break;
          }
          case 3: {
            tonemap_stream << tonemap_name << "(" << std::fixed
                           << std::setprecision(1) << gamma << " " << scale
                           << " " << saturation << ")";
            break;
          }
          case 4: {
            tonemap_stream << tonemap_name << "(" << std::fixed
                           << std::setprecision(1) << gamma << " " << intensity
                           << " " << light_adapt << " " << color_adapt << ")";
            break;
          }
        }

        scaled_stream.str("");
        scaled_stream << "Bits " << std::fixed << std::setprecision(2)
                      << bitRangeHigh << " - " << bitRangeLow;

        scaled_stream.str("");
        scaled_stream << "Bits " << std::fixed << std::setprecision(2)
                      << bitRangeHigh << " - " << bitRangeLow;
        // std::cout << scaled_stream.str() << std::endl;

        if (showTextOverlay == true) {
          cv::putText(display_img, tonemap_stream.str(),
                      cv::Point(10, 60),         // Coordinates
                      cv::FONT_HERSHEY_SIMPLEX,  // Font
                      1.5,                       // Scale
                      cv::Scalar(0, 100, 0),     // BGR Color
                      4,                         // Line Thickness
                      cv::LINE_AA);              // Anti-alias

          cv::putText(mScaledProcessed, scaled_stream.str(),
                      cv::Point(10, 60),         // Coordinates
                      cv::FONT_HERSHEY_SIMPLEX,  // Font
                      1.5,                       // Scale
                      cv::Scalar(0, 100, 0),     // BGR Color
                      4,                         // Line Thickness
                      cv::LINE_AA);              // Anti-alias
        }

        cv::namedWindow(window_scaled_title, cv::WINDOW_NORMAL);
        cv::resizeWindow(window_scaled_title, cv_windowSize);

        // OpenCV Control panel
        cv::namedWindow(window_control, cv::WINDOW_NORMAL);

        switch (tonemap_index) {
          case 0: {
            cv::createTrackbar("Gamma", window_control, &gamma_slider,
                               gamma_slider_max, on_gamma);
            on_gamma(gamma_slider, 0);
            break;
          }
          case 1: {
            cv::createTrackbar("Gamma", window_control, &gamma_slider,
                               gamma_slider_max, on_gamma);
            on_gamma(gamma_slider, 0);
            cv::createTrackbar("Saturation", window_control, &saturation_slider,
                               saturation_slider_max, on_saturation);
            on_saturation(saturation_slider, 0);
            cv::createTrackbar("Bias", window_control, &bias_slider,
                               bias_slider_max, on_bias);
            on_bias(bias_slider, 0);

            break;
          }
          case 2: {
            cv::createTrackbar("Gamma", window_control, &gamma_slider,
                               gamma_slider_max, on_gamma);
            on_gamma(gamma_slider, 0);
            cv::createTrackbar("Contrast", window_control, &contrast_slider,
                               contrast_slider_max, on_contrast);
            on_contrast(contrast_slider, 0);
            cv::createTrackbar("Saturation", window_control, &saturation_slider,
                               saturation_slider_max, on_saturation);
            on_saturation(saturation_slider, 0);
            cv::createTrackbar("Sigma Space", window_control,
                               &sigma_space_slider, sigma_space_slider_max,
                               on_sigma_space);
            on_sigma_space(sigma_space_slider, 0);
            cv::createTrackbar("Sigma Color", window_control,
                               &sigma_color_slider, sigma_color_slider_max,
                               on_sigma_color);
            on_sigma_color(sigma_color_slider, 0);

            break;
          }
          case 3: {
            cv::createTrackbar("Gamma", window_control, &gamma_slider,
                               gamma_slider_max, on_gamma);
            on_gamma(gamma_slider, 0);
            cv::createTrackbar("Scale", window_control, &scale_slider,
                               scale_slider_max, on_scale);
            on_scale(scale_slider, 0);
            cv::createTrackbar("Saturation", window_control, &saturation_slider,
                               saturation_slider_max, on_saturation);
            on_saturation(saturation_slider, 0);

            break;
          }
          case 4: {
            cv::createTrackbar("Gamma", window_control, &gamma_slider,
                               gamma_slider_max, on_gamma);
            on_gamma(gamma_slider, 0);
            cv::createTrackbar("Intensity", window_control, &intensity_slider,
                               intensity_slider_max, on_intensity);
            on_intensity(intensity_slider, 0);
            cv::createTrackbar("Light Adapt", window_control,
                               &light_adapt_slider, light_adapt_slider_max,
                               on_light_adapt);
            on_light_adapt(light_adapt_slider, 0);
            cv::createTrackbar("Color Adapt", window_control,
                               &color_adapt_slider, color_adapt_slider_max,
                               on_color_adapt);
            on_color_adapt(color_adapt_slider, 0);

            break;
          }
        }

        cv::imshow(window_scaled_title, mScaledProcessed);
        cv::imshow(window_title, display_img);

        cv::imshow(window_scaled_title, mScaledProcessed);
      } else {
        recorder.AppendImage(pImage->GetData());
        imagesRecorded++;
        display_img = cv::Mat((int)imgHeight, (int)imgWidth, CV_8UC1);
        display_img = 0;

        scaled_stream.str("");
        scaled_stream << "Recorded " << imagesRecorded << " images";
        std::cout << scaled_stream.str() << std::endl;

        cv::putText(display_img, scaled_stream.str(),
                    cv::Point(10, 60),          // Coordinates
                    cv::FONT_HERSHEY_SIMPLEX,   // Font
                    2,                          // Scale
                    cv::Scalar(255, 255, 255),  // BGR Color
                    5,                          // Line Thickness
                    cv::LINE_AA);               // Anti-alias

        cv::namedWindow(window_scaled_title, cv::WINDOW_NORMAL);
        cv::resizeWindow(window_scaled_title, cv_windowSize);

        cv::imshow(window_scaled_title, display_img);
      }

      key = cv::waitKey(1) & 0xFF;
      if (key == 'q') {
        break;
      }

      if (key == 'r') {
        isRecording = !isRecording;

        imagesRecorded = 0;

        if (isRecording == true) {
          cv::destroyWindow(window_control);
          cv::destroyWindow(window_title);

          time(&rawtime);
          localtime_s(&timeinfo, &rawtime);

          params.SetWidth(imgWidth);
          params.SetHeight(imgHeight);
          recorder.SetParams(params);
          recorder.SetRaw(pImage->GetPixelFormat());

          logFiletime.str("");
          logFiletime << std::to_string(1900 + timeinfo.tm_year) << std::setw(2)
                      << std::setfill('0')
                      << std::to_string(1 + timeinfo.tm_mon) << std::setw(2)
                      << std::setfill('0') << std::to_string(timeinfo.tm_mday)
                      << "_" << std::setw(2) << std::setfill('0')
                      << std::to_string(timeinfo.tm_hour) << std::setw(2)
                      << std::setfill('0') << std::to_string(timeinfo.tm_min)
                      << std::setw(2) << std::setfill('0')
                      << std::to_string(timeinfo.tm_sec);

          recorder_filename = logFiletime.str() + "_" + modelName + "_" +
                              imgPixelFormat + +"_" + std::to_string(imgWidth) +
                              "x" + std::to_string(imgHeight) + "_" +
                              fps.str() + "Hz.raw";
          std::cout << "Recording " << recorder_filename.c_str() << std::endl;
          recorder.SetFileNamePattern(recorder_filename.c_str());

          recorder.Open();
        } else {
          recorder.Close();
        }
      }

      if (key == 't') {
        tonemap_index++;
        if (tonemap_index > 4) {
          tonemap_index = 0;
        }
        destroyWindow(window_control);
      }

      if (key == 'o') {
        showTextOverlay = !showTextOverlay;
      }

      // Bit range for scaled image
      if (imgBpp > 8) {
        if (key == '.') {
          offset = offset + 1.0;
          bitRangeHigh = bitRangeHigh + 1;
          bitRangeLow = bitRangeLow + 1;

          if (offset > bitRangeMax) {
            bitRangeHigh = 7.0;
            bitRangeLow = 0.0;
            offset = 7.0;
          }
        }

        // Bit range for scaled image
        if (key == ',') {
          offset = offset - 1.0;
          bitRangeHigh = bitRangeHigh - 1;
          bitRangeLow = bitRangeLow - 1;

          if (offset < 7.0) {
            bitRangeHigh = bitRangeMax;
            bitRangeLow = bitRangeMax - 8.0 + 1.0;
            offset = bitRangeMax;
          }
        }
      }

      pDevice->RequeueBuffer(pImage);
    }
  }

  cv::destroyAllWindows();
  pDevice->StopStream();
}

int main(int argc, char** argv) {
  // flag to track when an exception has been thrown
  bool exceptionThrown = false;

  std::cout << "Cpp_Triton_Recorder\n";

  Arena::IDevice* pDevice = nullptr;

  try {
    // prepare example acquisition
    Arena::ISystem* pSystem = Arena::OpenSystem();

    pSystem->UpdateDevices(100);
    std::vector<Arena::DeviceInfo> deviceInfos = pSystem->GetDevices();
    if (deviceInfos.size() == 0) {
      std::cout << "\nNo camera connected\nPress enter to complete\n";
      std::getchar();
      return 0;
    }

    size_t numDevices = deviceInfos.size();
    if (numDevices == 0) {
      std::cout << "No camera connected. Press enter to exit." << std::endl;
      std::getchar();
      return 0;
    }

    std::cout << "Choose camera:\n";
    size_t x = 0;
    for (x; x < numDevices; x++) {
      std::cout << x << ": " << deviceInfos[x].ModelName()
                << ", sn: " << deviceInfos[x].SerialNumber()
                << ", fw: " << deviceInfos[x].DeviceVersion() << "\n";
    }

    size_t camera = 0;
    std::cin >> camera;

    if (!std::cin.good()) {
      camera = 0;
    }

    if (camera > (numDevices - 1) || camera < 0) {
      camera = 0;
    }
    std::cin.clear();
    std::cin.ignore(INT_MAX, '\n');

    pDevice = pSystem->CreateDevice(deviceInfos[camera]);

    // enable stream auto negotiate packet size
    Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(),
                              "StreamAutoNegotiatePacketSize", true);

    // enable stream packet resend
    Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(),
                              "StreamPacketResendEnable", true);

    Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetTLStreamNodeMap(),
                                           "StreamBufferHandlingMode",
                                           BUFFER_HANDLING_MODE);
    std::cout << "StreamBufferHandlingMode = " << BUFFER_HANDLING_MODE
              << std::endl;

    std::cout << "Set PixelFormat = BayerRG24" << std::endl;
    Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "PixelFormat",
                                           "BayerRG24");

    GenICam::gcstring pixelFormat = Arena::GetNodeValue<GenICam::gcstring>(
        pDevice->GetNodeMap(), "PixelFormat");
    std::cout << "PixelFormat is " << pixelFormat << std::endl;

    std::cout << "Set ColorTransformationEnable = False" << std::endl;
    Arena::SetNodeValue<bool>(pDevice->GetNodeMap(),
                              "ColorTransformationEnable", false);

    std::cout << "Set LUTEnable = False" << std::endl;
    Arena::SetNodeValue<bool>(pDevice->GetNodeMap(), "LUTEnable", false);

    GrabLoop(pDevice, pixelFormat);

    Arena::CloseSystem(pSystem);
  } catch (GenICam::GenericException& ge) {
    std::cout << "\nGenICam exception thrown: " << ge.what() << "\n";
    exceptionThrown = true;
  } catch (std::exception& ex) {
    std::cout << "\nStandard exception thrown: " << ex.what() << "\n";
    exceptionThrown = true;
  } catch (...) {
    std::cout << "\nUnexpected exception thrown\n";
    exceptionThrown = true;
  }

  std::cout << "Press enter to complete\n";
  // std::getchar();

  if (exceptionThrown)
    return -1;
  else
    return 0;
}
