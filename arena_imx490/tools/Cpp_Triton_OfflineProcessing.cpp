//
// Based on the example Cpp_Triton_OfflineProcessing from
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

#include <sstream>
#include <thread>

#include "ArenaApi.h"
#include "SaveApi.h"
#include "Utility.h"
#include "opencv2/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/photo.hpp"

using namespace cv;
using namespace std;
using namespace Arena;

#define TAB1 "  "
#define TAB2 "    "

// Default image size
size_t srcWidth = 2880;
size_t srcHeight = 1860;
size_t srcBytesPerPixel = 3;
size_t srcBitsPerPixel = 8;
double srcFPS = 1.0;
PfncFormat srcPixelFormat = PfncFormat::BayerRG24;

int tonemap_index = 2;
std::string tonemap_name = "";
std::string filename = "";

std::stringstream tonemap_stream;
std::stringstream scaled_stream;

cv::ColorConversionCodes cvBayerPattern = cv::COLOR_BayerBG2BGR;

bool showTextOverlay = true;

FILE* fp;
bool newFrame = true;
bool outputVideo = false;

// Tonemap parameters for the OpenCV controls
float gamma_ = 1.0f;
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
static void on_gamma(int, void*) {
  gamma_ = (float)gamma_slider / 10.0f;
  newFrame = true;
}
static void on_contrast(int, void*) {
  contrast = (float)contrast_slider / 10.0f;
  newFrame = true;
}
static void on_saturation(int, void*) {
  saturation = (float)saturation_slider / 10.0f;
  newFrame = true;
}
static void on_bias(int, void*) {
  bias = (float)bias_slider / 20.0f;
  newFrame = true;
}
static void on_scale(int, void*) {
  scale = (float)scale_slider / 10.0f;
  newFrame = true;
}
static void on_intensity(int, void*) {
  intensity = (float)(intensity_slider - 80) / 10.0f;
  newFrame = true;
}
static void on_light_adapt(int, void*) {
  light_adapt = (float)light_adapt_slider / 10.0f;
  newFrame = true;
}
static void on_color_adapt(int, void*) {
  color_adapt = (float)color_adapt_slider / 10.0f;
  newFrame = true;
}
static void on_sigma_color(int, void*) {
  sigma_color = (float)sigma_color_slider / 10.0f;
  newFrame = true;
}
static void on_sigma_space(int, void*) {
  sigma_space = (float)sigma_space_slider / 10.0f;
  newFrame = true;
}

Arena::IImage* CreateArenaImage(unsigned char* buffer,
                                PfncFormat pixelFormat = PfncFormat::BayerRG24,
                                size_t bytesPerPixel = 3,
                                size_t imgWidth = srcWidth,
                                size_t imgHeight = srcHeight) {
  // std::cout << "Create Arena Image" << std::endl;
  Arena::IImage* imageFromFile =
      Arena::ImageFactory::Create(buffer, imgWidth * imgHeight * bytesPerPixel,
                                  imgWidth, imgHeight, pixelFormat);
  return imageFromFile;
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

  // std::cout << "Unpack 3 or 2 bytes into 1 output pixel" << std::endl;
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
  // std::cout << "Using gamma_ = " << gamma_ << std::endl;

  switch (tonemap_index) {
    case 0: {
      // std::cout << "Using Tonemap" << std::endl;
      tonemap_name = "Tonemap";
      Ptr<Tonemap> tonemap = createTonemap(gamma_);
      tonemap->process(input, tonemap_output);
      break;
    }
    case 1: {
      // std::cout << "Using TonemapDrago" << std::endl;
      tonemap_name = "TonemapDrago";
      Ptr<TonemapDrago> tonemap_drago =
          createTonemapDrago(gamma_, saturation, bias);
      tonemap_drago->process(input, tonemap_output);
      break;
    }
    case 2: {
      std::cout << "TonemapDurand not supported" << std::endl;

      // std::cout << "Using TonemapDurand" << std::endl;
      //  tonemap_name = "TonemapDurand";
      //  Ptr<TonemapDurand> tonemap_durand =
      //  createTonemapDurand(gamma_, contrast, saturation,
      //  sigma_space, sigma_color); tonemap_durand->process(input,
      //  tonemap_output);
      break;
    }
    case 3: {
      // std::cout << "Using TonemapMantiuk" << std::endl;
      tonemap_name = "TonemapMantiuk";
      Ptr<TonemapMantiuk> tonemap_mantiuk =
          createTonemapMantiuk(gamma_, scale, saturation);
      tonemap_mantiuk->process(input, tonemap_output);
      break;
    }
    case 4: {
      // std::cout << "Using TonemapReinhard" << std::endl;
      tonemap_name = "TonemapReinhard";
      Ptr<TonemapReinhard> tonemap_reinhard =
          createTonemapReinhard(gamma_, intensity, light_adapt, color_adapt);
      tonemap_reinhard->process(input, tonemap_output);
      break;
    }
  }

  return tonemap_output;
}

Arena::IImage* ReadImage(FILE* fp) {
  unsigned char* buffer =
      (unsigned char*)malloc(srcWidth * srcHeight * srcBytesPerPixel);
  size_t bytes_read = fread(buffer, sizeof(unsigned char),
                            srcWidth * srcHeight * srcBytesPerPixel, fp);
  std::cout << "Read " << bytes_read << " bytes" << std::endl;
  Arena::IImage* pImage = CreateArenaImage(
      buffer, srcPixelFormat, srcBytesPerPixel, srcWidth, srcHeight);
  delete[] buffer;

  newFrame = true;
  return pImage;
}

void DisplayImage(Arena::IImage* pImage, long long fSize, size_t numImages,
                  size_t imgWidth = srcWidth, size_t imgHeight = srcHeight,
                  size_t imgBytesPerPixel = srcBytesPerPixel) {
  Mat img;
  Mat processed_img;
  Mat display_img;
  cv::Size cv_windowSize = cv::Size(1440, 928);
  cv::Size cv_windowSizeHalf = cv_windowSize / 2;

  int key = -1;
  std::string window_scaled_title = "Scaled Image";
  std::string window_title = "OpenCV Tonemapping";
  std::string window_control = "OpenCV Tonemapping Control Panel";

  Arena::IImage* pScaled = nullptr;
  Mat mScaled;
  Mat mScaledProcessed;

  long imgSize = (long)imgWidth * (long)imgHeight * (long)imgBytesPerPixel;
  off_t pos = 0;
  long long posCounter = 1;

  VideoWriter video;
  int fourcc = VideoWriter::fourcc('H', '2', '6', '4');

  GenICam::gcstring pixelFormat = GetPixelFormatName(srcPixelFormat);

  // This identifies bit ranges for the input image
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
  } else if (pixelFormat == "BayerRG24" || pixelFormat == "BayerGR24" ||
             pixelFormat == "BayerGB24" || pixelFormat == "BayerGB24" ||
             pixelFormat == "Mono24") {
    bitRangeMax = 23.0;
    bpp = 23.0;
    offset = 23.0;
    bitRangeHigh = 23.0;
    bitRangeLow = 16.0;
  }

  // This identifies the color processing mode depending on the pixel format
  //
  // \todo{amarburg@uw.edu}  Awkward conversion from GenICAM::gcstring to
  // std::string.  Look for better options.
  std::string bayerPatternString(pixelFormat.substr(0, 7).c_str());
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

  bool newScaledImage = true;

  while (true) {
    if (outputVideo == false) {
      if (newFrame == true) {
        // Tonemapped image
        img = LoadImageIntoOpenCV(pImage, imgWidth, imgHeight, imgBytesPerPixel,
                                  cvTile);
        processed_img = ProcessImageInOpenCV(img, gamma_);
        processed_img.convertTo(display_img, CV_8UC3, 255);
        newScaledImage = true;
      }

      if (newScaledImage == true) {
        // Get a scaled 8-bit image from 24-bit range
        pScaled =
            Arena::ImageFactory::SelectBitsAndScale(pImage, 8, bpp - offset);
        mScaled = cv::Mat((int)pScaled->GetHeight(), (int)pScaled->GetWidth(),
                          CV_8UC1, (void*)pScaled->GetData());
        cv::cvtColor(mScaled, mScaledProcessed, cvBayerPattern);
        Arena::ImageFactory::Destroy(pScaled);
      }

      tonemap_stream.str("");
      switch (tonemap_index) {
        case 0: {
          tonemap_stream << tonemap_name << "(" << std::fixed
                         << std::setprecision(1) << gamma_ << ")";
          break;
        }
        case 1: {
          tonemap_stream << tonemap_name << "(" << std::fixed
                         << std::setprecision(1) << gamma_ << " " << saturation
                         << " " << std::setprecision(2) << bias << ")";
          break;
        }
        case 2: {
          tonemap_stream << tonemap_name << "(" << std::fixed
                         << std::setprecision(1) << gamma_ << " " << contrast
                         << " " << saturation << " " << sigma_space << " "
                         << sigma_color << ")";
          break;
        }
        case 3: {
          tonemap_stream << tonemap_name << "(" << std::fixed
                         << std::setprecision(1) << gamma_ << " " << scale
                         << " " << saturation << ")";
          break;
        }
        case 4: {
          tonemap_stream << tonemap_name << "(" << std::fixed
                         << std::setprecision(1) << gamma_ << " " << intensity
                         << " " << light_adapt << " " << color_adapt << ")";
          break;
        }
      }

      scaled_stream.str("");
      scaled_stream << "Bits " << std::fixed << std::setprecision(2)
                    << bitRangeHigh << " - " << bitRangeLow;

      std::string positionText = "Image: " + std::to_string(posCounter) +
                                 " / " + std::to_string(numImages);

      if (showTextOverlay == true) {
        cv::putText(display_img, tonemap_stream.str() + " :: " + positionText,
                    cv::Point(10, 60),         // Coordinates
                    cv::FONT_HERSHEY_SIMPLEX,  // Font
                    1.5,                       // Scale
                    cv::Scalar(0, 100, 0),     // BGR Color
                    4,                         // Line Thickness
                    cv::LINE_AA);              // Anti-alias

        cv::putText(mScaledProcessed,
                    scaled_stream.str() + " :: " + positionText,
                    cv::Point(10, 60),         // Coordinates
                    cv::FONT_HERSHEY_SIMPLEX,  // Font
                    1.5,                       // Scale
                    cv::Scalar(0, 100, 0),     // BGR Color
                    4,                         // Line Thickness
                    cv::LINE_AA);              // Anti-alias
      }

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
          cv::createTrackbar("Sigma Space", window_control, &sigma_space_slider,
                             sigma_space_slider_max, on_sigma_space);
          on_sigma_space(sigma_space_slider, 0);
          cv::createTrackbar("Sigma Color", window_control, &sigma_color_slider,
                             sigma_color_slider_max, on_sigma_color);
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
          cv::createTrackbar("Light Adapt", window_control, &light_adapt_slider,
                             light_adapt_slider_max, on_light_adapt);
          on_light_adapt(light_adapt_slider, 0);
          cv::createTrackbar("Color Adapt", window_control, &color_adapt_slider,
                             color_adapt_slider_max, on_color_adapt);
          on_color_adapt(color_adapt_slider, 0);

          break;
        }
      }

      cv::namedWindow(window_scaled_title, cv::WINDOW_NORMAL);
      cv::namedWindow(window_title, cv::WINDOW_NORMAL);

      cv::resizeWindow(window_scaled_title, cv_windowSize);
      cv::resizeWindow(window_title, cv_windowSize);

      cv::imshow(window_scaled_title, mScaledProcessed);
      cv::imshow(window_title, display_img);

      newFrame = false;
      newScaledImage = false;

      key = cv::waitKey(1) & 0xFF;

      if (key == 'q') {
        break;
      }

      if (key == 's') {
        std::ofstream rawfile;
        rawfile.open("rawImage_" + std::to_string(posCounter) + ".raw",
                     ios::out | ios::binary);
        rawfile.write((char*)pImage->GetData(),
                      imgWidth * imgHeight * imgBytesPerPixel);
        rawfile.close();
        std::cout << "Saved rawImage_" << std::to_string(posCounter) << ".raw"
                  << std::endl;

        std::stringstream processed_filename;
        processed_filename.str("");
        processed_filename << tonemap_name << "_" << std::to_string(posCounter)
                           << ".png";
        cv::imwrite(processed_filename.str(), display_img);
        std::cout << "Saved " << processed_filename.str() << std::endl;

        processed_filename.str("");
        processed_filename << "ScaledImage_" << std::fixed
                           << std::setprecision(2) << bitRangeHigh << "-"
                           << std::fixed << std::setprecision(2) << bitRangeLow
                           << "_" << std::to_string(posCounter) << ".png";
        cv::imwrite(processed_filename.str(), mScaledProcessed);
        std::cout << "Saved " << processed_filename.str() << std::endl;
      }

      if (key == 't') {
        tonemap_index++;
        if (tonemap_index > 4) {
          tonemap_index = 0;
        }
        destroyWindow(window_control);
        newFrame = true;
      }

      if (key == 'o') {
        showTextOverlay = !showTextOverlay;
        newFrame = true;
      }

      // Bit range for scaled image
      if (imgBytesPerPixel > 1) {
        // + 1.0 bit
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

        // -1.0 bit
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

        newScaledImage = true;
      }

      // +1 frame
      if (key == ']') {
        pos = ftello(fp);
        if (pos + imgSize >= fSize) {
          std::cout << "EOF!" << std::endl;
        } else {
          Arena::ImageFactory::Destroy(pImage);
          posCounter++;
          pImage = ReadImage(fp);
        }
      }

      // -1 frame
      if (key == '[') {
        pos = ftello(fp);
        if (pos <= imgSize) {
          std::cout << "BOF!" << std::endl;
        } else {
          fseeko(fp, -imgSize - imgSize, SEEK_CUR);
          Arena::ImageFactory::Destroy(pImage);
          posCounter--;
          pImage = ReadImage(fp);
        }
      }

      // +100 frames
      if (key == '\'') {
        pos = ftello(fp);
        if (pos + (imgSize * 100) < fSize) {
          fseeko(fp, imgSize * 100, SEEK_CUR);
          Arena::ImageFactory::Destroy(pImage);
          posCounter = posCounter + 100;
          pImage = ReadImage(fp);
        } else if (pos + imgSize >= fSize) {
          std::cout << "EOF!" << std::endl;
        } else {
          fseeko(fp, 0, SEEK_END);
          fseeko(fp, -imgSize, SEEK_CUR);
          Arena::ImageFactory::Destroy(pImage);
          posCounter = (long)numImages;
          pImage = ReadImage(fp);
        }
      }

      // -100 frames
      if (key == ';') {
        pos = ftello(fp);
        if (pos + (-imgSize * 100) - imgSize > 0) {
          fseeko(fp, (-imgSize * 100) - imgSize, SEEK_CUR);
          Arena::ImageFactory::Destroy(pImage);
          posCounter = posCounter - 100;
          pImage = ReadImage(fp);
        } else if (pos <= imgSize) {
          std::cout << "BOF!" << std::endl;
        } else {
          fseeko(fp, 0, SEEK_SET);
          Arena::ImageFactory::Destroy(pImage);
          posCounter = 1;
          pImage = ReadImage(fp);
        }
      }

      // Reset tonemap parameters
      if (key == 'r') {
        gamma_ = 1.0f;
        gamma_slider = 10;
        contrast = 4.0f;
        contrast_slider = 40;
        saturation = 1.0f;
        saturation_slider = 10;
        bias = 0.85f;
        bias_slider = 17;
        scale = 0.70f;
        scale_slider = 7;
        intensity = 0.0f;
        intensity_slider = 80;
        light_adapt = 1.0f;
        light_adapt_slider = 10;
        color_adapt = 0.0f;
        color_adapt_slider = 0;
        sigma_space = 2.0f;
        sigma_space_slider = 20;
        sigma_color = 2.0f;
        sigma_color_slider = 20;

        newFrame = true;
      }

      if (key == 'v') {
        fseeko(fp, 0, SEEK_SET);
        Arena::ImageFactory::Destroy(pImage);
        posCounter = 1;
        cv::destroyAllWindows();
        pImage = ReadImage(fp);
        outputVideo = true;
      }
    } else {
      std::string video_filename =
          "VIDEO_" + filename + "_" + tonemap_stream.str() + ".mp4";
      video.open(video_filename, fourcc, srcFPS,
                 cv::Size((int)imgWidth, (int)imgHeight));

      while ((long)posCounter <= numImages) {
        std::cout << "Processing " << posCounter << "/" << numImages
                  << std::endl;

        // Tonemapped images
        img = LoadImageIntoOpenCV(pImage, imgWidth, imgHeight, imgBytesPerPixel,
                                  cvTile);
        processed_img = ProcessImageInOpenCV(img, gamma_);

        processed_img.convertTo(display_img, CV_8UC3, 255);
        video.write(display_img);

        Arena::ImageFactory::Destroy(pImage);
        posCounter++;
        pImage = ReadImage(fp);
      }

      video.release();
      outputVideo = false;
      newFrame = true;
      break;
    }
  }
  cv::destroyAllWindows();
}

void PrintUsage(std::string programName) {
  std::cout << endl;
  std::cout << "USAGE:" << std::endl;
  std::cout << "    " << programName
            << " <filename> <pixel_format> <width> <height> <fps> <gamma=1.0> "
               "<v=export_video>"
            << std::endl;
  std::cout << "    <v=export_video> is used for batch export" << std::endl;
}

int main(int argc, char** argv) {
  // flag to track when an exception has been thrown
  bool exceptionThrown = false;

  if (argc < 6) {
    PrintUsage(argv[0]);
    return 0;
  }

  filename = argv[1];
  srcPixelFormat = static_cast<PfncFormat>(GetPixelFormatInteger(argv[2]));
  srcWidth = strtol(argv[3], NULL, 10);
  srcHeight = strtol(argv[4], NULL, 10);
  srcFPS = strtod(argv[5], NULL);

  if (argc >= 7) {
    gamma_ = strtof(argv[6], NULL);
  }

  if (argc >= 8) {
    gamma_ = strtof(argv[6], NULL);

    if ((strcmp(argv[7], "v") == 0)) {
      outputVideo = true;
    }
  }

  // This identifies the input pixel format
  srcBytesPerPixel = 3;

  if (srcPixelFormat == BayerRG8 || srcPixelFormat == BayerGR8 ||
      srcPixelFormat == BayerGB8 || srcPixelFormat == BayerBG8 ||
      srcPixelFormat == Mono8) {
    srcBytesPerPixel = 1;
    srcBitsPerPixel = 8;
  } else if (srcPixelFormat == BayerRG12p ||
             srcPixelFormat == BayerRG12Packed ||
             srcPixelFormat == BayerGR12p ||
             srcPixelFormat == BayerGR12Packed ||
             srcPixelFormat == BayerGB12p ||
             srcPixelFormat == BayerGB12Packed ||
             srcPixelFormat == BayerBG12p ||
             srcPixelFormat == BayerBG12Packed || srcPixelFormat == Mono12p ||
             srcPixelFormat == Mono12Packed) {
    srcBytesPerPixel = 2;
    srcBitsPerPixel = 12;
  } else if (srcPixelFormat == BayerRG12 || srcPixelFormat == BayerRG16 ||
             srcPixelFormat == BayerGR12 || srcPixelFormat == BayerGR16 ||
             srcPixelFormat == BayerGB12 || srcPixelFormat == BayerGB16 ||
             srcPixelFormat == BayerBG12 || srcPixelFormat == BayerBG16 ||
             srcPixelFormat == Mono12 || srcPixelFormat == Mono16) {
    srcBytesPerPixel = 2;
    srcBitsPerPixel = 16;
  } else if (srcPixelFormat == BayerRG24 || srcPixelFormat == BayerGR24 ||
             srcPixelFormat == BayerGB24 || srcPixelFormat == BayerBG24 ||
             srcPixelFormat == Mono24) {
    srcBytesPerPixel = 3;
    srcBitsPerPixel = 24;
  }

  Arena::IDevice* pDevice = nullptr;

  try {
    // prepare example acquisition
    Arena::ISystem* pSystem = Arena::OpenSystem();

    fp = fopen(filename.c_str(), "rb");

    if (fp) {
      fseeko(fp, 0, SEEK_END);
      long long fSize = ftello(fp);
      fseeko(fp, 0, SEEK_SET);

      size_t numImages = (fSize / (srcWidth * srcHeight * srcBytesPerPixel));
      std::cout << "numImages: " << numImages << std::endl;

      Arena::IImage* pImage = nullptr;

      pImage = ReadImage(fp);
      DisplayImage(pImage, fSize, numImages, srcWidth, srcHeight,
                   srcBytesPerPixel);

    } else {
      std::cout << "File '" << filename << "' not found..." << std::endl;
    }

    fclose(fp);

    // std::getchar();
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

  // std::cout << "Press enter to complete\n";
  // std::getchar();

  if (exceptionThrown)
    return -1;
  else
    return 0;
}
