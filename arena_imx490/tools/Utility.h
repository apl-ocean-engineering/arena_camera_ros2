
#include <math.h>

#include "ArenaApi.h"
#include "SaveApi.h"
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

namespace Arena {

#define MAX_SUPPORTED_CORRECTION (15.9)

Mat Unpack12BitImage(IImage* image) {
  size_t buff_size = image->GetSizeOfBuffer();
  size_t width = image->GetWidth();
  size_t height = image->GetHeight();

  size_t n_pixels = width * height;
  size_t expected_12bit_packed_buff_size = n_pixels * 3 / 2;
  if (buff_size < expected_12bit_packed_buff_size) {
    throw std::runtime_error("Error unpacking 12-bit image!");
  }

  cv::Mat image16_bit((int)height, (int)width, CV_16UC1);

  const uint8_t* input_data = image->GetData();
  uint16_t* out_data16 = (uint16_t*)image16_bit.data;

  for (size_t i = 0; i < n_pixels; i += 2) {
    // Unpack 3 bytes into two output pixels
    out_data16[i] =
        ((uint16_t)input_data[0] << 4) + ((uint16_t)input_data[1] & 0x0F);
    out_data16[i + 1] = ((uint16_t)input_data[2] << 4) +
                        (((uint16_t)input_data[1] & 0xF0) >> 4);
    input_data += 3;
  }

  return image16_bit;
}

Mat GetCvImage(IDevice* pDevice) {
  IImage* image = pDevice->GetImage(2000);

  unsigned int retry_count = 0;
  const unsigned int retry_count_max = 50;
  while (image->IsIncomplete()) {
    retry_count++;
    pDevice->RequeueBuffer(image);
    image = pDevice->GetImage(2000);
    if (retry_count > retry_count_max) {
      char s[512];
      snprintf(s, 511,
               "Cannot get valid image data after %u "
               "retries\n",
               retry_count_max);
      throw std::runtime_error(s);
    }
  }

  uint64_t pixel_format = image->GetPixelFormat();
  size_t height = image->GetHeight();
  size_t width = image->GetWidth();

  if (((pixel_format & 0xFF000000) >> 24) > 1) {
    throw std::runtime_error(
        "GetCvImage() doesn't support composite pixel format.");
  }

  if (pixel_format == GVSP_Mono12Packed ||
      pixel_format == GVSP_BayerBG12Packed ||
      pixel_format == GVSP_BayerGB12Packed ||
      pixel_format == GVSP_BayerGR12Packed ||
      pixel_format == GVSP_BayerRG12Packed) {
    Mat cv_image = Unpack12BitImage(image);

    pDevice->RequeueBuffer(image);

    return cv_image;
  }

  size_t bits_per_pixel = image->GetBitsPerPixel();
  if (bits_per_pixel != 8 && bits_per_pixel != 16) {
    char c[512];
    snprintf(c, 511, "Unsupported value for image bits per pixel (%d)",
             (int)bits_per_pixel);
    throw std::runtime_error(c);
  }

  Mat cv_image(static_cast<int>(height), static_cast<int>(width),
               (bits_per_pixel == 16) ? CV_16UC1 : CV_8UC1, Scalar::all(0));

  size_t bytes_per_pixel = bits_per_pixel / 8;
  size_t image_data_size_bytes = width * height * bytes_per_pixel;

  // Copy bytes from Arena image buffer to opencv image buffer
  memcpy(cv_image.data, image->GetData(), image_data_size_bytes);

  // free image
  pDevice->RequeueBuffer(image);

  return cv_image;
}

// Capture flat field images and create a table
Mat CreateTableFromImages(IDevice* pDevice, double numImagesToAverage) {
  Mat cvImage = GetCvImage(pDevice);

  cv::Mat sumImage = cv::Mat::zeros(cvImage.rows, cvImage.cols, CV_64FC1);
  cv::add(sumImage, cvImage, sumImage, cv::Mat(), CV_64FC1);
  for (int i = 1; i < numImagesToAverage; i++) {
    cv::add(sumImage, GetCvImage(pDevice), sumImage, cv::Mat(), CV_64FC1);
  }
  sumImage.convertTo(cvImage, cvImage.type(), 1.0 / double(numImagesToAverage));
  sumImage.release();
  return cvImage;
}

unsigned int GetCellNodeDimX(unsigned int width_orig,
                             unsigned int cell_width_x) {
  // Calculate cell node count x
  unsigned int remainder = width_orig % cell_width_x;
  unsigned int cell_node_count_x = width_orig / cell_width_x + 1;
  return cell_node_count_x =
             remainder > 0 ? cell_node_count_x + 1 : cell_node_count_x;
}

unsigned int GetCellNodeDimY(unsigned int height_orig,
                             unsigned int cell_width_y) {
  // Calculate cell node count y
  unsigned int remainder = height_orig % cell_width_y;
  unsigned int cell_node_count_y = height_orig / cell_width_y + 1;
  return cell_node_count_y =
             remainder > 0 ? cell_node_count_y + 1 : cell_node_count_y;
}

bool VerifyCorrectionTable(Mat& correction_table) {
  for (int ir = 0; ir < correction_table.rows; ir++) {
    for (int ic = 0; ic < correction_table.cols; ic++) {
      double this_corr_amount = correction_table.at<double>(ir, ic);
      if (this_corr_amount > MAX_SUPPORTED_CORRECTION) {
        return 0;
      }
    }
  }
  return 1;
}

Mat ComputeCorrectionMap(Mat& image, unsigned int cell_width_x,
                         unsigned int cell_width_y,
                         unsigned int cell_node_count_x,
                         unsigned int cell_node_count_y) {
  Mat image_f64;
  image.convertTo(image_f64, CV_64FC1);

  // Get an image of cells
  unsigned int image_height = image.rows;
  unsigned int image_width = image.cols;
  unsigned int num_cell_x = cell_node_count_x - 1;
  unsigned int num_cell_y = cell_node_count_y - 1;
  Mat cell_image = Mat(num_cell_y, num_cell_x, CV_64FC1);

  for (unsigned int ir = 0; ir < num_cell_y; ir++) {
    unsigned int start_y = ir * cell_width_y;
    unsigned int stop_y = min((ir + 1) * cell_width_y, image_height) - 1;

    for (unsigned int ic = 0; ic < num_cell_x; ic++) {
      unsigned int start_x = ic * cell_width_x;
      unsigned int stop_x = min((ic + 1) * cell_width_x, image_width) - 1;
      cv::Mat this_cell = image_f64(cv::Rect(
          start_x, start_y, stop_x - start_x + 1, stop_y - start_y + 1));
      Scalar mu, sigma;
      meanStdDev(this_cell, mu, sigma);
      cell_image.at<double>(ir, ic) = mu.val[0];
    }
  }

  // Need to get the correction map of the whole image
  double min_val, max_val;
  minMaxLoc(cell_image, &min_val, &max_val);
  Mat correction_map = Mat(image_height, image_width, CV_64FC1);
  correction_map = max_val / image_f64;
  image_f64.release();
  cell_image.release();

  return correction_map;
}

Mat ContructCorrectionTable(Mat& correction_map, unsigned int cell_width_x,
                            unsigned int cell_width_y,
                            unsigned int cell_node_count_x,
                            unsigned int cell_node_count_y) {
  unsigned int width_orig = correction_map.cols;
  unsigned int height_orig = correction_map.rows;

  // Create a correction map such that its height is a multiple of cell_width_y
  // and its width is a multiple of cell_width_y
  unsigned int width_extented = cell_width_x * (cell_node_count_x - 1);
  unsigned int height_extented = cell_width_y * (cell_node_count_y - 1);
  Mat correction_map_extended =
      Mat(height_extented, width_extented, CV_64FC1, Scalar(0));

  for (unsigned int row = 0; row < height_extented; row++) {
    for (unsigned int col = 0; col < width_extented; col++) {
      if ((row < height_orig) && (col < width_orig)) {
        correction_map_extended.at<double>(row, col) =
            correction_map.at<double>(row, col);
      }

      if ((row >= height_orig) && (col >= width_orig)) {
        correction_map_extended.at<double>(row, col) =
            correction_map.at<double>(height_orig - 1, width_orig - 1);
      }

      if ((row < height_orig) && (col >= width_orig)) {
        correction_map_extended.at<double>(row, col) =
            correction_map.at<double>(row, width_orig - 1);
      }

      if ((row >= height_orig) && (col < width_orig)) {
        correction_map_extended.at<double>(row, col) =
            correction_map.at<double>(height_orig - 1, col);
      }
    }
  }

  // Sample the data using the mean of the surrounding pixels
  unsigned int height_sampled = cell_node_count_y;
  unsigned int width_sampled = cell_node_count_x;
  Mat correction_map_sampled(height_sampled, width_sampled, CV_64FC1,
                             Scalar(0));

  int start_x, end_x, start_y, end_y;
  unsigned int half_cell_width_x = cell_width_x / 2;
  unsigned int half_cell_width_y = cell_width_y / 2;
  for (unsigned int row = 0; row < height_sampled; row++) {
    if (row == 0) {
      start_y = 0;
    } else {
      start_y = cell_width_y * row - half_cell_width_y;
    }

    if (row == height_sampled - 1) {
      end_y = cell_width_y * row;
    } else {
      end_y = cell_width_y * row + half_cell_width_y;
    }

    for (unsigned int col = 0; col < width_sampled; col++) {
      if (col == 0) {
        start_x = 0;
      } else {
        start_x = cell_width_x * col - half_cell_width_x;
      }

      if (col == width_sampled - 1) {
        end_x = cell_width_x * col;
      } else {
        end_x = cell_width_x * col + half_cell_width_x;
      }

      // Only take the area around the grid node
      Mat grid_area = correction_map_extended(
          Rect(start_x, start_y, end_x - start_x, end_y - start_y));
      correction_map_sampled.at<double>(row, col) = mean(grid_area).val[0];
    }
  }

  // Check if the correction is larger than the max supported one
  bool table_is_good = VerifyCorrectionTable(correction_map_sampled);
  if (!table_is_good) {
    char e[1000];
    sprintf(e,
            "The calibrated correction amount is greater than the max "
            "supported amount %2.2f. The target may not be a flat field.",
            MAX_SUPPORTED_CORRECTION);
    throw std::runtime_error(e);
  }

  return correction_map_sampled;
}

}  // namespace Arena
