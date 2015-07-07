/*
 * intrinsic_calib_loader.cpp
 *
 *  Created on: Jun 10, 2015
 *      Author: md
 */

#include <pylon_camera/intrinsic_calib_loader.h>

namespace pylon_camera
{

IntrinsicCalibLoader::IntrinsicCalibLoader() :
                    img_cols_(-1),
                    img_rows_(-1),
                    K_(),
                    D_(),
                    intrinsic_yaml_file_("INVALID_YAML_FILE")
{
}

IntrinsicCalibLoader::~IntrinsicCalibLoader()
{
    // TODO Auto-generated destructor stub
}
bool IntrinsicCalibLoader::init(const std::string yaml_file)
{
    intrinsic_yaml_file_ = yaml_file;
    return true;
}
bool IntrinsicCalibLoader::loadCalib()
{
    if (intrinsic_yaml_file_ == "INVALID_YAML_FILE")
    {
        cerr << "Error loading Intrinsic calib file: call init() first!" << endl;
        return false;
    }
    //std::cout << intrinsic_yaml_file_ << std::endl;
    cv::FileStorage fs(intrinsic_yaml_file_, cv::FileStorage::READ + cv::FileStorage::MEMORY);

    if (!fs.isOpened())
    {
        cerr << "Could not open cv::Filestorage with intrinsic calib yaml file: " << endl;
        cerr << intrinsic_yaml_file_.c_str() << endl;
        return false;
    }
    else
    {

        fs["distortion"] >> D_;
        fs["cam_matrix"] >> K_;

        // two calib file versions: one with 'row', 'col' and one with 'width', 'height'
        // if not existent: filestorage sets to 0
        int row_1 = 0, col_1 = 0, row_2 = 0, col_2 = 0;

        fs["cols"] >> col_1;
        fs["rows"] >> row_1;
        fs["width"] >> col_2;
        fs["height"] >> row_2;

        row_1 > row_2 ? img_rows_ = row_1 : img_rows_ = row_2;
        col_1 > col_2 ? img_cols_ = col_1 : img_cols_ = col_2;

        fs.release();

        if (img_cols_ <= 0 || img_cols_ <= 0)
        {
            cerr << "Error: Unexpected img size (" << img_cols_ << ", " << img_rows_
                 << ") while reading from yaml file!"
                 << endl;
            return false;
        }
        if (D_.rows != 5 && D_.cols != 1)
        {
            cerr << "Error: Unexpected size of distortion matrix (" << D_.rows << " x " << D_.cols
                 << ") while reading from yaml file! Should be: (5 x 1)"
                 << endl;
            return false;
        }
        if (K_.rows != 3 && K_.cols != 3)
        {
            cerr << "Error: Unexpected size of camera matrix (" << K_.rows << " x " << K_.cols
                 << ") while reading from yaml file! Should be: (3 x 3)"
                 << endl;
            return false;
        }
    }

    return true;
}
cv::Mat IntrinsicCalibLoader::D()
{
    return D_;
}
cv::Mat IntrinsicCalibLoader::K()
{
    return K_;
}
int IntrinsicCalibLoader::img_rows()
{
    return img_rows_;
}
int IntrinsicCalibLoader::img_cols()
{
    return img_cols_;
}
} /* namespace pylon_camera */