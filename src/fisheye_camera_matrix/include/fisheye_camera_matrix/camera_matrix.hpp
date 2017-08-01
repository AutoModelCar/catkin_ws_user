#include <opencv2/core/core.hpp>

#include "fisheye_camera_matrix_msgs/CameraMatrix.h"

namespace fisheye_camera_matrix {

class CameraMatrix {
public:
  CameraMatrix(int width, int height, int cx, int cy,
      float focal_length, float ceiling_height, float scale);
  CameraMatrix();
  CameraMatrix(const char *path);
  CameraMatrix(const fisheye_camera_matrix_msgs::CameraMatrix &msg);
  ~CameraMatrix();

  fisheye_camera_matrix_msgs::CameraMatrix serialize();

  bool update_calibration();

  void undistort(const cv::Mat &src, cv::Mat &dst);
  cv::Point2i relative2image(const cv::Point2f &p);
  cv::Point2f image2relative(const cv::Point2i &p);

  int width;
  int height;
  int cx;
  int cy;
  float fl;
  float ceil_height;
  float scale;
};
}
