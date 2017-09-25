#include <math.h>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include "fisheye_camera_matrix/camera_matrix.hpp"

namespace fisheye_camera_matrix {

CameraMatrix::CameraMatrix(int _width, int _height, int _cx, int _cy,
    float _focal_length, float _ceiling_height, float _scale) :
      width(_width),
      height(_height),
      cx(_cx),
      cy(_cy),
      fl(_focal_length),
      ceil_height(_ceiling_height),
      scale(_scale)
{}

CameraMatrix::CameraMatrix() :
      width(0),
      height(0),
      cx(0),
      cy(0),
      fl(1),
      ceil_height(1),
      scale(1)
{}

CameraMatrix::CameraMatrix(const char *path) {
  std::ifstream file;
  std::string line_str;

  int _width         = 0;
  int _height        = 0;
  int _cx            = 0;
  int _cy            = 0;
  float _fl          = 0;
  float _ceil_height = 0;
  float _scale       = 0;

  file.open(path, std::ios::in);
  getline(file, line_str);

  std::stringstream line_ss(line_str);

  line_ss >> _width >> _height >> _cx >> _cy >> _fl >> _ceil_height >> _scale;

  file.close();

  width       = _width;
  height      = _height;
  cx          = _cx;
  cy          = _cy;
  fl          = _fl;
  ceil_height = _ceil_height;
  scale       = _scale;
}

CameraMatrix::CameraMatrix(const fisheye_camera_matrix_msgs::CameraMatrix &msg) {
  width       = msg.width;
  height      = msg.height;
  cx          = msg.cx;
  cy          = msg.cy;
  fl          = msg.fl;
  ceil_height = msg.h;
  scale       = msg.scale;
}

CameraMatrix::~CameraMatrix() {

}

fisheye_camera_matrix_msgs::CameraMatrix CameraMatrix::serialize() {
  fisheye_camera_matrix_msgs::CameraMatrix msg;

  msg.width  = width;
  msg.height = height;
  msg.cx     = cx;
  msg.cy     = cy;
  msg.fl     = fl;
  msg.h      = ceil_height;
  msg.scale  = scale;

  return msg;
}

bool CameraMatrix::update_calibration() {

  // TODO implement automatic calibration of ceil_height

  // TODO implement automatic calibration of focal length

  return false;
}

void CameraMatrix::undistort(const cv::Mat &src, cv::Mat &dst) {
  for(int x = 0; x < width; ++x) {
    const float rx = (float)(x - cx);

      for(int y = 0; y < height; ++y) {
        const float ry   = (float)(y - cy);
        const float r    = sqrtf(rx * rx + ry * ry);
        const float r_ud = scale * sinf(atanf(r / fl) ) * fl;
        const int x_ud   = (int)(cx + rx * r_ud / r);
        const int y_ud   = (int)(cy + ry * r_ud / r);

        if(x_ud >= 0 && y_ud >= 0 && x_ud < width && y_ud < height)
          dst.at<cv::Vec3b>(y, x) = src.at<cv::Vec3b>(y_ud, x_ud);
        else
          dst.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
      }
  }
}

cv::Point2i CameraMatrix::relative2image(const cv::Point2f &p) {
  cv::Point2f rot(p.y, -p.x);

  return cv::Point2i(
      (int)(cx + (1 / scale) * fl * rot.x / ceil_height),
      (int)(cy + (1 / scale) * fl * rot.y / ceil_height) );
}

cv::Point2f CameraMatrix::image2relative(const cv::Point2i &p) {
    cv::Point2f rel(
        (p.x - cx) * ceil_height / fl,
        (p.y - cy) * ceil_height / fl);

    return cv::Point2f(
        -scale * rel.y,
         scale * rel.x);
}

}
