#include <string>
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "fisheye_camera_matrix/camera_matrix.hpp"

fisheye_camera_matrix::CameraMatrix camera_matrix;
cv::Mat img;
cv::Mat img_ud;

int width;
int height;
int cx;
int cy;

int fl              = 320;
int fl_max          = 960;
int ceil_height     = 200;
int ceil_height_max = 350;
int scale           = 10;
int scale_max       = 30;

void apply(int, void*) {
  camera_matrix = fisheye_camera_matrix::CameraMatrix(
      width, height, cx, cy, fl, ceil_height / 100.0, scale / 10.0);

  camera_matrix.undistort(img, img_ud);

  cv::Point2f t1(-1,  0);
  cv::Point2f t2( 1,  0);
  cv::Point2f t3( 0, -1);
  cv::Point2f t4( 0,  1);
  cv::Point2f t5(-1, -1);
  cv::Point2f t6( 1,  1);

  cv::Point2i t5_img = camera_matrix.relative2image(t5);
  cv::Point2i t6_img = camera_matrix.relative2image(t6);

  printf("\n================ test: project forth and back ================\n");
  printf("ceil: (%.2f, %.2f) -> img: (%d, %d) -> ceil: (%.2f, %.2f)\n",
      t5.x, t5.y, t5_img.x, t5_img.y,
      camera_matrix.image2relative(t5_img).x, camera_matrix.image2relative(t5_img).y);
  printf("ceil: (%.2f, %.2f) -> img: (%d, %d) -> ceil: (%.2f, %.2f)\n",
      t6.x, t6.y, t6_img.x, t6_img.y,
      camera_matrix.image2relative(t6_img).x, camera_matrix.image2relative(t6_img).y);

  cv::line(img_ud, camera_matrix.relative2image(t1), camera_matrix.relative2image(t2), cv::Scalar(0, 0, 255), 2);
  cv::line(img_ud, camera_matrix.relative2image(t3), camera_matrix.relative2image(t4), cv::Scalar(0, 0, 255), 2);
  cv::imshow("undistorted", img_ud);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "calibration");

  if(argc < 3) {
    ROS_ERROR("Please use roslaunch: 'roslaunch fisheye_camera_matrix calibrate_offline_manual.launch "
              "[img:=FILE] [calib:=FILE]'");
    return 1;
  }

  std::string path_img   = ros::package::getPath("fisheye_camera_matrix") + std::string("/../../../captures/") + std::string(argv[1]);
  std::string path_calib = ros::package::getPath("fisheye_camera_matrix") + std::string("/config/") + std::string(argv[2]);

  if(access(path_img.c_str(), R_OK ) == -1) {
    ROS_ERROR("No such file: %s\nPlease give a path relative to catkin_ws/../captures/", path_img.c_str() );
    return 1;
  }

  ROS_INFO("calibration: using imagefile %s", path_img.c_str() );
  ROS_INFO("calibration: using calibrationfile: %s", path_calib.c_str() );

  img    = cv::imread(path_img);
  width  = img.cols;
  height = img.rows;
  cx     = width / 2;
  cy     = height / 2;

  img.copyTo(img_ud);
  cv::namedWindow("undistorted", CV_WINDOW_AUTOSIZE);
  cv::createTrackbar("focal length", "undistorted", &fl, fl_max, apply);
  cv::createTrackbar("distance lens<->ceiling (cm)", "undistorted", &ceil_height, ceil_height_max, apply);
  cv::createTrackbar("scale", "undistorted", &scale, scale_max, apply);
  apply(0, (void*)0);
  cv::waitKey(0);

  std::ofstream file;
  file.open(path_calib.c_str(), std::ios::out | std::ios::trunc);

  file << width << " " << height << " " << cx << " " << cy << " "
      << fl << " " << (ceil_height / 100.0) << " " << (scale / 10.0) << std::endl;

  file.close();
  ROS_INFO("Calibration written to %s.", path_calib.c_str() );

  return 0;
}
