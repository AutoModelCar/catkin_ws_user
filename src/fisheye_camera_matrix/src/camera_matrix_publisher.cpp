#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Point.h>
#include <fisheye_camera_matrix/perspective_projection.h>
#include "fisheye_camera_matrix/camera_matrix.hpp"

fisheye_camera_matrix::CameraMatrix camera_matrix;

bool relative2image(fisheye_camera_matrix::perspective_projection::Request  &req,
    fisheye_camera_matrix::perspective_projection::Response &res) {
  cv::Point2i p = camera_matrix.relative2image(cv::Point2f(req.x, req.y) );

  res.p.x = p.x;
  res.p.y = p.y;
  res.p.z = 0;

  return true;
}

bool image2relative(fisheye_camera_matrix::perspective_projection::Request  &req,
    fisheye_camera_matrix::perspective_projection::Response &res) {
  cv::Point2f p = camera_matrix.image2relative(cv::Point2f(req.x, req.y) );

   res.p.x = p.x;
   res.p.y = p.y;
   res.p.z = 0;

   return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "camera_matrix_publisher");

  if(argc < 2) {
    ROS_ERROR("Please use roslaunch: 'roslaunch fisheye_camera_matrix camera_matrix_publisher.launch "
              "[calib:=FILE]'");
    return 1;
  }

  std::string path = ros::package::getPath("fisheye_camera_matrix") + std::string("/config/") + std::string(argv[1]);

  if(access(path.c_str(), R_OK ) == -1) {
    ROS_ERROR("No such file: %s\nPlease give a path relative to catkin_ws/src/camera_matrix/config/", path.c_str() );
    return 1;
  }

  ROS_INFO("camera_matrix_publisher: using calibrationfile: %s", path.c_str() );

  bool auto_calibration_enabled = false;

  // TODO add switch to enable auto calibration

  camera_matrix = fisheye_camera_matrix::CameraMatrix(path.c_str() );

  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<fisheye_camera_matrix_msgs::CameraMatrix>("/usb_cam/camera_matrix", 1);
  ros::ServiceServer service_r2i = nh.advertiseService("relative2image", relative2image);
  ros::ServiceServer service_i2r = nh.advertiseService("image2relative", image2relative);
  fisheye_camera_matrix_msgs::CameraMatrix msg = camera_matrix.serialize();
  int seq_nbr = 0;
  ros::Rate r(1);

  while(ros::ok() ) {
    if(auto_calibration_enabled)
      if(camera_matrix.update_calibration() )
        msg = camera_matrix.serialize();

    msg.header.seq   = ++seq_nbr;
    msg.header.stamp = ros::Time::now();

    pub.publish(msg);
    r.sleep();
    ros::spinOnce();
  }
}
