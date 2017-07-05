#include<ros/ros.h>
#include<image_transport/image_transport.h>
#include<opencv2/opencv.hpp>
#include<cv_bridge/cv_bridge.h>

int main(int argc, char** argv)
{
  ros::init(argc,argv,"image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("image_reading",1);
/* from camera
  cv::VideoCapture cap(0);

  cv::Mat frame;
  sensor_msgs::ImagePtr frame_msg;
  ros::Rate loop_rate(5);
  while(ros::ok())
  {
    cap >> frame;
    if (! frame.empty())
    {
	frame_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
	pub.publish(frame_msg);
	cv::waitKey(1);
    }
  */

  //from image
  cv::Mat image = cv::imread(argv[1],CV_LOAD_IMAGE_COLOR);
  cv::Mat transport = cv::Mat(image);
  sensor_msgs::ImagePtr msg =  cv_bridge::CvImage(std_msgs::Header(), "bgr8", transport).toImageMsg();
  
  ros::Rate loop_rate(5);
  while(nh.ok()){
  pub.publish(msg);
  ros::spinOnce();
  loop_rate.sleep();

  }

}
