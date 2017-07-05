#include<ros/ros.h>
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>

#include<dlib/image_io.h>
#include<dlib/opencv.h>
#include<dlib/image_processing.h>
#include<dlib/gui_widgets.h>
#include<dlib/image_transforms.h>

#include<sys/time.h>

#define SUBSCRIBER_TOPIC "app/camera/rgb/image_raw"
#define PUBLISHER_TOPIC "traffic_sign_detection"

#define SVM_PATH "/root/catkin_ws_user/src/traffic_sign_detection/src/own27.svm"
#define MAX_CANDIDATES_CONSIDERED 5

#define CANDIDATE_PADDING 10
#define MS_TIMEOUT 700
#define MIN_RATIO 0.8
#define MAX_RATIO 1.2
#define MIN_AREA 100
#define MAX_AREA 10000

using namespace dlib;
using namespace std;

class preprocessor
{
	private:
		ros::NodeHandle nh;
		image_transport::ImageTransport it;
		image_transport::Subscriber imageSub;
		image_transport::Publisher imagePub;

	public:
		preprocessor():it(nh){
			imageSub = it.subscribe(SUBSCRIBER_TOPIC, 1, &preprocessor::imageCallBack, this);
			imagePub = it.advertise(PUBLISHER_TOPIC, 1);
		}

		void imageCallBack(const sensor_msgs::ImageConstPtr &msg){
			// Convert to cv type for further processing
			cv_bridge::CvImagePtr cv_ptr;
			try {
				cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
			}
			catch(cv_bridge::Exception &e) {
				ROS_ERROR("cv_bridge exception: %s",e.what());
				return;
			}


			typedef scan_fhog_pyramid<pyramid_down<6>> image_scanner_type;

			object_detector<image_scanner_type> detector;
			deserialize(SVM_PATH) >> detector;

			struct timeval tp;
			gettimeofday(&tp, NULL);
			long int ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;

			// Convert the image to HSV because we want to find red pixels
			// independent of lightning and other factors
			cv::Mat hsv_image;
			cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

			cv::Mat lower_mask;
			cv::Mat upper_mask;

			// In HSV red can appear in two intervals, so we have two masks
			cv::inRange(hsv_image, cv::Scalar(0, 50, 50), cv::Scalar(10, 255, 255), lower_mask);
			cv::inRange(hsv_image, cv::Scalar(160,50,50),cv::Scalar(180, 255, 255), upper_mask);

			cv::Mat combination;
			cv::addWeighted(lower_mask, 1.0, upper_mask, 1.0, 0.0, combination);

			// Find all contours around parts of the image that contain much red
			std::vector<std::vector<cv::Point>> contours;
			cv::findContours(
					combination,
					contours,
					CV_RETR_EXTERNAL, // Only use outer contours
					CV_CHAIN_APPROX_SIMPLE // Compress heavily overlapping contours
					);

			bool found_sign = false;
			int candidates_considered = 0;
			for(size_t i = 0 ; i < contours.size(); i++) {
				// We only want to consider candidates that are large enough to contain
				// a sign. By doing this, we of course ignore very small signs, but there
				// needs to be a cut-off at some point
				int area = cv::contourArea(contours[i]);

				if(area >= MIN_AREA && area <= MAX_AREA) {
					// Sometimes there are a lot of candidates. To get an acceptable runtime
					// we only look at the top ones
					if(candidates_considered >= MAX_CANDIDATES_CONSIDERED) {
						break;
					} else {
						candidates_considered++;
					}

					cv::Rect rect = cv::boundingRect(contours[i]);

					// The bounding box of a sign is nearly square, so we only consider
					// candidates where the ratio between width and height is similiar
					float ratio = ((float)rect.width)/((float)rect.height);

					if(ratio<= MAX_RATIO && ratio >= MIN_RATIO){
						// Each candidate is a subimage of the orginal image. Here, we are
						// just making sure the coordinates are valid: nonnegative and smaller
						// than the image width/height
						int sub_x = std::max(rect.x-CANDIDATE_PADDING, 0);
						int sub_y = std::max(rect.y-CANDIDATE_PADDING, 0);
						int sub_width = std::min(rect.width+ 2*CANDIDATE_PADDING, combination.cols - sub_x);
						int sub_height = std::min(rect.height+ 2*CANDIDATE_PADDING, combination.rows - sub_y);
						cv::Mat subimage = cv::Mat(cv_ptr->image, cv::Rect(sub_x, sub_y, sub_width, sub_height));

						cv::rectangle(cv_ptr->image, cv::Rect(sub_x, sub_y, sub_width, sub_height), cv::Scalar(0), 2, 8, 0);

						// img is the main image we will work on
						array2d<rgb_pixel> img;
						assign_image(img,cv_image<bgr_pixel>(subimage));

						// The candidate image is upsampled, to make sure we find all signs
						pyramid_up(img);
						pyramid_up(img);

						long int height;
						long int width;
						height = img.nc();
						width = img.nr();

						// To get correct bounding boxes, we need to make sure to consider
						// the factor by which dlib upscaled
						double row_factor = (double)height/(double)subimage.cols;
						double col_factor = (double)width/(double)subimage.rows;
						std::vector<dlib::rectangle> signs = detector(img);

						std::vector<dlib::rectangle> scaled_signs(signs.size());

						for(int i = 0; i < signs.size(); i++){
							scaled_signs[i] = dlib::rectangle(
									signs[i].left() / col_factor,
									signs[i].top() / row_factor,
									signs[i].right() / col_factor,
									signs[i].bottom() / row_factor
									);

							// Here we use the simplified rect bounding box. To get one closer to
							// the sign, you might want to use scaled_signs here
							cv::rectangle(cv_ptr->image, rect, cv::Scalar(255), 3, 8, 0);
							imagePub.publish(cv_ptr->toImageMsg());
							found_sign = true;
						}

						gettimeofday(&tp, NULL);
						long int ms_now = tp.tv_sec * 1000 + tp.tv_usec / 1000;
						if(ms_now - ms >= MS_TIMEOUT) {
							break;
						}
					}
				}
			}

			if(!found_sign) {
				imagePub.publish(cv_ptr->toImageMsg());
			}

		}
};

int main(int argc, char** argv)
{
	ros::init(argc,argv, "detection");
	preprocessor p;
	ros::spin();
	return 0;
}
