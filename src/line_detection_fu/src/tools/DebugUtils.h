#ifndef _DebugUtils_FILTER_HEADER_
#define _DebugUtils_FILTER_HEADER_

#include <stdio.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/video/background_segm.hpp>

#include "../laneDetection.h"
#include "Edges.h"
#include "NewtonPolynomial.h"
#include "FuPoint.h"

#include <stdlib.h>
#include <sys/stat.h>


class DebugUtils {
public:
    DebugUtils(ros::NodeHandle &nh, int minYPolyRoi, int maxYRoi, int roiBottomW, int roiTopW,
               int projImageW, int projImageH);

    void drawEdgeWindow(cv::Mat &img, std::vector<std::vector<EdgePoint>> edges);

    void drawLaneMarkingsWindow(cv::Mat &img, std::vector<FuPoint<int>> &laneMarkings);

    void drawGroupedLaneMarkingsWindow(cv::Mat &img, int defaultXLeft, int defaultXCenter, int defaultXRight,
                                       NewtonPolynomial &polyLeft, NewtonPolynomial &polyCenter, NewtonPolynomial &polyRight,
                                       std::vector<FuPoint<int>> &laneMarkingsLeft,
                                       std::vector<FuPoint<int>> &laneMarkingsCenter,
                                       std::vector<FuPoint<int>> &laneMarkingsRight,
                                       std::vector<FuPoint<int>> &laneMarkingsNotUsed);

    void drawRansacWindow(cv::Mat &img, NewtonPolynomial &polyLeft, NewtonPolynomial &polyCenter, NewtonPolynomial &polyRight,
                          NewtonPolynomial &movedPolyLeft, NewtonPolynomial &movedPolyCenter, NewtonPolynomial &movedPolyRight);

    void drawAngleWindow(cv::Mat &img, bool polyDetectedRight, bool isPolyMovedRight, double lastAngle,
                         double gradientForAngle, int angleAdjacentLeg, FuPoint<double> &pointForAngle,
                         FuPoint<double> &movedPointForAngle);

private:
    const bool PUBLISH_IMAGES = true;
    const bool SAVE_FRAME_IMAGES = true;

    bool SHOW_EDGE_WINDOW = true;
    bool SHOW_LANE_MARKINGS_WINDOW = true;
    bool SHOW_GROUPED_LANE_MARKINGS_WINDOW = true;
    bool SHOW_RANSAC_WINDOW = true;
    bool SHOW_ANGLE_WINDOW = true;

    image_transport::CameraPublisher image_publisher;
    image_transport::CameraPublisher image_publisher_ransac;
    image_transport::CameraPublisher image_publisher_lane_markings;

    // params for pubRGBImageMsg
    unsigned int head_sequence_id = 0;
    ros::Time head_time_stamp;
    std::string rgb_frame_id = "_rgb_optical_frame";
    sensor_msgs::CameraInfoPtr rgb_camera_info;

    // params from laneDetectionNode
    int minYPolyRoi;
    int maxYRoi;
    int roiBottomW;
    int roiTopW;
    int projImageHalfW;
    int projImageH;

    // frame number for saving image files
    std::size_t frame = 0;

    void pubRGBImageMsg(cv::Mat& rgb_mat, image_transport::CameraPublisher publisher);

    void debugPaintPolynom(cv::Mat &m, cv::Scalar color, NewtonPolynomial &p, int start, int end);

    void debugPaintPoints(cv::Mat &m, cv::Scalar color, std::vector<FuPoint<int>> &points);

    void debugWriteImg(cv::Mat &image, std::string folder);
};

#endif