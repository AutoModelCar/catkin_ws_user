#include "DebugUtils.h"

using namespace std;
using namespace cv;

DebugUtils::DebugUtils(ros::NodeHandle &nh, int minYPolyRoi, int maxYRoi, int roiBottomW, int roiTopW, int projImageW, int projImageH) {
    this->minYPolyRoi = minYPolyRoi;
    this->maxYRoi = maxYRoi;
    this->roiBottomW = roiBottomW;
    this->roiTopW = roiTopW;
    this->projImageHalfW = projImageW / 2;
    this->projImageH = projImageH;

    head_time_stamp = ros::Time::now();

    if (PUBLISH_IMAGES) {
        image_transport::ImageTransport image_transport(nh);

        image_publisher = image_transport.advertiseCamera("/lane_model/lane_model_image", cLaneDetectionFu::MY_ROS_QUEUE_SIZE);
        image_publisher_ransac = image_transport.advertiseCamera("/lane_model/ransac", cLaneDetectionFu::MY_ROS_QUEUE_SIZE);
        image_publisher_lane_markings = image_transport.advertiseCamera("/lane_model/lane_markings", cLaneDetectionFu::MY_ROS_QUEUE_SIZE);

        if (!rgb_camera_info) {
            rgb_camera_info.reset(new sensor_msgs::CameraInfo());
            rgb_camera_info->width = projImageW;
            rgb_camera_info->height = projImageH + 50;
        }
    }

    if (SAVE_FRAME_IMAGES) {
        mkdir("groupedLaneMarkings", S_IRWXU);
        mkdir("ransac", S_IRWXU);

        system("exec rm -r ./groupedLaneMarkings/*");
        system("exec rm -r ./ransac/*");
    }
}

void DebugUtils::drawEdgeWindow(Mat &img, vector<vector<EdgePoint>> edges) {
    if (!SHOW_EDGE_WINDOW) {
        return;
    }

    Mat transformedImagePaintable = img.clone();
    cvtColor(transformedImagePaintable, transformedImagePaintable, CV_GRAY2BGR);
    for (int i = 0; i < (int) edges.size(); i++) {
        for (int j = 0; j < edges[i].size(); j++) {
            FuPoint<int> edge = edges[i][j].getImgPos();
            Point edgeLoc = Point(edge.getX(), edge.getY());
            circle(transformedImagePaintable, edgeLoc, 1, Scalar(0, 0, edges[i][j].getValue()), -1);
        }
    }

    /*cv::Point2d p1(projImageHalfW-(roiBottomW/2),maxYRoi-1);
    cv::Point2d p2(projImageHalfW+(roiBottomW/2),maxYRoi-1);
    cv::Point2d p3(projImageHalfW+(roiTopW/2),minYPolyRoi);
    cv::Point2d p4(projImageHalfW-(roiTopW/2),minYPolyRoi);
    cv::line(transformedImagePaintable,p1,p2,cv::Scalar(0,200,0));
    cv::line(transformedImagePaintable,p2,p3,cv::Scalar(0,200,0));
    cv::line(transformedImagePaintable,p3,p4,cv::Scalar(0,200,0));
    cv::line(transformedImagePaintable,p4,p1,cv::Scalar(0,200,0));*/

    /*for(int i = 0; i < (int)scanlines.size(); i++)
    {
        LineSegment<int> scanline = scanlines[i][0];
        cv::Point scanlineStart = cv::Point(scanline.getStart().getX(), scanline.getStart().getY());
        cv::Point scanlineEnd = cv::Point(scanline.getEnd().getX(), scanline.getEnd().getY());
        cv::line(transformedImagePaintable,scanlineStart,scanlineEnd,cv::Scalar(255,0,0));
    }*/


    cv::namedWindow("ROI, scanlines and edges", WINDOW_NORMAL);
    cv::imshow("ROI, scanlines and edges", transformedImagePaintable);
    cv::waitKey(1);
}

void DebugUtils::drawLaneMarkingsWindow(Mat &img, vector<FuPoint<int>> &laneMarkings) {
    if (!SHOW_LANE_MARKINGS_WINDOW) {
        return;
    }

    Mat transformedImagePaintable = img.clone();
    cv::cvtColor(transformedImagePaintable, transformedImagePaintable, CV_GRAY2BGR);
    for (int i = 0; i < (int) laneMarkings.size(); i++) {
        FuPoint<int> marking = laneMarkings[i];
        cv::Point markingLoc = cv::Point(marking.getX(), marking.getY());
        cv::circle(transformedImagePaintable, markingLoc, 1, cv::Scalar(0, 255, 0), -1);
    }

    cv::namedWindow("Lane Markings", WINDOW_NORMAL);
    cv::imshow("Lane Markings", transformedImagePaintable);
    cv::waitKey(1);
}

void DebugUtils::drawGroupedLaneMarkingsWindow(Mat &img, int defaultXLeft, int defaultXCenter, int defaultXRight,
                                               NewtonPolynomial &polyLeft, NewtonPolynomial &polyCenter,
                                               NewtonPolynomial &polyRight,
                                               std::vector<FuPoint<int>> &laneMarkingsLeft,
                                               std::vector<FuPoint<int>> &laneMarkingsCenter,
                                               std::vector<FuPoint<int>> &laneMarkingsRight,
                                               std::vector<FuPoint<int>> &laneMarkingsNotUsed) {
    if (!PUBLISH_IMAGES && !SHOW_GROUPED_LANE_MARKINGS_WINDOW && !SAVE_FRAME_IMAGES) {
        return;
    }

    Mat transformedImagePaintable = img.clone();
    cv::cvtColor(transformedImagePaintable,transformedImagePaintable,CV_GRAY2BGR);

    debugPaintPoints(transformedImagePaintable, Scalar(0,0,255), laneMarkingsLeft);
    debugPaintPoints(transformedImagePaintable, Scalar(0,255,0), laneMarkingsCenter);
    debugPaintPoints(transformedImagePaintable, Scalar(255,0,0), laneMarkingsRight);
    debugPaintPoints(transformedImagePaintable, Scalar(0,255,255), laneMarkingsNotUsed);

    debugPaintPolynom(transformedImagePaintable, cv::Scalar(0, 255, 255), polyLeft, minYPolyRoi, maxYRoi);
    debugPaintPolynom(transformedImagePaintable, cv::Scalar(255, 255, 0), polyCenter, minYPolyRoi, maxYRoi);
    debugPaintPolynom(transformedImagePaintable, cv::Scalar(255, 0, 255), polyRight, minYPolyRoi, maxYRoi);

    cv::Point2d p1l(defaultXLeft,minYPolyRoi);
    cv::Point2d p2l(defaultXLeft,maxYRoi-1);
    cv::line(transformedImagePaintable,p1l,p2l,cv::Scalar(0,0,255));

    cv::Point2d p1c(defaultXCenter,minYPolyRoi);
    cv::Point2d p2c(defaultXCenter,maxYRoi-1);
    cv::line(transformedImagePaintable,p1c,p2c,cv::Scalar(0,255,0));

    cv::Point2d p1r(defaultXRight,minYPolyRoi);
    cv::Point2d p2r(defaultXRight,maxYRoi-1);
    cv::line(transformedImagePaintable,p1r,p2r,cv::Scalar(255,0,0));

    cv::Point2d p1(projImageHalfW-(roiBottomW/2),maxYRoi-1);
    cv::Point2d p2(projImageHalfW+(roiBottomW/2),maxYRoi-1);
    cv::Point2d p3(projImageHalfW+(roiTopW/2),minYPolyRoi);
    cv::Point2d p4(projImageHalfW-(roiTopW/2),minYPolyRoi);
    cv::line(transformedImagePaintable,p1,p2,cv::Scalar(0,200,0));
    cv::line(transformedImagePaintable,p2,p3,cv::Scalar(0,200,0));
    cv::line(transformedImagePaintable,p3,p4,cv::Scalar(0,200,0));
    cv::line(transformedImagePaintable,p4,p1,cv::Scalar(0,200,0));

    if (PUBLISH_IMAGES) {
        pubRGBImageMsg(transformedImagePaintable, image_publisher_lane_markings);
    }


    if (SHOW_GROUPED_LANE_MARKINGS_WINDOW) {
        cv::namedWindow("Grouped Lane Markings", WINDOW_NORMAL);
        cv::imshow("Grouped Lane Markings", transformedImagePaintable);
        cv::waitKey(1);
    }

    if (SAVE_FRAME_IMAGES) {
        debugWriteImg(transformedImagePaintable, "groupedLaneMarkings");
    }
}

void DebugUtils::drawRansacWindow(cv::Mat &img, NewtonPolynomial &polyLeft, NewtonPolynomial &polyCenter,
                                  NewtonPolynomial &polyRight, NewtonPolynomial &movedPolyLeft,
                                  NewtonPolynomial &movedPolyCenter, NewtonPolynomial &movedPolyRight) {

    if (!PUBLISH_IMAGES && !SHOW_RANSAC_WINDOW && !SAVE_FRAME_IMAGES) {
        return;
    }

    cv::Mat transformedImagePaintableRansac = img.clone();
    cv::cvtColor(transformedImagePaintableRansac,transformedImagePaintableRansac,CV_GRAY2BGR);

    debugPaintPolynom(transformedImagePaintableRansac, cv::Scalar(0, 0, 255), polyLeft, minYPolyRoi, maxYRoi);
    debugPaintPolynom(transformedImagePaintableRansac, cv::Scalar(0, 255, 0), polyCenter, minYPolyRoi, maxYRoi);
    debugPaintPolynom(transformedImagePaintableRansac, cv::Scalar(255, 0, 0), polyRight, minYPolyRoi, maxYRoi);

    debugPaintPolynom(transformedImagePaintableRansac, cv::Scalar(139, 0, 139), movedPolyLeft, minYPolyRoi, maxYRoi);
    debugPaintPolynom(transformedImagePaintableRansac, cv::Scalar(0, 0, 0), movedPolyCenter, minYPolyRoi, maxYRoi);
    debugPaintPolynom(transformedImagePaintableRansac, cv::Scalar(255, 120, 0), movedPolyRight, minYPolyRoi, maxYRoi);


    if (PUBLISH_IMAGES) {
        pubRGBImageMsg(transformedImagePaintableRansac, image_publisher_ransac);
    }

    if (SHOW_RANSAC_WINDOW) {
        cv::namedWindow("RANSAC results", WINDOW_NORMAL);
        cv::imshow("RANSAC results", transformedImagePaintableRansac);
        cv::waitKey(1);
    }

    if (SAVE_FRAME_IMAGES) {
        debugWriteImg(transformedImagePaintableRansac, "ransac");
    }
}

void DebugUtils::drawAngleWindow(Mat &img, bool polyDetectedRight, bool isPolyMovedRight, double lastAngle,
                                 double gradientForAngle, int angleAdjacentLeg, FuPoint<double> &pointForAngle,
                                 FuPoint<double> &movedPointForAngle) {
    frame++;

    if (!PUBLISH_IMAGES && !SHOW_ANGLE_WINDOW) {
        return;
    }

    Mat transformedImagePaintableLaneModel = img.clone();
    cvtColor(transformedImagePaintableLaneModel,transformedImagePaintableLaneModel,CV_GRAY2BGR);

    if (polyDetectedRight || isPolyMovedRight) {
        /*int r = lanePolynomial.getLastUsedPosition() == LEFT ? 255 : 0;
        int g = lanePolynomial.getLastUsedPosition() == CENTER ? 255 : 0;
        int b = lanePolynomial.getLastUsedPosition() == RIGHT ? 255 : 0;


        for(int i = minYPolyRoi; i < maxYRoi; i++) {
            cv::Point pointLoc = cv::Point(lanePolynomial.getLanePoly().at(i)+projImageHalfW, i);
            cv::circle(transformedImagePaintableLaneModel, pointLoc, 0, cv::Scalar(b,g,r), -1);
        }

        std::vector<FuPoint<int>> supps;
        switch (lanePolynomial.getLastUsedPosition()) {
            case LEFT:
                supps = supportersLeft;
                break;
            case CENTER:
                supps = supportersCenter;
                break;
            case RIGHT:
                supps = supportersRight;
                break;
            default:
                ROS_ERROR("No last used position");
                supps = supportersRight;
                break;
        };

        for(int i = 0; i < (int)supps.size(); i++) {
            FuPoint<int> supp = supps[i];
            cv::Point suppLoc = cv::Point(supp.getX(), supp.getY());
            cv::circle(transformedImagePaintableLaneModel, suppLoc, 1, cv::Scalar(b,g,r), -1);
        }*/

        cv::Point pointLoc = cv::Point(projImageHalfW, projImageH);
        cv::circle(transformedImagePaintableLaneModel, pointLoc, 2, cv::Scalar(0,0,255), -1);

        cv::Point anglePointLoc = cv::Point(sin(lastAngle * cLaneDetectionFu::PI / 180) * angleAdjacentLeg + projImageHalfW, projImageH - (cos(lastAngle * cLaneDetectionFu::PI / 180) * angleAdjacentLeg));
        cv::line(transformedImagePaintableLaneModel, pointLoc, anglePointLoc, cv::Scalar(255,255,255));

        cv::Point startNormalPoint = cv::Point(pointForAngle.getX(), pointForAngle.getY());
        cv::circle(transformedImagePaintableLaneModel, startNormalPoint, 2, cv::Scalar(100,100,100), -1);

        cv::Point targetPoint = cv::Point(movedPointForAngle.getX(), movedPointForAngle.getY());
        cv::circle(transformedImagePaintableLaneModel, targetPoint, 2, cv::Scalar(0,0,255), -1);

        /*cv::Point adjacentLegPoint = cv::Point(projImageHalfW, proj_image_h - adjacentLeg);
        cv::line(transformedImagePaintableLaneModel, pointLoc, adjacentLegPoint, cv::Scalar(255,0,0));

        cv::Point oppositeLegPoint = cv::Point(projImageHalfW + oppositeLeg, proj_image_h - adjacentLeg);
        cv::line(transformedImagePaintableLaneModel, adjacentLegPoint, oppositeLegPoint, cv::Scalar(0,255,0));*/

        double m = -gradientForAngle;

        double n = pointForAngle.getY() - m * pointForAngle.getX();
        double x = 10;
        double y = m * x + n;

        cv::Point endNormalPoint = cv::Point(x, y);
        cv::line(transformedImagePaintableLaneModel, startNormalPoint, endNormalPoint, cv::Scalar(0,0,0));

    } else {
        cv::Point pointLoc = cv::Point(5, 5);
        cv::circle(transformedImagePaintableLaneModel, pointLoc, 3, cv::Scalar(0,0,255), 0);
    }

    if (PUBLISH_IMAGES) {
        pubRGBImageMsg(transformedImagePaintableLaneModel, image_publisher);
    }

    if (SHOW_ANGLE_WINDOW) {
        cv::namedWindow("Lane polynomial", WINDOW_NORMAL);
        cv::imshow("Lane polynomial", transformedImagePaintableLaneModel);
        cv::waitKey(1);
    }
}

void DebugUtils::pubRGBImageMsg(cv::Mat& rgb_mat, image_transport::CameraPublisher publisher) {
    sensor_msgs::ImagePtr rgb_img(new sensor_msgs::Image);

    rgb_img->header.seq = head_sequence_id;
    rgb_img->header.stamp = head_time_stamp;
    rgb_img->header.frame_id = rgb_frame_id;

    rgb_img->width = rgb_mat.cols;
    rgb_img->height = rgb_mat.rows;

    rgb_img->encoding = sensor_msgs::image_encodings::BGR8;
    rgb_img->is_bigendian = 0;

    int step = sizeof(unsigned char) * 3 * rgb_img->width;
    int size = step * rgb_img->height;
    rgb_img->step = step;
    rgb_img->data.resize(size);
    memcpy(&(rgb_img->data[0]), rgb_mat.data, size);

    rgb_camera_info->header.frame_id = rgb_frame_id;
    rgb_camera_info->header.stamp = head_time_stamp;
    rgb_camera_info->header.seq = head_sequence_id;

    publisher.publish(rgb_img, rgb_camera_info);
}

void DebugUtils::debugPaintPolynom(cv::Mat &m, cv::Scalar color, NewtonPolynomial &p, int start, int end) {
    cv::Point point;
    for (int i = start; i < end; i++) {
        point = cv::Point(p.at(i), i);
        cv::circle(m, point, 0, color, -1);
    }
}

void DebugUtils::debugPaintPoints(cv::Mat &m, cv::Scalar color, vector<FuPoint<int>> &points) {
    for(FuPoint<int> point : points) {
        cv::Point pointLoc = cv::Point(point.getX(), point.getY());
        cv::circle(m, pointLoc, 1, color, -1);
    }
}

void DebugUtils::debugWriteImg(cv::Mat &image, string folder) {
    stringstream img;
    img << "./" << folder << "/" << frame << ".jpg";
    cv::imwrite(img.str(), image);
}