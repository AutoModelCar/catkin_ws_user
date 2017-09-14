#include "DebugUtils.h"

using namespace std;

DebugUtils() {
}

void paintOutputRoi(cv::Mat img, vector<vector<EdgePoint>> edges) {
    cv::Mat transformedImagePaintable = img.clone();
    cv::cvtColor(transformedImagePaintable,transformedImagePaintable,CV_GRAY2BGR);
    for(int i = 0; i < (int)edges.size(); i++)
    {
        for(int j=0; j < edges[i].size(); j++) {
            FuPoint<int> edge = edges[i][j].getImgPos();
            cv::Point edgeLoc = cv::Point(edge.getX(), edge.getY());
            cv::circle(transformedImagePaintable,edgeLoc,1,cv::Scalar(0,0,edges[i][j].getValue()),-1);
        }
    }

    /*cv::Point2d p1(proj_image_w_half-(roi_bottom_w/2),maxYRoi-1);
    cv::Point2d p2(proj_image_w_half+(roi_bottom_w/2),maxYRoi-1);
    cv::Point2d p3(proj_image_w_half+(roi_top_w/2),minYPolyRoi);
    cv::Point2d p4(proj_image_w_half-(roi_top_w/2),minYPolyRoi);
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