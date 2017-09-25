/**
Copyright (c) 
Audi Autonomous Driving Cup. All rights reserved.
 
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: �This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS �AS IS� AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

**/
// Christoph Brickl, AEV:

/*! \brief cSpurerkennung
 *         
�	Zur Erkennung einer Spur wird das RGB Bild der Asus Xtion eingelesen und verarbeitet
�	Weiteres Vorgehen:
�	Zuschneiden des Orginal Bildes auf die eingestellte Gr��e
�	Erzeugen eines Graustufen Bildes
�	Anwenden eines Schwellwertfilters
�	Kantendedektion
�	Suchen nach Kanten auf den eingestellten cm-Marken
�	Auswerten der gefundenen Punkte ob sinnvolle Linie erstellt werden kann
�	Anzeigen der Linie mit Hilfe der GLC

 */

#ifndef _LaneDetection_FILTER_HEADER_
#define _LaneDetection_FILTER_HEADER_

#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/video/background_segm.hpp>

#include <stdlib.h>
#include <sys/stat.h>

#include "tools/LineSegment.h"
#include "tools/Edges.h"
#include "tools/NewtonPolynomial.h"
#include "tools/LanePolynomial.h"
#include "tools/enums.h"
#include "tools/IPMapper.h"

#include <dynamic_reconfigure/server.h>
#include <line_detection_fu/LaneDetectionConfig.h>

class cLaneDetectionFu {
private:

    /**
     * Ros parameters (values can be changed in launch file)
     */

    int camW;
    int camH;
    int projYStart;
    int projImageH;
    int projImageW;
    int projImageWHalf;
    int projImageHorizontalOffset;
    int roiTopW;

    int roiBottomW;

    int scanlinesVerticalDistance;
    int scanlinesMaxCount;

    int gradientThreshold;
    int nonMaxWidth;

    int polyY1;
    int polyY2;
    int polyY3;

    int laneMarkingSquaredThreshold;

    int angleAdjacentLeg;
    int maxAngleDiff;

    // The maximum distance of a point to a polynomial so that it counts as a supporter.
    int maxDistance;

    /**
     * The horizontal distance to the last detected polynomial, within lane
     * markings have to lie to be considered for the detection of the next
     * polynomial. The width of the polynomial region of interest is two times
     * this distance.
     */
    int interestDistancePoly;

    /**
     * The horizontal distance to the default line, within lane markings have to
     * lie to be considered for the detection of a polynomial. The width of the
     * default region of interest is two times this distance.
     */
    int interestDistanceDefault;

    // The minimal y of the ROIs. Points with smaller y-Values are not used in RANSAC.
    int maxYRoi;

    // The maximal y of default ROIs. Points with bigger y-Values are not used.
    int minYDefaultRoi;

    // The maximal y of the polynomial ROIs. Points with bigger y-Values are not used.
    int minYPolyRoi;

    /**
     * The minimal proportion of supporters of all points within a ROI.
     * Polynomials with lower proportions are discarded.
     */
    double proportionThreshould;

    // Number of RANSAC iterations
    int iterationsRansac;


    /*
     *   Non-Ros-parameter variables:
     */


    // the node handle
    ros::NodeHandle nh_;

    // Node handle in the private namespace
    ros::NodeHandle priv_nh_;

    // subscribers
    ros::Subscriber read_images_;

    // publishers
    ros::Publisher publishAngle;
    image_transport::CameraPublisher imagePublisher;
    image_transport::CameraPublisher imagePublisherRansac;
    image_transport::CameraPublisher imagePublisherLaneMarkings;

    IPMapper ipMapper;

    std::string cameraName;

    // scanelines contain detected edges
    vector<vector<LineSegment<int>>> scanlines;

    // The lane marking polynomials detected in the current picture.
    NewtonPolynomial polyLeft;
    NewtonPolynomial polyCenter;
    NewtonPolynomial polyRight;

    /**
     * Horizontal relative positions of the default lane marking lines.

     *
     * These lines are situated in a position, where the lane markings of a
     * straight lane would show up in the relative coordinate system with the
     * car standing in the center of the right lane.
     */
    int defaultXLeft = 0;
    int defaultXCenter = 0;
    int defaultXRight = 0;

    /**
     * Flags to determine if a valid polynomial was detected in the last frame
     * and therefore the polynomial ROI should be used or if no polynomial could
     * be detected and the default ROI is used.
     */
    bool polyDetectedLeft;
    bool polyDetectedCenter;
    bool polyDetectedRight;

    /**
     * Pairs for saving the best lane polynomials produced during RANSAC
     *
     * first : current best NewtonPolynomial
     * second: proportion of supporters of used lane marking points (quality)
     */
    std::pair<NewtonPolynomial, double> bestPolyLeft;
    std::pair<NewtonPolynomial, double> bestPolyCenter;
    std::pair<NewtonPolynomial, double> bestPolyRight;


    /**
     * Lists containing the lane marking points selected for detecting the lane
     * polynomials during RANSAC
     */
    std::vector<FuPoint<int>> laneMarkingsLeft;
    std::vector<FuPoint<int>> laneMarkingsCenter;
    std::vector<FuPoint<int>> laneMarkingsRight;
    std::vector<FuPoint<int>> laneMarkingsNotUsed;

    // Vectors containing the supporters of the best polynomial
    std::vector<FuPoint<int>> supportersLeft;
    std::vector<FuPoint<int>> supportersCenter;
    std::vector<FuPoint<int>> supportersRight;

    // The polynomials detected on the previous picture
    NewtonPolynomial prevPolyLeft;
    NewtonPolynomial prevPolyCenter;
    NewtonPolynomial prevPolyRight;

    /**
     * The polynomials not detected on the current picture and generated by
     * shifting the detected polynomials accordingly
     */
    NewtonPolynomial movedPolyLeft;
    NewtonPolynomial movedPolyCenter;
    NewtonPolynomial movedPolyRight;

    /**
     * Booleans to denote whether a polynomial wasn't detected and had to be
     * shifted to the appropriate location
     */
    bool isPolyMovedRight = false;
    bool isPolyMovedCenter = false;
    bool isPolyMovedLeft = false;

    // Published angle in last frame
    double lastAngle;

    // Point on right polynomial (or shifted right polynomial) at x=angleAdjacentLeg
    FuPoint<double> pointForAngle;

    /* 
     * Result of moving pointForAngle half of laneWidth in direction of normal vector to left.
     * This is the target point where the car will move to.
     */
    FuPoint<double> movedPointForAngle;

    // Gradient of normal vector of right (shifted) polynomial at x=angleAdjacentLeg
    double gradientForAngle;

    // The default width between two lanes
    double laneWidth = 45.f;


    /**
     * Debugging methods for cleaner code in the ProcessInput() function
     */

    void drawEdgeWindow(cv::Mat &img, std::vector<std::vector<EdgePoint>> edges);

    void drawLaneMarkingsWindow(cv::Mat &img, std::vector<FuPoint<int>> &laneMarkings);

    void drawGroupedLaneMarkingsWindow(cv::Mat &img);

    void drawRansacWindow(cv::Mat &img);

    void drawAngleWindow(cv::Mat &img);

    void pubRGBImageMsg(cv::Mat &rgb_mat, image_transport::CameraPublisher publisher);

    void debugPaintPolynom(cv::Mat &m, cv::Scalar color, NewtonPolynomial &p, int start, int end);

    void debugPaintPoints(cv::Mat &m, cv::Scalar color, std::vector<FuPoint<int>> &points);

    void debugWriteImg(cv::Mat &image, std::string folder);

public:

    cLaneDetectionFu(ros::NodeHandle nh);

    virtual ~cLaneDetectionFu();

    void ProcessInput(const sensor_msgs::Image::ConstPtr &msg);

    void pubAngle();

    std::vector<std::vector<LineSegment<int>>> getScanlines();

    std::vector<std::vector<EdgePoint> > scanImage(cv::Mat image);

    std::vector<FuPoint<int>> extractLaneMarkings(const std::vector<std::vector<EdgePoint>> &edges);

    void buildLaneMarkingsLists(const std::vector<FuPoint<int>> &laneMarkings);

    void findLanePositions(vector<FuPoint<int>> &laneMarkings);

    void shiftPoint(FuPoint<double> &p, double m, double offset, int x, int y);

    void shiftPolynomial(NewtonPolynomial &f, NewtonPolynomial &g, double offset);

    void generateMovedPolynomials();

    bool isInRange(FuPoint<int> &lanePoint, FuPoint<int> &p);

    int horizDistanceToDefaultLine(ePosition &line, FuPoint<int> &p);

    int horizDistanceToPolynomial(NewtonPolynomial &poly, FuPoint<int> &p);

    bool isInDefaultRoi(ePosition position, FuPoint<int> &p);

    bool isInPolyRoi(NewtonPolynomial &poly, FuPoint<int> &p);

    void ransac();

    bool ransacInternal(ePosition position,
                        std::vector<FuPoint<int>> &laneMarkings,
                        std::pair<NewtonPolynomial, double> &bestPoly, NewtonPolynomial &poly,
                        std::vector<FuPoint<int>> &supporters, NewtonPolynomial &prevPoly);

    bool polyValid(ePosition, NewtonPolynomial, NewtonPolynomial);

    bool isSimilar(const NewtonPolynomial &poly1, const NewtonPolynomial &poly2);

    int horizDistance(FuPoint<int> &p1, FuPoint<int> &p2);

    double gradient(double, double, double, std::vector<double>);

    double intersection(FuPoint<double> &, double &, std::vector<FuPoint<int>> &,
                        std::vector<double> &);

    double nextGradient(double, NewtonPolynomial &,
                        std::vector<FuPoint<int>> &, std::vector<FuPoint<int>> &,
                        std::vector<double>, std::vector<double>, double);

    bool gradientsSimilar(double &, double &);

    ePosition maxProportion();

    void config_callback(line_detection_fu::LaneDetectionConfig &config, uint32_t level);

};

#endif 
