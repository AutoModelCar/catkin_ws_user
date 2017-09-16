#include "laneDetection.h"

using namespace std;

// publish ransac and grouped lane frames to show it in rviz
const bool PUBLISH_IMAGES = true;
// save frames as images in ~/.ros/
const bool SAVE_FRAME_IMAGES = true;

// show windows with results of each step in pipeline of one frame
const bool SHOW_EDGE_WINDOW = true;
const bool SHOW_LANE_MARKINGS_WINDOW = true;
const bool SHOW_GROUPED_LANE_MARKINGS_WINDOW = true;
const bool SHOW_RANSAC_WINDOW = true;
const bool SHOW_ANGLE_WINDOW = true;

// try kernel width 5 for now
const static int g_kernel1DWidth = 5;

// params for pubRGBImageMsg
unsigned int head_sequence_id = 0;
ros::Time head_time_stamp;
std::string rgb_frame_id = "_rgb_optical_frame";
sensor_msgs::CameraInfoPtr rgb_camera_info;

// frame number for saving image files
std::size_t frame = 0;

const uint32_t MY_ROS_QUEUE_SIZE = 1;
const double PI = 3.14159265;

cLaneDetectionFu::cLaneDetectionFu(ros::NodeHandle nh) : nh_(nh), priv_nh_("~") {
    std::string node_name = ros::this_node::getName();

    ROS_ERROR("Node name: %s", node_name.c_str());

    priv_nh_.param<std::string>(node_name + "/camera_name", camera_name, "/usb_cam/image_raw");

    priv_nh_.param<int>(node_name + "/camW", cam_w, 640);
    priv_nh_.param<int>(node_name + "/camH", cam_h, 480);
    priv_nh_.param<int>(node_name + "/projYStart", projYStart, 60);
    priv_nh_.param<int>(node_name + "/projImageH", projImageH, 100);
    priv_nh_.param<int>(node_name + "/projImageW", projImageW, 150);
    priv_nh_.param<int>(node_name + "/projImageHorizontalOffset", projImageHorizontalOffset, 0);
    priv_nh_.param<int>(node_name + "/roiTopW", roiTopW, 150);
    priv_nh_.param<int>(node_name + "/roiBottomW", roiBottomW, 57);

    priv_nh_.param<int>(node_name + "/maxYRoi", maxYRoi, 100);
    priv_nh_.param<int>(node_name + "/minYDefaultRoi", minYDefaultRoi, 10);
    priv_nh_.param<int>(node_name + "/minYPolyRoi", minYPolyRoi, 11);

    priv_nh_.param<int>(node_name + "/interestDistancePoly", interestDistancePoly, 10);
    priv_nh_.param<int>(node_name + "/interestDistanceDefault", interestDistanceDefault, 5);

    priv_nh_.param<int>(node_name + "/iterationsRansac", iterationsRansac, 38);
    priv_nh_.param<double>(node_name + "/proportionThreshould", proportionThreshould, 0.68);

    priv_nh_.param<int>(node_name + "/gradientThreshold", gradientThreshold, 10);
    priv_nh_.param<int>(node_name + "/nonMaxWidth", nonMaxWidth, 16);
    priv_nh_.param<int>(node_name + "/laneMarkingSquaredThreshold", laneMarkingSquaredThreshold, 36);

    priv_nh_.param<int>(node_name + "/angleAdjacentLeg", angleAdjacentLeg, 18);

    priv_nh_.param<int>(node_name + "/scanlinesVerticalDistance", scanlinesVerticalDistance, 2);
    priv_nh_.param<int>(node_name + "/scanlinesMaxCount", scanlinesMaxCount, 100);

    priv_nh_.param<int>(node_name + "/maxAngleDiff", maxAngleDiff, 14);

    priv_nh_.param<int>(node_name + "/polyY1", polyY1, 20);
    priv_nh_.param<int>(node_name + "/polyY2", polyY2, 50);
    priv_nh_.param<int>(node_name + "/polyY3", polyY3, 70);


    double f_u;
    double f_v;
    double c_u;
    double c_v;
    double camDeg;
    double camHeight;
    int cam_h_half = cam_h / 2;

    priv_nh_.param<double>(node_name + "/f_u", f_u, 624.650635);
    priv_nh_.param<double>(node_name + "/f_v", f_v, 626.987244);
    priv_nh_.param<double>(node_name + "/c_u", c_u, 309.703230);
    priv_nh_.param<double>(node_name + "/c_v", c_v, 231.473613);
    priv_nh_.param<double>(node_name + "/camDeg", camDeg, 13.0);
    priv_nh_.param<double>(node_name + "/camHeight", camHeight, 36.2);

    ipMapper = IPMapper(cam_w, cam_h_half, f_u, f_v, c_u, c_v, camDeg, camHeight);

    proj_image_w_half = projImageW / 2;

    polyDetectedLeft = false;
    polyDetectedCenter = false;
    polyDetectedRight = false;

    bestPolyLeft = std::make_pair(NewtonPolynomial(), 0);
    bestPolyCenter = std::make_pair(NewtonPolynomial(), 0);
    bestPolyRight = std::make_pair(NewtonPolynomial(), 0);

    laneMarkingsLeft = std::vector<FuPoint<int>>();
    laneMarkingsCenter = std::vector<FuPoint<int>>();
    laneMarkingsRight = std::vector<FuPoint<int>>();
    laneMarkingsNotUsed = std::vector<FuPoint<int>>();

    polyLeft = NewtonPolynomial();
    polyCenter = NewtonPolynomial();
    polyRight = NewtonPolynomial();

    supportersLeft = std::vector<FuPoint<int>>();
    supportersCenter = std::vector<FuPoint<int>>();
    supportersRight = std::vector<FuPoint<int>>();

    prevPolyLeft = NewtonPolynomial();
    prevPolyCenter = NewtonPolynomial();
    prevPolyRight = NewtonPolynomial();

    pointsLeft = std::vector<FuPoint<int>>();
    pointsCenter = std::vector<FuPoint<int>>();
    pointsRight = std::vector<FuPoint<int>>();

    movedPolyLeft = NewtonPolynomial();
    movedPolyCenter = NewtonPolynomial();
    movedPolyRight = NewtonPolynomial();

    movedPointsRight = std::vector<FuPoint<int>>();
    movedPointForAngle = FuPoint<double>();
    pointForAngle = FuPoint<double>();

    maxDistance = 1;

    lastAngle = 0;

    read_images_ = nh.subscribe(nh_.resolveName(camera_name), MY_ROS_QUEUE_SIZE, &cLaneDetectionFu::ProcessInput, this);

    publish_angle = nh.advertise<std_msgs::Float32>("/lane_model/angle", MY_ROS_QUEUE_SIZE);

    //from camera properties and ROI etc we get scanlines (=line segments, úsečky)
    //these line segments are lines in image on whose we look for edges
    //the outer vector represents rows on image, inner vector is vector of line segments of one row, usualy just one line segment
    //we should generate this only once in the beginning! or even just have it pregenerated for our cam
    scanlines = getScanlines();

    head_time_stamp = ros::Time::now();

    if (PUBLISH_IMAGES) {
        image_transport::ImageTransport image_transport(nh);

        image_publisher = image_transport.advertiseCamera("/lane_model/lane_model_image", MY_ROS_QUEUE_SIZE);
        image_publisher_ransac = image_transport.advertiseCamera("/lane_model/ransac", MY_ROS_QUEUE_SIZE);
        image_publisher_lane_markings = image_transport.advertiseCamera("/lane_model/lane_markings", MY_ROS_QUEUE_SIZE);

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

cLaneDetectionFu::~cLaneDetectionFu() {
}

void cLaneDetectionFu::ProcessInput(const sensor_msgs::Image::ConstPtr &msg) {
    // clear some stuff from the last cycle
    bestPolyLeft = std::make_pair(NewtonPolynomial(), 0);
    bestPolyCenter = std::make_pair(NewtonPolynomial(), 0);
    bestPolyRight = std::make_pair(NewtonPolynomial(), 0);

    supportersLeft.clear();
    supportersCenter.clear();
    supportersRight.clear();

    //use ROS image_proc or opencv instead of ip mapper?

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat image = cv_ptr->image.clone();
    Mat cut_image = image(cv::Rect(0, cam_h * 0.25f, cam_w, cam_h * 0.75f));

    Mat remapped_image = ipMapper.remap(cut_image);

    cv::Mat transformedImage = remapped_image(cv::Rect((cam_w / 2) - proj_image_w_half + projImageHorizontalOffset,
                                                       projYStart, projImageW, projImageH)).clone();

    cv::flip(transformedImage, transformedImage, 0);

    //scanlines -> edges (in each scanline we find maximum and minimum of kernel fn ~= where the edge is)
    //this is where we use input image!
    vector<vector<EdgePoint>> edges = cLaneDetectionFu::scanImage(transformedImage);

    cv::Mat transformedImagePaintable;
    drawEdgeWindow(transformedImage, edges);

    //edges -> lane markings
    vector<FuPoint<int>> laneMarkings = cLaneDetectionFu::extractLaneMarkings(edges);
    drawLaneMarkingsWindow(transformedImage, laneMarkings);

    findLanePositions(laneMarkings);

    // start actual execution
    buildLaneMarkingsLists(laneMarkings);
    drawGroupedLaneMarkingsWindow(transformedImage);

    ransac();
    generateMovedPolynomials();
    drawRansacWindow(transformedImage);

    pubAngle();
    drawAngleWindow(transformedImage);
}

/*
 *      Pipeline:
 */

/**
 * Compute scanlines. Each may consist of multiple segments, split at regions
 * that should not be inspected by the kernel.
 * @param side
 * @return vector of segments of scanlines, walk these segments with the kernel
 */
vector<vector<LineSegment<int>>> cLaneDetectionFu::getScanlines() {
    vector<vector<LineSegment<int>>> scanlines;

    vector<cv::Point> checkContour;
    checkContour.push_back(cv::Point(proj_image_w_half - (roiBottomW / 2), maxYRoi - 1));
    checkContour.push_back(cv::Point(proj_image_w_half + (roiBottomW / 2), maxYRoi - 1));
    checkContour.push_back(cv::Point(proj_image_w_half + (roiTopW / 2), minYPolyRoi));
    checkContour.push_back(cv::Point(proj_image_w_half - (roiTopW / 2), minYPolyRoi));

    int scanlineStart = 0;
    int scanlineEnd = projImageW;

    int segmentStart = -1;
    vector<LineSegment<int>> scanline;
    //i = y; j = x;
    for (int i = 1;
         (i / scanlinesVerticalDistance) < scanlinesMaxCount && i <= projImageH;
         i += scanlinesVerticalDistance) {
        scanline = vector<LineSegment<int>>();

        // walk along line
        for (int j = scanlineStart; j <= scanlineEnd; j++) {
            bool isInside = pointPolygonTest(checkContour, cv::Point(j, i), false) >= 0;

            // start new scanline segment
            if (isInside && j < scanlineEnd) {
                if (segmentStart == -1) segmentStart = j;
                // found end of scanline segment, reset start
            } else if (segmentStart != -1) {
                scanline.push_back(
                        LineSegment<int>(
                                FuPoint<int>(segmentStart, i),
                                FuPoint<int>(j - 1, i)
                        )
                );

                segmentStart = -1;
            }
        }
        // push segments found
        if (scanline.size()) {
            scanlines.push_back(scanline);
        }
    }
    return scanlines;
}

/**
 * Walk with prewitt/sobel kernel along all scanlines.
 * @param image
 * @return All edgePoints on side, sorted by scanlines.
 */
vector<vector<EdgePoint>> cLaneDetectionFu::scanImage(cv::Mat image) {
    //ROS_INFO_STREAM("scanImage() - " << scanlines.size() << " scanlines.");
    vector<vector<EdgePoint>> edgePoints;

    //const Image &image = getImage();
    //const ImageDimensions &imgDim = getImageDimensions();
    //const OmnidirectionalCameraMatrix &cameraMatrix = getOmnidirectionalCameraMatrix();

    // scanline length can maximal be image height/width
    int scanlineMaxLength = image.cols;

    // store kernel results on current scanline in here
    vector<int> scanlineVals(scanlineMaxLength, 0);

    // walk over all scanlines
    for (auto scanline : scanlines) {
        // set all brightness values on scanline to 0;
        std::fill(scanlineVals.begin(), scanlineVals.end(), 0);
        int offset = 0;
        if (scanline.size()) {
            offset = scanline.front().getStart().getY();
        }

        // scanline consisting of multiple segments
        // walk over each but store kernel results for whole scanline
        for (auto segment : scanline) {
            int start = segment.getStart().getX();
            int end = segment.getEnd().getX();

            // walk along segment
            for (int i = start; i < end - g_kernel1DWidth; i++) {
                int sum = 0;

                //cv::Mat uses ROW-major system -> .at(y,x)
                // use kernel width 5 and try sobel kernel
                sum -= image.at<uint8_t>(offset - 1, i);
                sum -= image.at<uint8_t>(offset - 1, i + 1);
                // kernel is 0
                sum += image.at<uint8_t>(offset - 1, i + 2);
                sum += image.at<uint8_t>(offset - 1, i + 4);

                sum -= 2 * image.at<uint8_t>(offset, i);
                sum -= 2 * image.at<uint8_t>(offset, i + 1);
                // kernel is 0
                sum += 2 * image.at<uint8_t>(offset, i + 2);
                sum += 2 * image.at<uint8_t>(offset, i + 4);

                sum -= image.at<uint8_t>(offset + 1, i);
                sum -= image.at<uint8_t>(offset + 1, i + 1);
                // kernel is 0
                sum += image.at<uint8_t>(offset + 1, i + 2);
                sum += image.at<uint8_t>(offset + 1, i + 4);


                // +4 because of sobel weighting
                sum = sum / (3 * g_kernel1DWidth + 4);
                //ROS_INFO_STREAM(sum << " is kernel sum.");
                if (std::abs(sum) > gradientThreshold) {
                    // set scanlineVals at center of kernel
                    scanlineVals[i + g_kernel1DWidth / 2] = sum;
                }
            }
        }

        // after walking over all segments of one scanline
        // do non-max-suppression
        // for both minima and maxima at same time
        // TODO: Jannis: find dryer way
        int indexOfLastMaximum = 0;
        int valueOfLastMaximum = 0;
        int indexOfLastMinimum = 0;
        int valueOfLastMinimum = 0;
        for (int i = 1; i < scanlineMaxLength - 1; i++) {
            // check if maximum
            if (scanlineVals[i] > 0) {
                if (scanlineVals[i] < scanlineVals[i - 1] or scanlineVals[i] < scanlineVals[i + 1]) {
                    scanlineVals[i] = 0;
                } else {
                    // this pixel can just survive if the next maximum is not too close
                    if (i - indexOfLastMaximum > nonMaxWidth) {
                        // this is a new maximum
                        indexOfLastMaximum = i;
                        valueOfLastMaximum = scanlineVals[i];
                    } else {
                        if (valueOfLastMaximum < scanlineVals[i]) {
                            // this is a new maximum
                            // drop the old maximum
                            scanlineVals[indexOfLastMaximum] = 0;
                            indexOfLastMaximum = i;
                            valueOfLastMaximum = scanlineVals[i];
                        } else {
                            scanlineVals[i] = 0;
                        }
                    }
                }
            }
            // check if minimum
            if (scanlineVals[i] < 0) {
                if (scanlineVals[i] > scanlineVals[i - 1] or scanlineVals[i] > scanlineVals[i + 1]) {
                    scanlineVals[i] = 0;
                } else {
                    // this pixel can just survive if the next minimum is not too close
                    if (i - indexOfLastMinimum > nonMaxWidth) {
                        // this is a new minimum
                        indexOfLastMinimum = i;
                        valueOfLastMinimum = scanlineVals[i];
                    } else {
                        if (valueOfLastMinimum > scanlineVals[i]) {
                            // this is a new maximum
                            // drop the old maximum
                            scanlineVals[indexOfLastMinimum] = 0;
                            indexOfLastMinimum = i;
                            valueOfLastMinimum = scanlineVals[i];
                        } else {
                            scanlineVals[i] = 0;
                        }
                    }
                }
            }
        }
        // collect all the edgePoints for scanline
        vector<EdgePoint> scanlineEdgePoints;
        for (int i = 0; i < static_cast<int>(scanlineVals.size()); i++) {
            if (scanlineVals[i] != 0) {
                FuPoint<int> imgPos = FuPoint<int>(i, offset);

                FuPoint<Meter> relPos = FuPoint<Meter>();//offset, i);//cameraMatrix.transformToLocalCoordinates(imgPos);
                scanlineEdgePoints.push_back(EdgePoint(imgPos, relPos, scanlineVals[i]));
            }
        }
        edgePoints.push_back(std::move(scanlineEdgePoints));
    }
    // after walking along all scanlines
    // return edgePoints
    return edgePoints;
}


/**
 * uses Edges to extract lane markings
 */
std::vector<FuPoint<int>> cLaneDetectionFu::extractLaneMarkings(const std::vector<std::vector<EdgePoint>> &edges) {
    vector<FuPoint<int>> result;

    for (const auto &line : edges) {
        if (line.empty()) continue;

        for (
                auto edgePosition = line.begin(), nextEdgePosition = edgePosition + 1;
                nextEdgePosition != line.end();
                edgePosition = nextEdgePosition, ++nextEdgePosition
                ) {
            if (edgePosition->isPositive() and not nextEdgePosition->isPositive()) {
                FuPoint<int> candidateStartEdge = edgePosition->getImgPos();
                FuPoint<int> candidateEndEdge = nextEdgePosition->getImgPos();
                if ((candidateStartEdge - candidateEndEdge).squaredMagnitude() < laneMarkingSquaredThreshold) {
                    result.push_back(center(candidateStartEdge, candidateEndEdge));
                }
            }
        }
    }

    // sort the lane marking edge points
    std::sort(result.begin(), result.end(),
              [](FuPoint<int> a, FuPoint<int> b) {
                  return a.getY() > b.getY();
              });

    return result;
}

/**
 * Calculates x positions of the lanes. Lane markings of first 10 rows from the bottom
 * of the image are used. Car must start between right and center lane because all lane
 * marking points in left half of the image are considered as possible lane edge points
 * of center lane (analog: right half of image for right lane). Lane marking points in
 * range of other lane marking points are supporters because they form a line. The x-value
 * of found lane points with maximum supporters will be used. This ensures that non-lane
 * points are ignored. Start position of left lane is calculated after start positions
 * of center and right lanes are found.
 */
void cLaneDetectionFu::findLanePositions(vector<FuPoint<int>> &laneMarkings) {
    // defaultXLeft is calculated after center and right lane position is found
    if (defaultXLeft > 0) {
        return;
    }

    // counts how many lane marking points form a line with point in centerStart
    // at same index
    vector<int> centerSupporter;
    vector<int> rightSupporter;

    // possible start points of center lane
    vector<FuPoint<int> *> centerStart;
    vector<FuPoint<int> *> rightStart;

    for (int j = 0; j < laneMarkings.size(); j++) {
        FuPoint<int> *laneMarking = &laneMarkings.at(j);

        if (laneMarking->getY() > maxYRoi) {
            continue;
        }

        bool isSupporter = false;
        if (laneMarking->getX() < proj_image_w_half + projImageHorizontalOffset) {
            for (int i = 0; i < centerStart.size(); i++) {
                if (isInRange(*centerStart.at(i), *laneMarking)) {
                    isSupporter = true;
                    centerSupporter.at(i)++;

                    if (centerSupporter.at(i) > 5) {
                        goto defaultLineCalculation;
                    }

                    break;
                }
            }

            if (!isSupporter) {
                centerStart.push_back(laneMarking);
                centerSupporter.push_back(0);
            }
        } else {
            for (int i = 0; i < rightStart.size(); i++) {
                if (isInRange(*rightStart.at(i), *laneMarking)) {
                    isSupporter = true;
                    rightSupporter.at(i)++;

                    if (rightSupporter.at(i) > 5) {
                        goto defaultLineCalculation;
                    }

                    break;
                }
            }

            if (!isSupporter) {
                rightStart.push_back(laneMarking);
                rightSupporter.push_back(0);
            }
        }
    }

    defaultLineCalculation:

    // use x-value of lane marking point with most (and at least 3) supporters
    if (centerStart.size() > 0) {
        vector<int>::iterator maxCenterElement = max_element(centerSupporter.begin(), centerSupporter.end());

        if (*maxCenterElement > 3) {
            int position = distance(centerSupporter.begin(), maxCenterElement);
            defaultXCenter = centerStart.at(position)->getX();
        }
    }

    if (rightStart.size() > 0) {
        vector<int>::iterator maxRightElement = max_element(rightSupporter.begin(), rightSupporter.end());

        if (*maxRightElement > 3) {
            int position = distance(rightSupporter.begin(), maxRightElement);
            defaultXRight = rightStart.at(position)->getX();
        }
    }

    if (defaultXCenter > 0 && defaultXRight > 0) {
        laneWidth = defaultXRight - defaultXCenter;
        defaultXLeft = defaultXCenter - (int) laneWidth;
    }
}

/**
 * Creates three vectors of lane marking points out of the given lane marking
 * point vector.
 *
 * A point has to lie within the ROI of the previously detected lane polynomial
 * or within the default ROI, if no polynomial was detected.
 * The lists are the input data for the RANSAC algorithm.
 *
 * @param laneMarkings  a vector containing all detected lane markings
 */
void cLaneDetectionFu::buildLaneMarkingsLists(const std::vector<FuPoint<int>> &laneMarkings) {
    laneMarkingsLeft.clear();
    laneMarkingsCenter.clear();
    laneMarkingsRight.clear();
    laneMarkingsNotUsed.clear();

    for (FuPoint<int> laneMarking : laneMarkings) {

        // check if lane marking point is near to found lane poly of ransac

        if (polyDetectedRight) {
            if (isInPolyRoi(polyRight, laneMarking)) {
                laneMarkingsRight.push_back(laneMarking);
                continue;
            }
        }

        if (polyDetectedCenter) {
            if (isInPolyRoi(polyCenter, laneMarking)) {
                laneMarkingsCenter.push_back(laneMarking);
                continue;
            }
        }

        if (polyDetectedLeft) {
            if (isInPolyRoi(polyLeft, laneMarking)) {
                laneMarkingsLeft.push_back(laneMarking);
                continue;
            }
        }

        // check if lane marking point is near to moved poly of ransac

        if (movedPolyLeft.isInitialized()) {
            if (isInPolyRoi(movedPolyLeft, laneMarking)) {
                laneMarkingsLeft.push_back(laneMarking);
                continue;
            }
        }

        if (movedPolyCenter.isInitialized()) {
            if (isInPolyRoi(movedPolyCenter, laneMarking)) {
                laneMarkingsCenter.push_back(laneMarking);
                continue;
            }
        }

        if (movedPolyRight.isInitialized()) {
            if (isInPolyRoi(movedPolyRight, laneMarking)) {
                laneMarkingsRight.push_back(laneMarking);
                continue;
            }
        }

        // if ransac found a polynomial in last frame skip default lane comparison
        if (polyDetectedLeft || polyDetectedCenter || polyDetectedRight) {
            continue;
        }

        // no poly available from last frame, check if lane marking point is near to
        // default lane or near to already classified point (this way points are also
        // classified properly if car starts in a turn)

        if (laneMarkingsRight.size() > 0) {
            if (isInRange(laneMarkingsRight.at(laneMarkingsRight.size() - 1), laneMarking)) {
                laneMarkingsRight.push_back(laneMarking);
                continue;
            }
        }

        if (laneMarkingsCenter.size() > 0) {
            if (isInRange(laneMarkingsCenter.at(laneMarkingsCenter.size() - 1), laneMarking)) {
                laneMarkingsCenter.push_back(laneMarking);
                continue;
            }
        }

        if (laneMarkingsLeft.size() > 0) {
            if (isInRange(laneMarkingsLeft.at(laneMarkingsLeft.size() - 1), laneMarking)) {
                laneMarkingsLeft.push_back(laneMarking);
                continue;
            }
        }

        if (isInDefaultRoi(RIGHT, laneMarking)) {
            laneMarkingsRight.push_back(laneMarking);
            continue;
        }

        if (isInDefaultRoi(CENTER, laneMarking)) {
            laneMarkingsCenter.push_back(laneMarking);
            continue;
        }

        if (isInDefaultRoi(LEFT, laneMarking)) {
            laneMarkingsLeft.push_back(laneMarking);
            continue;
        }

        laneMarkingsNotUsed.push_back(laneMarking);
    }
}

/**
 * Starts the RANSAC algorithm for detecting each of the three lane marking
 * polynomials.
 */
void cLaneDetectionFu::ransac() {
    polyDetectedLeft = ransacInternal(LEFT, laneMarkingsLeft, bestPolyLeft,
                                      polyLeft, supportersLeft, prevPolyLeft, pointsLeft);

    polyDetectedCenter = ransacInternal(CENTER, laneMarkingsCenter,
                                        bestPolyCenter, polyCenter, supportersCenter, prevPolyCenter,
                                        pointsCenter);

    polyDetectedRight = ransacInternal(RIGHT, laneMarkingsRight, bestPolyRight,
                                       polyRight, supportersRight, prevPolyRight, pointsRight);
}

/**
 * Detects a polynomial with RANSAC in a given list of lane marking edge points.
 *
 * @param position      The position of the wanted polynomial
 * @param laneMarkings  A reference to the list of lane marking edge points
 * @param bestPoly      A reference to a pair containing the present best
 *                      detected polynomial and a value representing the fitting
 *                      quality called proportion
 * @param poly          A reference to the polynomial that gets detected
 * @param supporters    A reference to the supporter points of the present best
 *                      polynomial
 * @param prevPoly      A reference to the previous polynomial detected at this
 *                      position
 * @param points        A reference to the points selected for interpolating the
 *                      present best polynomial
 * @return              true if a polynomial could be detected and false when not
 */
bool cLaneDetectionFu::ransacInternal(ePosition position,
                                      std::vector<FuPoint<int>> &laneMarkings,
                                      std::pair<NewtonPolynomial, double> &bestPoly, NewtonPolynomial &poly,
                                      std::vector<FuPoint<int>> &supporters, NewtonPolynomial &prevPoly,
                                      std::vector<FuPoint<int>> &points) {

    if (laneMarkings.size() < 7) {
        prevPoly = poly;
        poly.clear();
        return false;
    }

    std::vector<FuPoint<int>> tmpSupporters = std::vector<FuPoint<int>>();

    // vectors for points selected from the bottom, mid and top of the sorted
    // point vector
    std::vector<FuPoint<int>> markings1 = std::vector<FuPoint<int>>();
    std::vector<FuPoint<int>> markings2 = std::vector<FuPoint<int>>();
    std::vector<FuPoint<int>> markings3 = std::vector<FuPoint<int>>();

    bool highEnoughY = false;

    // Points are selected from the bottom, mid and top. The selection regions
    // are spread apart for better results during RANSAC
    for (std::vector<FuPoint<int>>::size_type i = 0; i != laneMarkings.size(); i++) {
        if (i < double(laneMarkings.size()) / 7) {
            markings1.push_back(laneMarkings[i]);
        } else if (i >= (double(laneMarkings.size()) / 7) * 3
                   && i < (double(laneMarkings.size()) / 7) * 4) {
            markings2.push_back(laneMarkings[i]);
        } else if (i >= (double(laneMarkings.size()) / 7) * 6) {
            markings3.push_back(laneMarkings[i]);
        }

        if (laneMarkings[i].getY() > 5) {
            highEnoughY = true;
        }
    }

    //what is this for?
    if (position == CENTER) {
        if (!highEnoughY) {
            prevPoly = poly;
            poly.clear();
            return false;
        }
    }

    // save the polynomial from the previous picture
    prevPoly = poly;

    for (int i = 0; i < iterationsRansac; i++) {

        // randomly select 3 different lane marking points from bottom, mid and
        // top
        int pos1 = rand() % markings1.size();
        int pos2 = rand() % markings2.size();
        int pos3 = rand() % markings3.size();

        FuPoint<int> p1 = markings1[pos1];
        FuPoint<int> p2 = markings2[pos2];
        FuPoint<int> p3 = markings3[pos3];

        double p1X = p1.getX();
        double p1Y = p1.getY();
        double p2X = p2.getX();
        double p2Y = p2.getY();
        double p3X = p3.getX();
        double p3Y = p3.getY();

        // clear poly for reuse
        poly.clear();

        // create a polynomial with the selected points
        poly.addData(p1X, p1Y);
        poly.addData(p2X, p2Y);
        poly.addData(p3X, p3Y);

        // check if this polynomial is not useful
        if (!polyValid(position, poly, prevPoly)) {
            poly.clear();
            continue;
        }

        // count the supporters and save them for debugging
        int count1 = 0;
        int count2 = 0;
        int count3 = 0;

        // find the supporters
        tmpSupporters.clear();

        for (FuPoint<int> p : markings1) {
            if (horizDistanceToPolynomial(poly, p) <= maxDistance) {
                count1++;
                tmpSupporters.push_back(p);
            }
        }

        for (FuPoint<int> p : markings2) {
            if (horizDistanceToPolynomial(poly, p) <= maxDistance) {
                count2++;
                tmpSupporters.push_back(p);
            }
        }

        for (FuPoint<int> p : markings3) {
            if (horizDistanceToPolynomial(poly, p) <= maxDistance) {
                count3++;
                tmpSupporters.push_back(p);
            }
        }

        if (count1 == 0 || count2 == 0 || count3 == 0) {
            poly.clear();
            //DEBUG_TEXT(dbgMessages, "Poly had no supporters in one of the regions");
            continue;
        }

        // calculate the proportion of supporters of all lane markings
        double proportion = (double(count1) / markings1.size()
                             + double(count2) / markings2.size()
                             + 3 * (double(count3) / markings3.size())) / 5;

        if (proportion < proportionThreshould) {
            poly.clear();
            //DEBUG_TEXT(dbgMessages, "Poly proportion was smaller than threshold");
            continue;
        }

        // check if poly is better than bestPoly
        if (proportion > bestPoly.second) {
            bestPoly = std::make_pair(poly, proportion);
            supporters = tmpSupporters;

            points.clear();
            points.push_back(p1);
            points.push_back(p2);
            points.push_back(p3);
        }
    }

    poly = bestPoly.first;

    if (poly.getDegree() == -1) {
        return false;
    }

    return true;
}

void cLaneDetectionFu::generateMovedPolynomials() {
    movedPolyLeft.clear();
    movedPolyCenter.clear();
    movedPolyRight.clear();
    movedPointsRight.clear();

    isPolyMovedLeft = false;
    isPolyMovedCenter = false;
    isPolyMovedRight = false;

    if ((!polyDetectedLeft && !polyDetectedCenter && !polyDetectedRight)
        || (polyDetectedLeft && polyDetectedCenter && polyDetectedRight)) {
        return;
    }

    FuPoint<double> pointLeft1 = FuPoint<double>();
    FuPoint<double> pointLeft2 = FuPoint<double>();
    FuPoint<double> pointLeft3 = FuPoint<double>();

    FuPoint<double> pointCenter1 = FuPoint<double>();
    FuPoint<double> pointCenter2 = FuPoint<double>();
    FuPoint<double> pointCenter3 = FuPoint<double>();

    FuPoint<double> pointRight1 = FuPoint<double>();
    FuPoint<double> pointRight2 = FuPoint<double>();
    FuPoint<double> pointRight3 = FuPoint<double>();

    double m1 = 0;
    double m2 = 0;
    double m3 = 0;

    NewtonPolynomial usedPoly;

    /*
     * Depending on the sign of the gradient of the poly at the different
     * x-values and depending on which position we are, we have to add or
     * subtract the expected distance to the respective lane polynomial, to get
     * the wanted points.
     *
     * The calculation is done for the x- and y-components of the points
     * separately using the trigonometric ratios of right triangles and the fact
     * that arctan of some gradient equals its angle to the x-axis in degree.
     */
    if (polyDetectedRight && !polyDetectedCenter) {
        usedPoly = polyRight;
        m1 = gradient(pointsRight.at(0).getY(), pointsRight, usedPoly.getCoefficients());
        m2 = gradient(pointsRight.at(1).getY(), pointsRight, usedPoly.getCoefficients());
        m3 = gradient(pointsRight.at(2).getY(), pointsRight, usedPoly.getCoefficients());

        isPolyMovedCenter = true;

        shiftPoint(pointCenter1, m1, -laneWidth, pointsRight.at(0));
        shiftPoint(pointCenter2, m2, -laneWidth, pointsRight.at(1));
        shiftPoint(pointCenter3, m3, -laneWidth, pointsRight.at(2));

        if (!polyDetectedLeft) {
            isPolyMovedLeft = true;

            shiftPoint(pointLeft1, m1, -2 * laneWidth, pointsRight.at(0));
            shiftPoint(pointLeft2, m2, -2 * laneWidth, pointsRight.at(1));
            shiftPoint(pointLeft3, m3, -2 * laneWidth, pointsRight.at(2));
        }
    } else if (polyDetectedLeft && !polyDetectedCenter) {
        usedPoly = polyLeft;
        m1 = gradient(pointsLeft.at(0).getY(), pointsLeft, usedPoly.getCoefficients());
        m2 = gradient(pointsLeft.at(1).getY(), pointsLeft, usedPoly.getCoefficients());
        m3 = gradient(pointsLeft.at(2).getY(), pointsLeft, usedPoly.getCoefficients());

        isPolyMovedCenter = true;

        shiftPoint(pointCenter1, m1, laneWidth, pointsLeft.at(0));
        shiftPoint(pointCenter2, m2, laneWidth, pointsLeft.at(1));
        shiftPoint(pointCenter3, m3, laneWidth, pointsLeft.at(2));

        if (!polyDetectedRight) {
            isPolyMovedRight = true;

            shiftPoint(pointRight1, m1, 2 * laneWidth, pointsLeft.at(0));
            shiftPoint(pointRight2, m2, 2 * laneWidth, pointsLeft.at(1));
            shiftPoint(pointRight3, m3, 2 * laneWidth, pointsLeft.at(2));
        }
    } else if (polyDetectedCenter) {
        usedPoly = polyCenter;
        m1 = gradient(pointsCenter.at(0).getY(), pointsCenter, usedPoly.getCoefficients());
        m2 = gradient(pointsCenter.at(1).getY(), pointsCenter, usedPoly.getCoefficients());
        m3 = gradient(pointsCenter.at(2).getY(), pointsCenter, usedPoly.getCoefficients());

        if (!polyDetectedLeft) {
            isPolyMovedLeft = true;

            shiftPoint(pointLeft1, m1, -laneWidth, pointsCenter.at(0));
            shiftPoint(pointLeft2, m2, -laneWidth, pointsCenter.at(1));
            shiftPoint(pointLeft3, m3, -laneWidth, pointsCenter.at(2));
        }

        if (!polyDetectedRight) {
            isPolyMovedRight = true;

            shiftPoint(pointRight1, m1, laneWidth, pointsCenter.at(0));
            shiftPoint(pointRight2, m2, laneWidth, pointsCenter.at(1));
            shiftPoint(pointRight3, m3, laneWidth, pointsCenter.at(2));
        }
    }

    // create the lane polynomial out of the shifted points

    if (isPolyMovedLeft) {
        movedPolyLeft.addData(pointLeft1);
        movedPolyLeft.addData(pointLeft2);
        movedPolyLeft.addData(pointLeft3);
    }

    if (isPolyMovedCenter) {
        movedPolyCenter.addData(pointCenter1);
        movedPolyCenter.addData(pointCenter2);
        movedPolyCenter.addData(pointCenter3);
    }

    if (isPolyMovedRight) {
        movedPolyRight.addData(pointRight1);
        movedPolyRight.addData(pointRight2);
        movedPolyRight.addData(pointRight3);

        movedPointsRight.push_back(FuPoint<int>(pointRight1.getY(), pointRight1.getX()));
        movedPointsRight.push_back(FuPoint<int>(pointRight2.getY(), pointRight2.getX()));
        movedPointsRight.push_back(FuPoint<int>(pointRight3.getY(), pointRight3.getX()));
    }
}

void cLaneDetectionFu::pubAngle() {
    if (!polyDetectedRight && !isPolyMovedRight) {
        return;
    }

    int y = projImageH - angleAdjacentLeg;
    double xRightLane;
    double m;

    if (polyDetectedRight) {
        xRightLane = polyRight.at(y);
        m = gradient(y, pointsRight, polyRight.getCoefficients());
    } else {
        xRightLane = movedPolyRight.at(y);
        m = gradient(y, movedPointsRight, movedPolyRight.getCoefficients());
    }

    double offset = -1 * laneWidth / 2;
    shiftPoint(movedPointForAngle, m, offset, (int) xRightLane, y);

    pointForAngle.setX(xRightLane);
    pointForAngle.setY(y);

    gradientForAngle = m;

    oppositeLeg = movedPointForAngle.getX() - proj_image_w_half;
    adjacentLeg = projImageH - movedPointForAngle.getY();
    double result = atan(oppositeLeg / adjacentLeg) * 180 / PI;

    /*
     * filter too rash steering angles / jitter in polynomial data
     */
    if (std::abs(result - lastAngle) > maxAngleDiff) {
        if (result - lastAngle > 0)
            result = lastAngle + maxAngleDiff;
        else
            result = lastAngle - maxAngleDiff;
    }

    lastAngle = result;

    std_msgs::Float32 angleMsg;

    angleMsg.data = result;

    publish_angle.publish(angleMsg);
}



/*
 *      Utils:
 */



void cLaneDetectionFu::shiftPoint(FuPoint<double> &p, double m, double offset, FuPoint<int> &origin) {
    shiftPoint(p, m, offset, origin.getX(), origin.getY());
}

/**
 *
 * @param p
 * @param m
 * @param offset Positive if shifting to the left, negative to the right
 * @param y
 * @param x
 */
void cLaneDetectionFu::shiftPoint(FuPoint<double> &p, double m, double offset, int x, int y) {
    /*
     * Depending on the sign of the gradient of the poly at the different
     * x-values and depending on which position we are, we have to add or
     * subtract the expected distance to the respective lane polynomial, to get
     * the wanted points.
     *
     * The calculation is done for the x- and y-components of the points
     * separately using the trigonometric ratios of right triangles and the fact
     * that arctan of some gradient equals its angle to the x-axis in degree.
     */
    if (m >= 0) {
        p.setX(x - offset * sin(atan(-1 / m)));
        p.setY(y - offset * cos(atan(-1 / m)));
        return;
    }
    p.setX(x + offset * sin(atan(-1 / m)));
    p.setY(y + offset * cos(atan(-1 / m)));
}

bool cLaneDetectionFu::isInRange(FuPoint<int> &lanePoint, FuPoint<int> &p) {
    if (p.getY() < minYDefaultRoi || p.getY() > maxYRoi) {
        return false;
    }
    if (p.getY() > lanePoint.getY()) {
        return false;
    }

    double distanceX = std::abs(p.getX() - lanePoint.getX());
    double distanceY = std::abs(p.getY() - lanePoint.getY());

    return distanceX < interestDistancePoly && distanceY < interestDistancePoly;
}


/**
 * Calculates the horizontal distance between a point and the default line given
 * by its position.
 *
 * @param line  The position of the default line (LEFT, CENTER or RIGHT)
 * @param p     The given point
 * @return      The horizontal distance between default line and point, horizontal distance = difference in X!!!
 */
int cLaneDetectionFu::horizDistanceToDefaultLine(ePosition &line, FuPoint<int> &p) {
    double pX = p.getX();
    double distance = 0;

    switch (line) {
        case LEFT:
            distance = std::abs(pX - defaultXLeft);
            break;
        case CENTER:
            distance = std::abs(pX - defaultXCenter);
            break;
        case RIGHT:
            distance = std::abs(pX - defaultXRight);
            break;
    }

    return distance;
}

/**
 * Calculates the horizontal distance between a point and a polynomial.
 *
 * @param poly  The given polynomial
 * @param p     The given point
 * @return      The horizontal distance between the polynomial and the point, horizontal distance = difference in X!!!
 */
int cLaneDetectionFu::horizDistanceToPolynomial(NewtonPolynomial &poly, FuPoint<int> &p) {
    double pY = p.getY();
    double pX = p.getX();

    double polyX = poly.at(pY);
    double distance = std::abs(pX - polyX);

    return distance;
}

/**
 * Method, that checks if a point lies within the default ROI of a position.
 *
 * @param position  The position of the default ROI
 * @param p         The given point, which is checked
 * @return          True, if the point lies within the default ROI
 */
bool cLaneDetectionFu::isInDefaultRoi(ePosition position, FuPoint<int> &p) {
    if (p.getY() < minYDefaultRoi || p.getY() > maxYRoi) {
        return false;
    } else if (horizDistanceToDefaultLine(position, p) <= interestDistanceDefault) {
        return true;
    } else {
        return false;
    }
}

/**
 * Method, that checks if a point lies within the the ROI of a polynomial.
 *
 * @param poly      The polynomial, whose ROI is used
 * @param p         The point, which is checked
 * @return          True, if the point lies within the polynomial's ROI
 */
bool cLaneDetectionFu::isInPolyRoi(NewtonPolynomial &poly, FuPoint<int> &p) {
    if (p.getY() < minYPolyRoi || p.getY() > maxYRoi) {
        return false;
    } else if (horizDistanceToPolynomial(poly, p) <= interestDistancePoly) {
        return true;
    } else {
        return false;
    }
}

/**
 * Calculates the horizontal distance between two points.
 *
 * @param p1    The first point
 * @param p2    The second point
 * @return      The horizontal distance between the two points, horizontal distance = difference in X!!!
 */
int cLaneDetectionFu::horizDistance(FuPoint<int> &p1, FuPoint<int> &p2) {
    double x1 = p1.getX();
    double x2 = p2.getX();

    return std::abs(x1 - x2);
}

/**
 * Calculates the gradient of a polynomial at a given x value. The used formula
 * was obtained by the following steps:
 * - start with the polynomial of 2nd degree in newton basis form:
 *   p(x) = c0 + c1(x - x0) + c2(x - x0)(x - x1)
 * - expand the equation and sort it by descending powers of x
 * - form the first derivative
 *
 * Applying the given x value then results in the wanted gradient.
 *
 * @param x         The given x value
 * @param points    The data points used for interpolating the polynomial
 * @param coeffs    The coefficients under usage of the newton basis
 * @return          The gradient of the polynomial at x
 */
double cLaneDetectionFu::gradient(double x, vector<FuPoint<int>> &points, vector<double> coeffs) {
    return (2 * coeffs[2] * x + coeffs[1])
           - (coeffs[2] * points[1].getY())
           - (coeffs[2] * points[0].getY());
}

/**
 * Method, that checks, if a polynomial produced during RANSAC counts as usable.
 *
 * @param position  The position of the polynomial, that is checked
 * @param poly      The polynomial, that is checked
 * @param prevPoly  The previous polynomial detected at this position
 * @return          True, if the polynomial counts as valid
 */
bool cLaneDetectionFu::polyValid(ePosition position, NewtonPolynomial poly, NewtonPolynomial prevPoly) {

    if (prevPoly.getDegree() != -1) {
        return isSimilar(poly, prevPoly);
    }
    if (position == RIGHT) {
        if (polyDetectedRight) {
            return isSimilar(poly, polyRight);
        }
        if (isPolyMovedRight) {
            return isSimilar(poly, movedPolyRight);
        }
    }
    if (position == CENTER) {
        if (polyDetectedCenter) {
            return isSimilar(poly, polyCenter);
        }
        if (isPolyMovedCenter) {
            return isSimilar(poly, movedPolyCenter);
        }
    }
    if (position == LEFT) {
        if (polyDetectedLeft) {
            return isSimilar(poly, polyLeft);
        }
        if (isPolyMovedLeft) {
            return isSimilar(poly, movedPolyLeft);
        }
    }

    return true;
}

bool cLaneDetectionFu::isSimilar(const NewtonPolynomial &poly1, const NewtonPolynomial &poly2) {
    FuPoint<int> p1 = FuPoint<int>(poly1.at(polyY1), polyY1);
    FuPoint<int> p2 = FuPoint<int>(poly2.at(polyY1), polyY1);

    if (horizDistance(p1, p2) > interestDistancePoly) { //0.05 * meters
        return false;
    }

    FuPoint<int> p3 = FuPoint<int>(poly1.at(polyY2), polyY2);
    FuPoint<int> p4 = FuPoint<int>(poly2.at(polyY2), polyY2);

    if (horizDistance(p3, p4) > interestDistancePoly) { //0.05 * meters
        return false;
    }

    FuPoint<int> p5 = FuPoint<int>(poly1.at(polyY3), polyY3);
    FuPoint<int> p6 = FuPoint<int>(poly2.at(polyY3), polyY3);

    if (horizDistance(p5, p6) > interestDistancePoly) { //0.05 * meters
        return false;
    }

    return true;
}



/*
 *      debug functions
 */



void cLaneDetectionFu::drawEdgeWindow(Mat &img, vector<vector<EdgePoint>> edges) {
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

void cLaneDetectionFu::drawLaneMarkingsWindow(Mat &img, vector<FuPoint<int>> &laneMarkings) {
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

void cLaneDetectionFu::drawGroupedLaneMarkingsWindow(Mat &img) {
    if (!PUBLISH_IMAGES && !SHOW_GROUPED_LANE_MARKINGS_WINDOW && !SAVE_FRAME_IMAGES) {
        return;
    }

    Mat transformedImagePaintable = img.clone();
    cv::cvtColor(transformedImagePaintable, transformedImagePaintable, CV_GRAY2BGR);

    debugPaintPoints(transformedImagePaintable, Scalar(0, 0, 255), laneMarkingsLeft);
    debugPaintPoints(transformedImagePaintable, Scalar(0, 255, 0), laneMarkingsCenter);
    debugPaintPoints(transformedImagePaintable, Scalar(255, 0, 0), laneMarkingsRight);
    debugPaintPoints(transformedImagePaintable, Scalar(0, 255, 255), laneMarkingsNotUsed);

    debugPaintPolynom(transformedImagePaintable, cv::Scalar(0, 255, 255), polyDetectedLeft ? polyLeft : movedPolyLeft,
                      minYPolyRoi, maxYRoi);
    debugPaintPolynom(transformedImagePaintable, cv::Scalar(255, 255, 0),
                      polyDetectedCenter ? polyCenter : movedPolyCenter, minYPolyRoi, maxYRoi);
    debugPaintPolynom(transformedImagePaintable, cv::Scalar(255, 0, 255),
                      polyDetectedRight ? polyRight : movedPolyRight, minYPolyRoi, maxYRoi);

    cv::Point2d p1l(defaultXLeft, minYPolyRoi);
    cv::Point2d p2l(defaultXLeft, maxYRoi - 1);
    cv::line(transformedImagePaintable, p1l, p2l, cv::Scalar(0, 0, 255));

    cv::Point2d p1c(defaultXCenter, minYPolyRoi);
    cv::Point2d p2c(defaultXCenter, maxYRoi - 1);
    cv::line(transformedImagePaintable, p1c, p2c, cv::Scalar(0, 255, 0));

    cv::Point2d p1r(defaultXRight, minYPolyRoi);
    cv::Point2d p2r(defaultXRight, maxYRoi - 1);
    cv::line(transformedImagePaintable, p1r, p2r, cv::Scalar(255, 0, 0));

    cv::Point2d p1(proj_image_w_half - (roiBottomW / 2), maxYRoi - 1);
    cv::Point2d p2(proj_image_w_half + (roiBottomW / 2), maxYRoi - 1);
    cv::Point2d p3(proj_image_w_half + (roiTopW / 2), minYPolyRoi);
    cv::Point2d p4(proj_image_w_half - (roiTopW / 2), minYPolyRoi);
    cv::line(transformedImagePaintable, p1, p2, cv::Scalar(0, 200, 0));
    cv::line(transformedImagePaintable, p2, p3, cv::Scalar(0, 200, 0));
    cv::line(transformedImagePaintable, p3, p4, cv::Scalar(0, 200, 0));
    cv::line(transformedImagePaintable, p4, p1, cv::Scalar(0, 200, 0));

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

void cLaneDetectionFu::drawRansacWindow(cv::Mat &img) {

    if (!PUBLISH_IMAGES && !SHOW_RANSAC_WINDOW && !SAVE_FRAME_IMAGES) {
        return;
    }

    cv::Mat transformedImagePaintableRansac = img.clone();
    cv::cvtColor(transformedImagePaintableRansac, transformedImagePaintableRansac, CV_GRAY2BGR);

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

void cLaneDetectionFu::drawAngleWindow(Mat &img) {
    frame++;

    if (!PUBLISH_IMAGES && !SHOW_ANGLE_WINDOW) {
        return;
    }

    Mat transformedImagePaintableLaneModel = img.clone();
    cvtColor(transformedImagePaintableLaneModel, transformedImagePaintableLaneModel, CV_GRAY2BGR);

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

        cv::Point pointLoc = cv::Point(proj_image_w_half, projImageH);
        cv::circle(transformedImagePaintableLaneModel, pointLoc, 2, cv::Scalar(0, 0, 255), -1);

        cv::Point anglePointLoc = cv::Point(sin(lastAngle * PI / 180) * angleAdjacentLeg + proj_image_w_half,
                                            projImageH - (cos(lastAngle * PI / 180) * angleAdjacentLeg));
        cv::line(transformedImagePaintableLaneModel, pointLoc, anglePointLoc, cv::Scalar(255, 255, 255));

        cv::Point startNormalPoint = cv::Point(pointForAngle.getX(), pointForAngle.getY());
        cv::circle(transformedImagePaintableLaneModel, startNormalPoint, 2, cv::Scalar(100, 100, 100), -1);

        cv::Point targetPoint = cv::Point(movedPointForAngle.getX(), movedPointForAngle.getY());
        cv::circle(transformedImagePaintableLaneModel, targetPoint, 2, cv::Scalar(0, 0, 255), -1);

        /*cv::Point adjacentLegPoint = cv::Point(projImageHalfW, projImageH - adjacentLeg);
        cv::line(transformedImagePaintableLaneModel, pointLoc, adjacentLegPoint, cv::Scalar(255,0,0));

        cv::Point oppositeLegPoint = cv::Point(projImageHalfW + oppositeLeg, projImageH - adjacentLeg);
        cv::line(transformedImagePaintableLaneModel, adjacentLegPoint, oppositeLegPoint, cv::Scalar(0,255,0));*/

        double m = -gradientForAngle;

        double n = pointForAngle.getY() - m * pointForAngle.getX();
        double x = 10;
        double y = m * x + n;

        cv::Point endNormalPoint = cv::Point(x, y);
        cv::line(transformedImagePaintableLaneModel, startNormalPoint, endNormalPoint, cv::Scalar(0, 0, 0));

    } else {
        cv::Point pointLoc = cv::Point(5, 5);
        cv::circle(transformedImagePaintableLaneModel, pointLoc, 3, cv::Scalar(0, 0, 255), 0);
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

void cLaneDetectionFu::pubRGBImageMsg(cv::Mat &rgb_mat, image_transport::CameraPublisher publisher) {
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

void cLaneDetectionFu::debugPaintPolynom(cv::Mat &m, cv::Scalar color, NewtonPolynomial &p, int start, int end) {
    cv::Point point;
    for (int i = start; i < end; i++) {
        point = cv::Point(p.at(i), i);
        cv::circle(m, point, 0, color, -1);
    }
}

void cLaneDetectionFu::debugPaintPoints(cv::Mat &m, cv::Scalar color, vector<FuPoint<int>> &points) {
    for (FuPoint<int> point : points) {
        cv::Point pointLoc = cv::Point(point.getX(), point.getY());
        cv::circle(m, pointLoc, 1, color, -1);
    }
}

void cLaneDetectionFu::debugWriteImg(cv::Mat &image, string folder) {
    stringstream img;
    img << "./" << folder << "/" << frame << ".jpg";
    cv::imwrite(img.str(), image);
}



/*
 *      ROS:
 */



void cLaneDetectionFu::config_callback(line_detection_fu::LaneDetectionConfig &config, uint32_t level) {
    ROS_ERROR("Reconfigure Request");

    interestDistancePoly = config.interestDistancePoly;
    interestDistanceDefault = config.interestDistanceDefault;
    iterationsRansac = config.iterationsRansac;
    maxYRoi = config.maxYRoi;
    minYDefaultRoi = config.minYDefaultRoi;
    minYPolyRoi = config.minYPolyRoi;
    polyY1 = config.polyY1;
    polyY2 = config.polyY2;
    polyY3 = config.polyY3;
    maxAngleDiff = config.maxAngleDiff;
    projYStart = config.projYStart;
    roiTopW = config.roiTopW;
    roiBottomW = config.roiBottomW;
    proportionThreshould = config.proportionThreshould;
    gradientThreshold = config.gradientThreshold;
    nonMaxWidth = config.nonMaxWidth;
    laneMarkingSquaredThreshold = config.laneMarkingSquaredThreshold;
    angleAdjacentLeg = config.angleAdjacentLeg;
    scanlinesVerticalDistance = config.scanlinesVerticalDistance;
    scanlinesMaxCount = config.scanlinesMaxCount;

    scanlines = getScanlines();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "cLaneDetectionFu");
    ros::NodeHandle nh;

    cLaneDetectionFu node = cLaneDetectionFu(nh);

    dynamic_reconfigure::Server<line_detection_fu::LaneDetectionConfig> server;
    dynamic_reconfigure::Server<line_detection_fu::LaneDetectionConfig>::CallbackType f;
    f = boost::bind(&cLaneDetectionFu::config_callback, &node, _1, _2);
    server.setCallback(f);

    while (ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}



/*
 *      Unused (but may be useful in future)
 */



/**
 * Calculates the x value of the point where the normal of the tangent of a
 * polynomial at a given point p intersects with a second polynomial.
 *
 * The formula for the intersection point is obtained by setting equal the
 * following two formula:
 *
 * 1. the formula of the normal in point-slope-form:
 *     y - p_y = -(1 / m) * (x - p_x) which is the same as
 *           y = -(x / m) + (p_x / m) + p_y
 *
 * 2. the formula of the second polynomial of 2nd degree in newton basis form:
 *           y = c0 + c1(x - x0) + c2(x - x0)(x - x1)
 *
 * Expanding everything and moving it to the right side gives a quadratic
 * equation in the general form of 0 = ax^2 + bx + c, which can be solved using
 * the general quadratic formula x = (-b +- sqrt(b^2 - 4ac)) / 2a
 *
 * The three cases for the discriminant are taken into account.
 *
 * @param p         The point of the first poly at which its tangent is used
 * @param m         The gradient of the tangent
 * @param points    The data points used for interpolating the second polynomial
 * @param coeffs    The coeffs of the second polynomial with newton basis
 * @return          The x value of the intersection point of normal and 2nd poly
 */
double cLaneDetectionFu::intersection(FuPoint<double> &p, double &m,
                                      std::vector<FuPoint<int>> &points, std::vector<double> &coeffs) {
    double a = coeffs[2];
    double b = coeffs[1] - (coeffs[2] * points[1].getY())
               - (coeffs[2] * points[0].getY()) + (1.0 / m);
    double c = coeffs[0] - (coeffs[1] * points[0].getY())
               + (coeffs[2] * points[0].getY() * points[1].getY())
               - p.getY() - (p.getX() / m);

    double dis = std::pow(b, 2) - (4 * a * c);
    double x1 = 0;
    double x2 = 0;

    if (dis < 0) {
        return -1;
    } else if (dis == 0) {
        return -b / (2 * a);
    } else {
        x1 = (-b + std::sqrt(std::pow(b, 2) - (4 * a * c))) / (2 * a);
        x2 = (-b - std::sqrt(std::pow(b, 2) - (4 * a * c))) / (2 * a);
    }

    return fmax(x1, x2);
}

/**
 * Calculates the gradient of a second polynomial at the point, at which the
 * normal of the tangent of the first polynomial at the given point
 * intersects with the second polynomial.
 *
 * @param x         The given x value of the point on the first polynomial
 * @param poly1     The first polynomial
 * @param points1   The data points used for interpolating the first poly
 * @param points2   The data points used for interpolating the second poly
 * @param coeffs1   The coeffs of the first poly using newton basis
 * @param coeffs2   The coeffs of the second poly using newton basis
 * @param m1        The gradient of the first poly at x
 * @return          The gradient of the second poly at the intersection point
 */
double cLaneDetectionFu::nextGradient(double x, NewtonPolynomial &poly1,
                                      std::vector<FuPoint<int>> &points1, std::vector<FuPoint<int>> &points2,
                                      std::vector<double> coeffs1, std::vector<double> coeffs2, double m1) {

    FuPoint<double> p = FuPoint<double>(x, poly1.at(x));
    double x2 = intersection(p, m1, points2, coeffs2);

    return gradient(x2, points2, coeffs2);
}

/**
 * Check two gradients for similarity. Return true if the difference in degree
 * is less than 10.
 *
 * @param m1    The first gradient
 * @param m2    The second gradient
 * @return      True, if the diffenence between the gradients is less than 10°
 */
bool cLaneDetectionFu::gradientsSimilar(double &m1, double &m2) {
    double a1 = atan(m1) * 180 / PI;
    double a2 = atan(m2) * 180 / PI;

    if (abs(a1 - a2) < 10) {
        return true;
    } else {
        return false;
    }
}

/**
 * Finds the position of the polynomial with the highest proportion.
 * @return The position of the best polynomial
 */
ePosition cLaneDetectionFu::maxProportion() {
    ePosition maxPos = LEFT;
    double maxVal = bestPolyLeft.second;

    if (bestPolyCenter.second > maxVal) {
        maxPos = CENTER;
        maxVal = bestPolyCenter.second;
    }

    if (bestPolyRight.second > maxVal) {
        maxPos = RIGHT;
    }

    return maxPos;
}