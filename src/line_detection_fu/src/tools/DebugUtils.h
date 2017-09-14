#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/video/background_segm.hpp>

#include "Edges.h"

#include <stdlib.h>

using namespace std;

class DebugUtils {
public:
    DebugUtils();

    void paintOutputRoi(cv::Mat img, vector<vector<EdgePoint>> edges);
};