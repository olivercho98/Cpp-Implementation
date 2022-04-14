#ifndef RIFT_NO_ROTATION_INVARIANCE_H
#define RIFT_NO_ROTATION_INVARIANCE_H

#endif // RIFT_NO_ROTATION_INVARIANCE_H

#include <iostream>
using namespace std;
#include <algorithm>
//<algorithm> is used for maxmium counting

#include <cstdlib>
#include <cmath>
#include <iomanip>

#include "opencv2/opencv.hpp"
#include "phase.h"
#include "rift_descriptor_no_rotation_invariance.h"
using namespace cv;

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/contrib/contrib.hpp"
//these hpp files are included for the counting of maxmium and minmium

#include<opencv2/xfeatures2d.hpp>
using namespace cv::xfeatures2d;

/*
struct keyPoint
{
    Mat des;
    Mat kps;
};
*/

class rift_no_rotation_invariance
{
public:
    rift_no_rotation_invariance(void);
    ~rift_no_rotation_invariance(void);
    Mat RIFT_no_rotation_invariance(Mat im1, int s, int o, int patch_size);
    std::vector<KeyPoint> keypoints1;
    std::vector<KeyPoint> keypoints_robust;
    std::vector<KeyPoint> kps;
    cv::Mat descriptor_1;
    Mat M1;
    Mat eo_single;
    std::vector<cv::Mat> eo;
private:
    //int thre = 40;
    int thre = 5;
    Mat im1_edges, im1_corners, im1_gray, M1_BGR, M1_F;
    //Mat M1;
    Mat des_m1;
    //std::vector<KeyPoint> keypoints1;
    //std::vector<KeyPoint> keypoints2;
};
