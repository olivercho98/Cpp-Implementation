#ifndef RIFT_DESCRIPTOR_NO_ROTATION_INVARIANCE_H
#define RIFT_DESCRIPTOR_NO_ROTATION_INVARIANCE_H

#endif // RIFT_DESCRIPTOR_NO_ROTATION_INVARIANCE_H

#include <iostream>
using namespace std;
#include "opencv2/opencv.hpp"
using namespace cv;

#include <math.h>
#include <algorithm>

class rift_descriptor_no_rotation_invariance
{
public:
    rift_descriptor_no_rotation_invariance();
    Mat RIFT_descriptor_no_rotation_invariance(Mat im, std::vector<KeyPoint> keypoints, std::vector<Mat> eo,
                                               int patch_size, int s, int o);
    std::vector<KeyPoint> kps;

private:
    int histSize_RI = 6; //set the number of bins in histogram
    int xim, yim;
    Mat CS;
    Mat des;
    //Mat RIFT_des;
    //Mat MIM, MIM_Xsec;
    //std::vector<int> MIM_x;
    //float hrange[2] = {0, 255};
    //const float* histRange[1] = { hrange };

    // potential problems

};
