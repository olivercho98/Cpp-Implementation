#include <iostream>
using namespace std;
#include <sstream>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <cstring>
#include <string>
#include <opencv2/highgui/highgui_c.h>
#include <time.h>

#include <opencv2/opencv.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include "rift_no_rotation_invariance.h"
using namespace cv;

#include <stdio.h>

float RATIO = 1;

int main()
{
    Mat im2 = imread("/home/pcy/RIFT/CropImage1.jpg");
    Mat im1 = imread("/home/pcy/RIFT/IRImage3.jpg");

    //=======================================
    resize(im2, im2, im1.size(), INTER_CUBIC);
    // size(cols, rows), cols is the first parameter and rows is the second.
    //=======================================

    //RIFT feature detection and description
    rift_no_rotation_invariance IM1, IM2;
    Mat des_m1, des_m2;
    des_m1 = IM1.RIFT_no_rotation_invariance(im1,4,6,96);
    des_m2 = IM2.RIFT_no_rotation_invariance(im2,4,6,96);

    //Match
    BFMatcher matcher;
    std::vector<std::vector<cv::DMatch> > matches;
    matcher.knnMatch(des_m1, des_m2, matches, 2);
    
    std::list<int, std::allocator<int> > temple;
    std::list<int> temple_index_repeat;
    std::list<int> temple_index_unique;

    for (int i = 0; i < matches.size(); i++)
    {
        if (std::find(temple.begin(), temple.end(), matches[i][0].trainIdx) != temple.end() )
        // trainIdx is the idx of the corresponding counterpart about current keypoint.
        // The code above means finding the corresponding one in the temple
        {
            temple_index_repeat.push_back(i);
        }
        else
        {
            temple_index_unique.push_back(i);
            temple.push_back(matches[i][0].trainIdx);
        }
    }

    std::vector<std::vector<cv::DMatch> > new_matches;

    for (int i: temple_index_unique)
    {
        new_matches.push_back(matches[i]); //matches
    }
    matches = new_matches;
    
    std::vector<cv::DMatch> match;
    for (int i = 0; i < matches.size(); i++)
    {
        if (matches[i][0].distance < RATIO * matches[i][1].distance)
        {
            match.push_back(matches[i][0]);
        }
    }

    // Applying ransac
    std::vector<cv::DMatch> InlierMatches;
    std::vector<cv::Point2f> p1, p2;
    for (int i = 0; i < match.size(); i++)
    {
        p1.push_back(IM1.kps[match[i].queryIdx].pt); // pt position
        p2.push_back(IM2.kps[match[i].trainIdx].pt);
    }
    
    // RANSAC FindFundamental Matrix
    std::vector<uchar> RANSACStatus;
    cv::findFundamentalMat(p1, p2, RANSACStatus, CV_FM_RANSAC); // p1 and p2 denote the point locations
    printf("findFundamentalMat_finished. Line:%d in main.\n", __LINE__);
    for (int i = 0; i < match.size(); i++)
    {
        if (RANSACStatus[i] != 0)
        {
            InlierMatches.push_back(match[i]); // Preserve inlier matches
        }
    }
    
    cv::Mat Img_matches;
    imshow("RANSAC_match", Img_matches);
    cv::waitKey(0);

    return EXIT_SUCCESS;
}
