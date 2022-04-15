#include "rift_no_rotation_invariance.h"


rift_no_rotation_invariance::rift_no_rotation_invariance(void)
{

}

rift_no_rotation_invariance::~rift_no_rotation_invariance(void)
{

}

bool sortFunction(KeyPoint fir, KeyPoint sec)
{
    return (fir.response > sec.response);  //from high value to low value
}

Mat rift_no_rotation_invariance::RIFT_no_rotation_invariance(Mat im1, int s, int o, int patch_size)
{
    auto im1_size = im1.size();

    PhaseCongruency pc1(im1_size, s, o);

    cvtColor(im1,im1_gray,CV_BGR2GRAY);
    pc1.feature(im1_gray, im1_edges, im1_corners);

    M1 = Mat::zeros(pc1.maxMoment.size(),CV_8U);

    M1 = pc1.maxMoment.clone();

    double minv1 = 0, maxv1 = 0;

    double* minp1 = &minv1;
    double* maxp1 = &maxv1;

    minMaxIdx(M1,minp1,maxp1);

    for (int j=0; j<M1.rows; j++)
    {
        for (int i=0; i<M1.cols; i++)
        {
            M1.at<double>(j,i) = (M1.at<double>(j,i)-minv1)/(maxv1-minv1);
        }
    }

    M1.convertTo(M1_F, CV_8U, 255, 0);
    // The original version it was converted to uint8, so CV_8U. FAST only accepts CV_8U or 32F.
    // int alpha, int beta, for every element x it means ax+b
    // In the original version M1 was multiplied with 255, which means recover from the state of normalization.

    cvtColor(M1_F,M1_BGR,CV_GRAY2BGR);

    Ptr<FastFeatureDetector> detector1 = FastFeatureDetector::create(thre);

    detector1->detect(M1_BGR,keypoints1);

    if(keypoints1.size()>1001)
    {
        std::sort(keypoints1.begin(), keypoints1.end(), sortFunction);
        keypoints1.erase(keypoints1.begin()+1000, keypoints1.end());
    }

    // RIFT
    rift_descriptor_no_rotation_invariance RI;
    des_m1 = RI.RIFT_descriptor_no_rotation_invariance(im1, keypoints1, pc1.eo, patch_size, s, o);
    kps.assign(RI.kps.begin(), RI.kps.end());

    for (int i=0; i<24; i++)
    {
        eo_single = pc1.eo[i].clone();
        eo.push_back(eo_single);
    }

    return des_m1;
}
