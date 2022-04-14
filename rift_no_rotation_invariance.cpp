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
/*
    //useful paras
    Mat M1, M2;
    Mat eo1, eo2;

    //useless paras
    Mat m1, m2;
    Mat PC1, PC2;
    Mat pcSum1, pcSum2;
    double T1, T2;
    double ori1, ori2;
    double ft1, ft2;
*/

    //M1, m1, ori1, ft1, PC1, eo1, pcSum1 = phasecong3(im1,s,o,3,'mult',1.6,'sigmaOnf',0.75,'g', 3, 'k',1);
    //M2, m2, ori2, ft2, PC2, eo2, pcSum2 = phasecong3(im2,s,o,3,'mult',1.6,'sigmaOnf',0.75,'g', 3, 'k',1);

    printf("success_regis_begin\n");
    // This part may have potential for further improvement.
    auto im1_size = im1.size();
    //auto im2_size = im2.size();
    PhaseCongruency pc1(im1_size, s, o);
    //PhaseCongruency pc2(im1_size, s, o);
    //Mat im1_edges, im1_corners, im1_gray;
    //Mat im2_edges, im2_corners;
    cvtColor(im1,im1_gray,CV_BGR2GRAY);
    pc1.feature(im1_gray, im1_edges, im1_corners);
    //pc2.feature(im2, im2_edges, im2_corners);

/*
    for (int j=0; j<M1.rows; j++)
    {
        for (int i=0; i<M1.cols; i++)
        {

        }
    }
*/
/*
    int a = *max_element(M1,M1+sizeof(M1)); // there a problem may lay
    int b = *min_element(M1,M1+sizeof(M1));
    int m_1 = (m1-b)/(a-b);
    int a = *max_element(M2,M2+sizeof(M2)); // there a problem may lay
    int b = *min_element(M2,M2+sizeof(M2));
    int m_2 = (m1-b)/(a-b);
*/
    printf("success_pc_over.\n");
    printf("success_pc_over_again.\n");
    M1 = Mat::zeros(pc1.maxMoment.size(),CV_8U);
    printf("success_M1_create.\n");
    printf("depth of pc.M1:%d, Line:%d.\n", pc1.maxMoment.depth(), __LINE__);
    printf("rows of pc.M1:%d, Line:%d.\n", pc1.maxMoment.rows, __LINE__);
    printf("cols of pc.M1:%d, Line:%d.\n", pc1.maxMoment.cols, __LINE__);
    M1 = pc1.maxMoment.clone();

    //imshow("M1", M1);
    //cv::waitKey(0);

    printf("success_maxMoment.\n");
    printf("depth of M1:%d, Line:%d.\n", M1.depth(), __LINE__);
    //Mat M2 = pc2.maxMoment;
    double minv1 = 0, maxv1 = 0;
    printf("success_minmaxValue.\n");
    double* minp1 = &minv1;
    double* maxp1 = &maxv1;
    printf("success_pointer.\n");
    minMaxIdx(M1,minp1,maxp1);

    printf("success_minmax.\n");
    // Potential question: int to double

    for (int j=0; j<M1.rows; j++)
    {
        for (int i=0; i<M1.cols; i++)
        {
            M1.at<double>(j,i) = (M1.at<double>(j,i)-minv1)/(maxv1-minv1);
            //printf("%f\n", M1.at<double>(j,i));
            //========This part were M1 normalization after GRAY2BGR==============
            //----The order was changed: calculation first and convert later------

            //M1.at<Vec3b>(j,i)[0] = (M1.at<Vec3b>(j,i)[0]-minv1)/(maxv1-minv1);
            //M1.at<Vec3b>(j,i)[1] = (M1.at<Vec3b>(j,i)[1]-minv1)/(maxv1-minv1);
            //M1.at<Vec3b>(j,i)[2] = (M1.at<Vec3b>(j,i)[2]-minv1)/(maxv1-minv1);
            //====================================================================
        }
    }
    printf("success_normalization.\n");
    printf("depth of M1:%d, Line:%d.\n", M1.depth(), __LINE__);
    M1.convertTo(M1_F, CV_8U, 255, 0);
    // The python version it was converted to uint8, so CV_8U. FAST only accepts CV_8U or 32F.
    // int alpha, int beta, for every element x it means ax+b
    // In the python version M1 was multiplied with 255, which means recover from the state of normalization.

    cvtColor(M1_F,M1_BGR,CV_GRAY2BGR);
    printf("depth of M1_BGR:%d, Line:%d.\n", M1_BGR.depth(), __LINE__);
    /*
    int minv2 = 0, maxv2 = 0;
    int* minp2 = &minv2;
    int* maxp2 = &maxv2;
    minMaxIdx(M2,minp2,maxp2);
    for (int j=0; j<M2.rows; j++)
    {
        for (int i=0; i<M2.cols; i++)
        {
            M2.at(j,i) = (M2.at(j,i)-minv2)/(maxv2-minv2);
        }
    }
    */

    printf("success_M1_convert.\n");
    //FAST detector on the maximum moment maps to extract edge feature points.

    //Mat dst1 = M1.clone();
    //Mat dst2 = M2.clone();

    //==================2 alternative: SIFT and FAST========================
    /*
    cv::Ptr<cv::xfeatures2d::SIFT> detector1 =
            cv::xfeatures2d::SIFT::create(thre);
    detector1->detectAndCompute(im1, cv::Mat(), keypoints1, descriptor_1);
    */
    //======================================================================
    Ptr<FastFeatureDetector> detector1 = FastFeatureDetector::create(thre);
    //Ptr<FastFeatureDetector> detector2 = FastFeatureDetector::create(thre);

    detector1->detect(M1_BGR,keypoints1);
    //detector2->detect(im2,keypoints2);
    //======================================================================
    printf("success_FAST.\n");
    printf("number of original keypoints:%ld, Line:%d.\n", keypoints1.size(), __LINE__);
    //printf("x of original keypoints[0]:%f, Line:%d.\n", keypoints1[0].pt.x, __LINE__);
    //printf("y of original keypoints[0]:%f, Line:%d.\n", keypoints1[0].pt.y, __LINE__);

    if(keypoints1.size()>1001)
    // I set 501 here in order to avoid potential exception: keypoints1.begin()+500 = keypoints1.end().
    {
        std::sort(keypoints1.begin(), keypoints1.end(), sortFunction);
        keypoints1.erase(keypoints1.begin()+1000, keypoints1.end());
    }

    //RIFT
    rift_descriptor_no_rotation_invariance RI;
    printf("success_RIFT_CREATE.\n");
    // Update: vectors kps as outputs are moved.
    //Mat des_m1;
    des_m1 = RI.RIFT_descriptor_no_rotation_invariance(im1, keypoints1, pc1.eo, patch_size, s, o);
    kps.assign(RI.kps.begin(), RI.kps.end());
    //int size_filtered_kps = kps.size();
    for (int i=0; i<24; i++)
    {
        eo_single = pc1.eo[i].clone();
        eo.push_back(eo_single);
    }

    //print("number of filtered keypoints in rift: %ld\n", size_filtered_kps);
    //std::vector kps1 = rift_descriptor_no_rotation_invariance.kps;
    /*
    Mat des_m2 = rift_descriptor_no_rotation_invariance::
            rift_descriptor_no_rotation_invariance(im2, keypoints2, pc2.eo, patch_size, s, o);
    */
    return des_m1;
}
