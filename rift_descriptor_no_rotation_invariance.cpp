#include "rift_descriptor_no_rotation_invariance.h"

#if 1
#define DEBUG_PRINT()   printf("%d\n", __LINE__)
#else
#define DEBUG_PRINT()
#endif

rift_descriptor_no_rotation_invariance::rift_descriptor_no_rotation_invariance()
{

}

Mat rift_descriptor_no_rotation_invariance::
RIFT_descriptor_no_rotation_invariance(Mat im, std::vector<KeyPoint> keypoints, std::vector<Mat> eo, int patch_size, int s, int o)
{
    //int *temp = NULL;
    printf("success_RIFT_Entered.\n");
    printf("size of eo:%ld, Line:%d.\n", eo.size(), __LINE__);
    //printf("size of eo[0]:%ld, Line:%d.\n", eo[1].size(), __LINE__);
    printf("rows of eo[0][0]:%d, Line:%d.\n", eo[23].rows, __LINE__);
    printf("cols of eo[0][0]:%d, Line:%d.\n", eo[23].cols, __LINE__);
    yim = im.rows;
    xim = im.cols;
    printf("success_xy_valued.\n");
    //histSize = o+2;
    //histSize = 8;
    //int histSize_RI = 8;

    printf("success_histSize_defined.\n");

    // CS is a 3D matrix
    int size_cs[] = {yim, xim, o};
    printf("yim:%d\n",yim);
    printf("xim:%d\n",xim);
    printf("o:%d\n",o);

    int res_cs = yim*xim*o;
    double *d_cs = new double[res_cs];

    for(int i=0; i<res_cs; i++)
    {
        d_cs[i] = 0;
    }
    CS = Mat(3, size_cs, CV_64F, d_cs);
    /*
    Mat CS_single = Mat(3, size_cs, CV_64F, d_cs);
    vector<Mat> CS_channels;
    for (int i=0;i<3;i++)
    {
        CS_channels.push_back(CS_single);
    }
    merge(CS_channels,CS);
    */

    //CS = Mat(xim,yim,o,CV_8UC3);
    // Maybe the parameter CVU3 should be added.
    //Mat CS(cv::Size(xim,yim,o));
    // size(width, height), first cols and second rows, take care
    // 3D matrix for convolution sequence

    printf("success_CS_created.\n");

    /*
    for(int j=0; j<xim; j++)
    {
        for(int i=0; i<yim; i++)
        {
            for(int k=0; k<o; k++)
            {
                CS.at<Vec3b>(j,i,k)[0] = 0;
                CS.at<Vec3b>(j,i,k)[1] = 0;
                CS.at<Vec3b>(j,i,k)[2] = 0;
            }

        }
    }
    */

    //row == heigh == Point.y
    //col == width == Point.x
    //Mat::at(Point(x, y)) == Mat::at(y,x)


    for(int j=0; j<o; j++)
    {
        //printf("entered_o %d times.\n", j);
        for(int i=0; i<s; i++)
        {
            //printf("entered_s %d times.\n", i);
            for(int m=0; m<yim; m++)
            {
                //printf("entered_yim %d times.\n", m);
                for(int n=0; n<xim; n++)
                {
                    //CS.at(m,n,j) = CS.at(m,n,j)+abs(eo.at(m,n,i,j));
                    CS.at<double>(m,n,j) = CS.at<double>(m,n,j)+sqrt(static_cast<double>(eo[j*s+i].at<Vec2d>(m,n)[0] * eo[j*s+i].at<Vec2d>(m,n)[0]
                            + eo[j*s+i].at<Vec2d>(m,n)[1] * eo[j*s+i].at<Vec2d>(m,n)[1]));
                    //CS.at<double>(m,n,j) = CS.at<double>(m,n,j)+abs(eo[j*s+i].at<double>(m,n));
                    //printf("eo[j*s+i].at<double>(m,n):%f\n", eo[j*s+i].at<double>(m,n));
                    //CS.at<Vec3f>(m,n,j)[1] = CS.at<Vec3f>(m,n,j)[1]+(float)fabs(eo[j][i].at<double>(m,n));
                    //CS.at<Vec3f>(m,n,j)[2] = CS.at<Vec3f>(m,n,j)[2]+(float)fabs(eo[j][i].at<double>(m,n));
                    // further improvement: maybe changed into matrix add

                    // potential problem!!!
                }
            }
        }
    }
    printf("type of eo[0]:%d, Line:%d.\n", eo[0].type(), __LINE__);
    printf("type of eo[4]:%d, Line:%d.\n", eo[0].type(), __LINE__);
    std::complex<double> z1 {eo[0*s+0].at<Vec2d>(0,0)[0], eo[0*s+0].at<Vec2d>(0,0)[1]};
    double z1_manual = sqrt(static_cast<double>(eo[0].at<Vec2d>(0,0)[0] * eo[0].at<Vec2d>(0,0)[0]
                    + eo[0].at<Vec2d>(0,0)[1] * eo[0].at<Vec2d>(0,0)[1]));
    printf("manual abs of eo[0*s+0].at<Vec2f>(0,0):%lf\n", z1_manual);
    printf("abs of eo[0*s+0].at<Vec2f>(0,0):%lf\n", abs(z1));
    printf("abs of eo[0*s+0].at<double>(0,0):%lf\n", abs(eo[0*s+0].at<double>(0,0)));
    //printf("fabs of eo[0*s+0].at<Vec2f>(0,0):%lf\n", fabs(z1));
    //printf("fabs of eo[0*s+0].at<double>(0,0):%lf\n", fabs(eo[0*s+0].at<double>(0,0)));
    printf("eo[0*s+0].at<Vec2f>(0,0)[0]:%lf\n", eo[0*s+0].at<Vec2d>(0,0)[0]);
    printf("eo[0*s+0].at<Vec2f>(0,0)[1]:%lf\n", eo[0*s+0].at<Vec2d>(0,0)[1]);
    printf("eo[0*s+0].at<double>(25,25):%lf\n", eo[0*s+0].at<double>(25,25));
    printf("eo[0*s+0].at<Vec2f>(25,25)[0]:%lf\n", eo[0*s+0].at<Vec2d>(25,25)[0]);
    printf("eo[0*s+0].at<Vec2f>(25,25)[1]:%lf\n", eo[0*s+0].at<Vec2d>(25,25)[1]);
    double z2_manual = sqrt(static_cast<double>(eo[0].at<Vec2d>(25,25)[0] * eo[0].at<Vec2d>(25,25)[0]
                    + eo[0].at<Vec2d>(25,25)[1] * eo[0].at<Vec2d>(25,25)[1]));
    printf("manual abs of eo[0*s+0].at<Vec2f>(25,25):%lf\n", z2_manual);
    std::complex<double> z2 {eo[0*s+0].at<Vec2d>(25,25)[0], eo[0*s+0].at<Vec2d>(25,25)[1]};
    printf("abs of eo[0*s+0].at<Vec2f>(25,25):%lf\n", abs(z2));
    printf("abs of eo[0*s+0].at<double>(0,0):%lf\n", abs(eo[0*s+0].at<double>(0,0)));
    printf("eo[1*s+0].at<double>(0,0):%lf\n", eo[1*s+0].at<double>(0,0));
    printf("eo[1*s+0].at<double>(25,25):%lf\n", eo[1*s+0].at<double>(25,25));
    printf("eo[2*s+0].at<double>(0,0):%lf\n", eo[2*s+0].at<double>(0,0));
    printf("eo[2*s+0].at<double>(25,25):%lf\n", eo[2*s+0].at<double>(25,25));
    printf("eo[3*s+0].at<double>(0,0):%lf\n", eo[3*s+0].at<double>(0,0));
    printf("eo[3*s+0].at<double>(25,25):%lf\n", eo[3*s+0].at<double>(25,25));
    //+++++++++++++++++++++
    /*
    for(int j=0; j<o; j++)
    {
        for(int i=0; i<s; i++)
        {
            printf("%f\n", eo[j][i].at<double>(0,0));
        }
    }
    */
    //+++++++++++++++++++++

    printf("successful_CS_Valued.%s:%d\n",__FILE__, __LINE__);

    //--------------------TEST------------------------
    /*
    for (int i = 0; i<yim; i++)
    {
        for (int j = 0; j<xim; j++)
        {
            for (int k = 0; k<o; k++)
            {
                printf("%f\n", CS.at<double>(i,j,k));
            }
        }
    }
    */
    // CS is incorrect.
    //------------------------------------------------

    Mat MIM = Mat(yim, xim, CV_8U);
    // MIM is used for the storage of index of min/max
    printf("successful_MIM_created:%ld\n", MIM.total());
    // tri-channel, so should MIM be a 3D matrix?



    Mat MIM_Xsec = Mat(o, 1, CV_64F);
    std::vector<double> MIM_o;
    // For each member in MIM: the corresponding x axis in CS is fixed and change y,z to compare

    for (int i = 0; i<yim; i++)
    {
        for (int j = 0; j<xim; j++)
        {
            for (int k = 0; k<o; k++)
            {

                MIM_Xsec.at<double>(k,0) = CS.at<double>(i,j,k);
                //printf("%f\n",MIM_Xsec.at<double>(k,0));
                //MIM_Xsec.at<Vec3f>(k,0)[1] = CS.at<Vec3f>(i,j,k)[1];
                //MIM_Xsec.at<Vec3f>(k,0)[2] = CS.at<Vec3f>(i,j,k)[2];
            }
            //MIM_Xsec stores the max orient for a fixed pair of x and y.

            //cvtColor(MIM_Xsec,MIM_Xsec,CV_RGB2GRAY);
            // Here 'cause I changed CS from a 3-channel to a single channel
            // The former cvtColor was deleted.

            /*
            double minv = 0.0, maxv = 0.0;
            double* minp = &minv;
            double* maxp = &maxv;
            minMaxIdx(MIM_Xsec,minp,maxp);
            MIM_Xsec.reshape(0, 1);
            */

            MIM_o.clear();
            int maxIdx = 0;
            double maxValue = 0;
            for (int x=0; x<o; x++)
            {
                if(MIM_Xsec.at<double>(x,0) > maxValue)
                {
                    maxValue = MIM_Xsec.at<double>(x,0);
                    maxIdx = x;
                }
            }
            //==============Abandoned block=========================
            /*
            for (int x=0; x<o; x++)
            {
                MIM_o.push_back(MIM_Xsec.at<double>(x,0));
            }
            int maxIdx = std::distance(MIM_o.begin(), std::max_element(MIM_o.begin(), MIM_o.end()));
            */
            // distance(): if [0]~[9] the output is 10
            // There a potential error may be laid.
            //======================================================

            MIM.at<uchar>(i,j) = maxIdx+1;
            //printf("MIM.at<uchar>(i,j):%d\n", MIM.at<uchar>(i,j));
            //MIM.at<Vec3b>(i,j)[1] = maxIdx-1;
            //MIM.at<Vec3b>(i,j)[2] = maxIdx-1;
        }
        //MIM.at(j,k) = minMaxIdx(MIM_Xsec);

        // maybe changed
    }
    printf("successful_MIM_Xsec_Valued.\n");
    //delete [] d_cs;

    //MIM_Xsec.reshape(1, 0); // the two parameters are channels and rows

    //--------------------TEST------------------------
    /*
    for (int i = 0; i<yim; i++)
    {
        for (int j = 0; j<xim; j++)
        {
            printf("%d\n", MIM.at<uchar>(i,j));
        }
    }
    */
    // MIM is erroneous.
    //------------------------------------------------

    // Here 'cause I changed MIM_Xsec from a 3-channel to a single channel
    // The former code was moved into the 'for' cycle.


    printf("successful_MIM_x_Calculated.\n");

    /*
    for (int j = 0; j<xim; j++)
    {
        for (int k = 0; k<yim; k++)
        {
            MIM.at<Vec3b>(j,k)[0] = maxIdx-1;
            MIM.at<Vec3b>(j,k)[1] = maxIdx-1;
            MIM.at<Vec3b>(j,k)[2] = maxIdx-1;
        }
    }
    */
    printf("successful_MIM_valued.\n");

    //vector<int>::size_type sizes = keypoints.size();
    //int size_ss = sizes;
    int size_ss = keypoints.size();
    printf("size_ss:%d\n", size_ss);
    printf("keypoints[0].size:%f\n", keypoints[0].size);

    int size_des[] = {36*o, size_ss};
    printf("successful_size_des_created.\n");


    int res_des = 36*o*size_ss;
    printf("successful_res_des_created.\n");
    //uint16_t *d_des = new uint16_t[res_des];
    float *d_des = new float[res_des];
    printf("successful_d_des_created.\n");
    for(int i=0; i<res_des; i++)
    {
        d_des[i] = 0;
    }
    printf("successful_d_des_valued.\n");
    des = Mat(2, size_des, CV_32F, d_des);
    //des = Mat::zeros(36*o, size_ss, CV_16U);
    // 2nd dim of kps
    // Here I remove the C3 and change it to single channel
    printf("successful_des_created.\n");

    // For 2D vectors, .size() means length of rows, [i].size() means length of cols
    Mat kps_to_ignore = Mat::zeros(1,(int)size_ss, CV_16U);
    printf("successful_des&kpsIgnore_created.\n");

    //+++++++++This part is quite essential+++++++++++++
    float hrange[] = {1, 6};
    const float*histRange = { hrange };
    //++++++++++++++++++++++++++++++++++++++++++++++++++

    int ns = 6;
    int size_R_des[] = {ns, ns, o};

    int res_R_des = ns*ns*o;
    //printf("successful_res_R_des_created.\n");
    //uint16_t *d_R_des = new uint16_t[res_R_des];
    float *d_R_des = new float[res_R_des];
    //printf("successful_d_R_des_created.\n");
    for(int i=0; i<res_R_des; i++)
    {
        d_R_des[i] = 0;
    }
    printf("successful_d_des_valued.\n");
    Mat RIFT_des = Mat(3, size_R_des, CV_32F, d_R_des);
    //Mat RIFT_des = Mat(ns, ns, o, CV_16UC3);

    for (int k=0; k<size_ss; k++)
    {
        //=================================================================================
        //printf("entered_RIFT_for_one_kp %d times. Line %d\n", k, __LINE__);
        //=================================================================================

        //Care! From there, x is the rows and y is the cols, contrary to the defination before.

        //+++++++++++++++++++++++Very Essential!++++++++++++++++++++++++
        //==============================================================
        int y = (int)round(keypoints[k].pt.x); //pt.x is the cols, y is the cols
        int x = (int)round(keypoints[k].pt.y); //pt.y is the rows, x is the rows
        //I exchanged the x and y then the output kps became normal.
        //==============================================================
        //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        int pat_flo = floor(patch_size/2);
        int x1 = max(1, x-pat_flo);
        int y1 = max(1, y-pat_flo);
        int x2 = min(x+pat_flo, im.rows);  //??
        int y2 = min(y+pat_flo, im.cols);

        if ((y2-y1)!=patch_size || (x2-x1)!=patch_size)
        {
            kps_to_ignore.at<uchar>(0,k) = 1;
            continue;
        }

        //=================================================================================
        //printf("The %d kp is not ignored. Line %d\n", k, __LINE__);
        //=================================================================================

        //=================================================================
        /*
        for (int fa=0; fa< kps_to_ignore.cols; fa++)
        {
            printf("kps_to_ignore[%d]:%d\n", fa, kps_to_ignore.at<uchar>(0,fa));
        }
        */
        //=================================================================

        //Mat patch(y2-y1+1,x2-x1+1);
        //parameter of data type may be needed.
        // CARE!!! : And the order of x and y may cause potential problems.

        //---------This part I misunderstand "x1:x2" in python-----------
        //Therefore here is an adjust. Range(x1,x2) is ok.
        /*
        if(x2 == im.rows || y2 == im.cols)
        {
            kps_to_ignore.at<uchar>(0,k) = 1;
            continue;
        }
        */
        Mat patch = MIM(Range(x1,x2),Range(y1,y2));
        //-----------------------------------------------

        // Use of Range may cause some problems: Is patch just a pointer to MIM?
        //==================================================
        //Mat patch = MIM(Rect(y1, x1, y2-y1+1, x2-x1+1));
        //==================================================

        //Mat patch = Mat::zeros((x2-x1), (y2-y1), CV_8UC3);
        /*
        for(int i=x1; i<=x2; i++)
        {
            for(int j=y1; j<=y2; j++)
            {
                patch.at<Vec3b>(i-x1,j-y1)[0] = MIM.at<Vec3b>(i,j)[0];
                patch.at<Vec3b>(i-x1,j-y1)[1] = MIM.at<Vec3b>(i,j)[1];
                patch.at<Vec3b>(i-x1,j-y1)[2] = MIM.at<Vec3b>(i,j)[2];
            }
        }
        */

        int xs = patch.rows;
        int ys = patch.cols;
        //=================================================================================
        //printf("successful_patch_created %d times. Line %d\n", k, __LINE__);
        //=================================================================================

        //int ns = 6;

        //Mat RIFT_des = Mat(ns, ns, o, CV_8UC3); //descriptor vector

        //-------------TEST---------------------
        /*
        printf("%d\n", RIFT_des.size().width);
        printf("%d\n", RIFT_des.size().height);
        printf("total:%ld\n", RIFT_des.total());
        printf("depth:%d\n", RIFT_des.depth());
        */
        //--------------------------------------

        for(int a=0; a<ns; a++)
        {
            for(int b=0; b<ns; b++)
            {
                for(int c=0; c<o; c++)
                {
                    //printf("successful_RIFT_des_initialised_3.\n");
                    RIFT_des.at<float>(a,b,c) = 0;
                    //==============================
                    //RIFT_des.at<Vec3b>(a,b,c)[1] = 0;
                    //RIFT_des.at<Vec3b>(a,b,c)[2] = 0;
                    //==============================
                }
                //printf("successful_RIFT_des_initialised_2.\n");
            }
            //printf("successful_RIFT_des_initialised_1.\n");
        }
        //printf("successful_RIFT_des_initialised.\n");

        //Mat r_hist, g_hist, b_hist;
        Mat hist;
        //histogram vectors
        for (int j=1; j<ns+1; j++)
        {
            for (int i=1; i<ns+1; i++)
            {
                //Here: take care that i and j start with the initial value of 1.

                //=================================================================================
                //printf("entered_hist_calculated_once:%d\n",__LINE__);
                //=================================================================================
                /*
                for(int m=0; m<clip.rows; m++)
                {
                    for(int n=0; n<clip.cols; n++)
                    {

                    }
                }
                */
                Mat clip = patch(Range((int)(round((j-1)*xs/ns)), (int)(round(j*xs/ns))),
                                 Range((int)(round((i-1)*ys/ns)), (int)(round(i*ys/ns))));
                //printf("rows of clip:%d\n",clip.rows);
                //printf("cols of clip:%d\n",clip.cols);
                //-------------------------Abandoned version--------------------------
                /*
                Mat clip = patch(Rect((int)(round((i-1)*ys/ns)),(int)(round((j-1)*xs/ns)),
                                      (int)(round(i*ys/ns))-(int)(round((i-1)*ys/ns)),
                                      (int)(round(j*xs/ns))-(int)(round((j-1)*xs/ns))));
                */
                // Rect(width, height) = (cols, rows)
                //--------------------------------------------------------------------

                //+++++++++++++++++Here a potential problem may be laid+++++++++++++++

                //====================================================================
                /*
                //vector<Mat> rgb_clips;

                Mat clip = patch(Range((int)round((j-1)*xs/ns),(int)round(j*xs/ns)+1),
                                 Range((int)round((i-1)*ys/ns),(int)round(i*ys/ns)+1));
                */
                // Here I adjust the order of xs and ys in the edition
                //split(clip, rgb_clips);

                // And why abandoned the use of Range()?
                // 'cause Range(a,b) does not include the b,
                // then I use Range(a,b+1), there a error about index emerged:
                // Assertion failed (0 <= _colRange.start && _colRange.start <= _colRange.end && _colRange.end <= m.cols)
                //====================================================================

                //clip.reshape(0,1);

                for(int alpha=0; alpha<clip.rows; alpha++)
                {
                    for(int beta=0; beta<clip.cols; beta++)
                    {
                        switch(clip.at<uchar>(alpha,beta))
                        {
                        case 1:
                        {
                            RIFT_des.at<float>(j-1,i-1,0)++;
                            break;
                        }
                        case 2:
                        {
                            RIFT_des.at<float>(j-1,i-1,1)++;
                            break;
                        }
                        case 3:
                        {
                            RIFT_des.at<float>(j-1,i-1,2)++;
                            break;
                        }
                        case 4:
                        {
                            RIFT_des.at<float>(j-1,i-1,3)++;
                            break;
                        }
                        case 5:
                        {
                            RIFT_des.at<float>(j-1,i-1,4)++;
                            break;
                        }
                        case 6:
                        {
                            RIFT_des.at<float>(j-1,i-1,5)++;
                            break;
                        }
                        default:
                        {
                            cout <<"ERROR!" <<endl;
                        }
                        }
                    }
                }

                /*
                calcHist(&clip, 1, 0, //channels
                         Mat(), hist, 1, //dim
                         &histSize_RI, &histRange, true, false);
                         */
                //=================================================================================
                //printf("successful_hist_calculated_once:%d\n",__LINE__);
                //=================================================================================

                //========================================
                /*
                calcHist(&rgb_clips[1], 1, 0, //channels
                         Mat(), g_hist, 1, //dim
                         &histSize_RI, &histRange[0], true, false);
                calcHist(&rgb_clips[2], 1, 0, //channels
                         Mat(), b_hist, 1, //dim
                         &histSize_RI, &histRange[0], true, false);
                */

                //why I add a [0] to histRange and it works?
                // range()?
                //========================================
                //printf("rows of hist:%d\n", hist.rows);
                //printf("cols of hist:%d\n", hist.cols);

                /*
                for (int num=0; num<o; num++)
                {

                    RIFT_des.at<float>(j-1,i-1,num) = (float)hist.at<uchar>(num);
                    printf("the %d of hist:%d\n", num+1, hist.at<uchar>(num));
                    // Follow the order I detacted above!!!
                    //========================================================
                    //RIFT_des.at<Vec3b>(j-1,i-1,num)[0] = r_hist.at<uchar>(num);
                    //RIFT_des.at<Vec3b>(j-1,i-1,num)[1] = g_hist.at<uchar>(num);
                    //RIFT_des.at<Vec3b>(j-1,i-1,num)[2] = b_hist.at<uchar>(num);
                    //========================================================
                }
                */
            }
        }
        //printf("successful_hist_calculated:%d\n",__LINE__);

        //turn the matrix into a vector: ravel() in python.
        //vector<int> RIFT_Des =；
        // This part now can be fulfilled by the following cycles:
        for (int j=0; j<ns; j++)
        {
            for (int i=0; i<ns; i++)
            {
                for (int num=0; num<o; num++)
                {
                    des.at<float>(o*(ns*j+i)+num,k) = RIFT_des.at<float>(j,i,num);
                    //printf("the %d of hist:%f\n", num+1, RIFT_des.at<float>(j,i,num));
                    //=================================================================
                    /*
                    des.at<Vec3b>(o*(ns*j+i)+num,k)[0] = RIFT_des.at<Vec3b>(j,i,num)[0];
                    des.at<Vec3b>(o*(ns*j+i)+num,k)[1] = RIFT_des.at<Vec3b>(j,i,num)[1];
                    des.at<Vec3b>(o*(ns*j+i)+num,k)[2] = RIFT_des.at<Vec3b>(j,i,num)[2];
                    */
                    //=================================================================
                    // Now des is a (36*o, kps.size()) matrix
                }
            }
        }
        //=================================================================================
        //printf("successful_RIFT_for_one_kp_calculated: %d times. Line %d\n", k, __LINE__);
        //=================================================================================

        /*for (int num=0; num<36*o; num++)
        {
            des.at(num,k) = RIFT_des.at();
        }
        */
    }
    printf("successful_RIFT_calculated.\n");

    // Whether the matrix des should be transpose?
    //des = des.t();


    //===============================================================
    // Abandoned alternative codes for des_final

    std::vector<int> des_fin_single;
    std::vector<vector<int> > des_fin;
    for(int k=0; k<size_ss; k++)
    {
        //=================================================================================
        //printf("des_fin_valued_1D:%d times. line:%d.\n", k, __LINE__);
        //=================================================================================
        if(kps_to_ignore.at<uchar>(0,k) == 0)
        {
            for(int j=0; j<des.rows; j++)
            {
                des_fin_single.push_back((int)des.at<float>(j,k));
            }
            // After every cycle of j, remember to clear the des_fin_single!
            //=================================================================================
            //printf("size of des_fin_single:%ld, Line:%d.\n", des_fin_single.size(), __LINE__);
            //=================================================================================
            des_fin.push_back(des_fin_single);
            des_fin_single.clear();
        }
        else
        {
            continue;
        }
    }
    printf("size of des_fin:%ld, Line:%d.\n", des_fin.size(), __LINE__);
    printf("cols of des_fin:%ld, Line:%d.\n", des_fin[0].size(), __LINE__);
    printf("rows of des:%d, Line:%d.\n", des.rows, __LINE__);
    printf("cols of des:%d, Line:%d.\n", des.cols, __LINE__);
    printf("successful_des_fin_valued:%d.\n",__LINE__);

    Mat des_final = Mat(des_fin.size(), des.rows, CV_32F);
    for(int k=0; k<des_fin.size(); k++)
    {
        //=================================================================================
        //printf("entered_des_final_envalued %d times. Line:%d\n", k, __LINE__);
        //=================================================================================
        for(int j=0; j<des.rows; j++)
        {
            //=================================================================================
            //printf("entered_des_final_envalued for single kp %d times. Line:%d\n", j, __LINE__);
            //=================================================================================
            des_final.at<float>(k,j) = des_fin[k][j];
        }
    }
    //Mat des_final{des_fin}; // use vector to initialize the matrix
    //Mat des_final = Mat::zeros(1,1,CV_8U);

    //===============================================================

    // Update: define of kps moved to header file.
    //std::vector<keypoints> kps;

    //Former vector keypoints is "n*2"

    //===============================================================
    // The part of valid keypoint has problems, while it is not needed in the following processing, so abandoned.

    //++++++The block below are used for display of kps_to_ignore++++++++
    /*
    for (int fa=0; fa< kps_to_ignore.cols; fa++)
    {
        printf("kps_to_ignore[%d]:%d\n", fa, kps_to_ignore.at<uchar>(0,fa));
    }
    */
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    for(int k=0; k<size_ss; k++)
    {
        if(kps_to_ignore.at<uchar>(0,k) == 0)
        {
            kps.push_back(keypoints[k]);
        }
    }
    printf("successful_kps_envalued. Line:%d.\n",__LINE__);
    //===============================================================

    printf("number of extracted kps:%ld, Line:%d.\n", keypoints.size(), __LINE__);
    printf("number of filtered kps:%ld, Line:%d.\n", kps.size(), __LINE__);
    printf("rows of des:%d, Line:%d.\n", des_final.rows, __LINE__);
    printf("cols of des:%d, Line:%d.\n", des_final.cols, __LINE__);
    return des_final;
}
