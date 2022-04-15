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
    yim = im.rows;
    xim = im.cols;

    // CS is a 3D matrix
    int size_cs[] = {yim, xim, o};

    int res_cs = yim*xim*o;
    double *d_cs = new double[res_cs];

    for(int i=0; i<res_cs; i++)
    {
        d_cs[i] = 0;
    }
    CS = Mat(3, size_cs, CV_64F, d_cs);
    // 3D matrix for convolution sequence

    for(int j=0; j<o; j++)
    {
        for(int i=0; i<s; i++)
        {
            for(int m=0; m<yim; m++)
            {
                for(int n=0; n<xim; n++)
                {
                    CS.at<double>(m,n,j) = CS.at<double>(m,n,j)+sqrt(static_cast<double>(eo[j*s+i].at<Vec2d>(m,n)[0] * eo[j*s+i].at<Vec2d>(m,n)[0]
                            + eo[j*s+i].at<Vec2d>(m,n)[1] * eo[j*s+i].at<Vec2d>(m,n)[1]));
                }
            }
        }
    }

    std::complex<double> z1 {eo[0*s+0].at<Vec2d>(0,0)[0], eo[0*s+0].at<Vec2d>(0,0)[1]};
    double z1_manual = sqrt(static_cast<double>(eo[0].at<Vec2d>(0,0)[0] * eo[0].at<Vec2d>(0,0)[0]
                    + eo[0].at<Vec2d>(0,0)[1] * eo[0].at<Vec2d>(0,0)[1]));

    double z2_manual = sqrt(static_cast<double>(eo[0].at<Vec2d>(25,25)[0] * eo[0].at<Vec2d>(25,25)[0]
                    + eo[0].at<Vec2d>(25,25)[1] * eo[0].at<Vec2d>(25,25)[1]));

    std::complex<double> z2 {eo[0*s+0].at<Vec2d>(25,25)[0], eo[0*s+0].at<Vec2d>(25,25)[1]};

    Mat MIM = Mat(yim, xim, CV_8U);
    // MIM is used for the storage of index of min/max

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
            }
            // MIM_Xsec stores the max orient for a fixed pair of x and y.
            // Note that CS is changed from a 3-channel to a single channel.

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

            MIM.at<uchar>(i,j) = maxIdx+1;
        }
    }

    int size_ss = keypoints.size();
    int size_des[] = {36*o, size_ss};

    int res_des = 36*o*size_ss;
    float *d_des = new float[res_des];

    for(int i=0; i<res_des; i++)
    {
        d_des[i] = 0;
    }

    des = Mat(2, size_des, CV_32F, d_des);
    // 2nd dim of kps

    // For 2D vectors, .size() means length of rows, [i].size() means length of cols
    Mat kps_to_ignore = Mat::zeros(1,(int)size_ss, CV_16U);

    //+++++++++This part is quite essential+++++++++++++
    float hrange[] = {1, 6};
    const float*histRange = { hrange };
    //++++++++++++++++++++++++++++++++++++++++++++++++++

    int ns = 6;
    int size_R_des[] = {ns, ns, o};

    int res_R_des = ns*ns*o;
    float *d_R_des = new float[res_R_des];

    for(int i=0; i<res_R_des; i++)
    {
        d_R_des[i] = 0;
    }

    Mat RIFT_des = Mat(3, size_R_des, CV_32F, d_R_des);

    for (int k=0; k<size_ss; k++)
    {
        //Note that from there x denotes the rows and y denotes the cols, contrary to the defination before.

        //==============================================================
        int y = (int)round(keypoints[k].pt.x);  //pt.x is the cols, y is the cols
        int x = (int)round(keypoints[k].pt.y);  //pt.y is the rows, x is the rows
        //==============================================================

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

        Mat patch = MIM(Range(x1,x2),Range(y1,y2));

        int xs = patch.rows;
        int ys = patch.cols;

        for(int a=0; a<ns; a++)
        {
            for(int b=0; b<ns; b++)
            {
                for(int c=0; c<o; c++)
                {
                    RIFT_des.at<float>(a,b,c) = 0;
                }
            }
        }

        Mat hist;
        //histogram vectors
        for (int j=1; j<ns+1; j++)
        {
            for (int i=1; i<ns+1; i++)
            {
                //Note that i and j start with the initial value of 1.
                Mat clip = patch(Range((int)(round((j-1)*xs/ns)), (int)(round(j*xs/ns))),
                                 Range((int)(round((i-1)*ys/ns)), (int)(round(i*ys/ns))));

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
            }
        }

        // Turn the matrix into a vector: ravel() in python.
        // This part now can be fulfilled by the following cycles:
        for (int j=0; j<ns; j++)
        {
            for (int i=0; i<ns; i++)
            {
                for (int num=0; num<o; num++)
                {
                    des.at<float>(o*(ns*j+i)+num,k) = RIFT_des.at<float>(j,i,num);
                    // Now des is a (36*o, kps.size()) matrix
                }
            }
        }
    }

    std::vector<int> des_fin_single;
    std::vector<vector<int> > des_fin;
    for(int k=0; k<size_ss; k++)
    {
        if(kps_to_ignore.at<uchar>(0,k) == 0)
        {
            for(int j=0; j<des.rows; j++)
            {
                des_fin_single.push_back((int)des.at<float>(j,k));
            }
            // After every cycle of j, just clear the des_fin_single
            des_fin.push_back(des_fin_single);
            des_fin_single.clear();
        }
        else
        {
            continue;
        }
    }

    Mat des_final = Mat(des_fin.size(), des.rows, CV_32F);
    for(int k=0; k<des_fin.size(); k++)
    {
        for(int j=0; j<des.rows; j++)
        {
            des_final.at<float>(k,j) = des_fin[k][j];
        }
    }

    for(int k=0; k<size_ss; k++)
    {
        if(kps_to_ignore.at<uchar>(0,k) == 0)
        {
            kps.push_back(keypoints[k]);
        }
    }

    return des_final;
}
