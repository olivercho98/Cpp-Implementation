#include "phase.h"

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <vector>

#define _USE_MATH_DEFINES
#include <math.h>

using namespace cv;

// Rearrange the quadrants of Fourier image so that the origin is at
// the image center
void shiftDFT(InputArray _src, OutputArray _dst)
{
    Mat src = _src.getMat();
    Size size = src.size();

    _dst.create(size, src.type());
    auto dst = _dst.getMat();

    const int cx = size.width / 2;
    const int cy = size.height / 2; // image center

    Mat s1 = src(Rect(0, 0, cx, cy));
    Mat s2 = src(Rect(cx, 0, cx, cy));
    Mat s3 = src(Rect(cx, cy, cx, cy));
    Mat s4 = src(Rect(0, cy, cx, cy));

    Mat d1 = dst(Rect(0, 0, cx, cy));
    Mat d2 = dst(Rect(cx, 0, cx, cy));
    Mat d3 = dst(Rect(cx, cy, cx, cy));
    Mat d4 = dst(Rect(0, cy, cx, cy));

    Mat tmp;
    s3.copyTo(tmp);
    s1.copyTo(d3);
    tmp.copyTo(d1);

    s4.copyTo(tmp);
    s2.copyTo(d4);
    tmp.copyTo(d2);
}

#define MAT_TYPE CV_64FC1
#define MAT_TYPE_CNV CV_64F

// Making a filter
// src & dst arrays of equal size & type
PhaseCongruency::PhaseCongruency(Size _size, size_t _nscale, size_t _norient)
{
    size = _size;
    nscale = _nscale;
    norient = _norient;

    filter.resize(nscale * norient);

    const int dft_M = getOptimalDFTSize(_size.height); //nrows
    const int dft_N = getOptimalDFTSize(_size.width);  //ncols

    Mat radius = Mat::zeros(dft_M, dft_N, MAT_TYPE);
    Mat normradius = Mat::zeros(dft_M, dft_N, MAT_TYPE);
    Mat matAr[2];
    matAr[0] = Mat::zeros(dft_M, dft_N, MAT_TYPE);
    matAr[1] = Mat::zeros(dft_M, dft_N, MAT_TYPE);
    Mat lp = Mat::zeros(dft_M, dft_N, MAT_TYPE);
    Mat angular = Mat::zeros(dft_M, dft_N, MAT_TYPE);
    std::vector<Mat> gabor(nscale);

    
    //Matrix values contain *normalised* radius
    // values ranging from 0 at the centre to
    // 0.5 at the boundary.
    
    const int dft_M_2 = floor(dft_M / 2);  //cy
    const int dft_N_2 = floor(dft_N / 2);  //cx
    //int r;
    //int y_norm = dft_M_2;
    //int x_norm = dft_N_2;
    //if (dft_M > dft_N) r = dft_N_2;
    //else r = dft_M_2;
    const double dr_y = 1.0 / static_cast<double>(dft_M);
    const double dr_x = 1.0 / static_cast<double>(dft_N);
    for (int row = 0; row < dft_M; row++)
    {
        auto radius_row = radius.ptr<double>(row);
        for (int col = 0; col < dft_N; col++)
        {
            double m = (row - dft_M_2);  //y-cy
            double n = (col - dft_N_2);  //x-cx
            
            m = m * dr_y;
            n = n * dr_x;
            
            radius_row[col] = sqrt(static_cast<double>(m * m + n * n));
        }
    }
    
    radius.at<double>(dft_M_2, dft_N_2) = 1.0;  //radius[cy, cx] = 1

    normradius = radius * 1.0; //abs(x).max()*2, abs(x).max() is 0.5
    lp = normradius / pcc.cutOff;
    pow(lp, 30.0, lp);
    lp += Scalar::all(1.0);
    
    // The following implements the log-gabor transfer function.
    double mt = 1.0f;
    for (int scale = 0; scale < nscale; scale++)
    {
        const double wavelength = pcc.minwavelength * mt;  //wavelength = minWavelength*mult*s
        
        gabor[scale] = radius * wavelength;
        log(gabor[scale], gabor[scale]);
        pow(gabor[scale], 2.0, gabor[scale]); //log(radius/fo)**2
        gabor[scale] *= pcc.sigma;
        exp(gabor[scale], gabor[scale]);
        
        divide(gabor[scale], lp, gabor[scale]);  //logGabor*lowpassbutterworth
        gabor[scale].at<double>(dft_M_2, dft_N_2) = 0.0;
        mt = mt * pcc.mult;
    }

    const double angle_const = static_cast<double>(M_PI) / static_cast<double>(norient);  //pi/6
    for (int ori = 0; ori < norient; ori++)
    {
        double angl = (double)ori * angle_const;
        //Now we calculate the angular component that controls the orientation selectivity of the filter.
        for (int i = 0; i < dft_M; i++)  //nrows
        {
            auto angular_row = angular.ptr<double>(i);
            for (int j = 0; j < dft_N; j++)  //ncols
            {
                double m = atan2(-((double)i / (double)dft_M - 0.5), (double)j / (double)dft_N - 0.5);

                double s = sin(m);
                double c = cos(m);
                m = s * cos(angl) - c * sin(angl);
                //ds
                double n = c * cos(angl) + s * sin(angl);
                //dc
                s = fabs(atan2(m, n));
                //dtheta

                angular_row[j] = (cos(min(s * (double)norient * 0.5, M_PI)) + 1.0) * 0.5;
                // In this part, min(s * (double)norient * 0.5, M_PI) means new dtheta correspondingly,
                // and angular_row is the "spread".
            }
        }

        for (int scale = 0; scale < nscale; scale++)
        {
            multiply(gabor[scale], angular, matAr[0]); //Product of the two components.
            //the corresponding "filter"

            merge(matAr, 2, filter[nscale * ori + scale]);
            // Here, problem with merge()
            // matAr[0] and [1] represent the real part and imag part
        }//scale
    }//orientation
    //Filter ready
}

void PhaseCongruency::setConst(PhaseCongruencyConst _pcc)
{
    pcc = _pcc;
}

//Phase congruency calculation
void PhaseCongruency::calc(InputArray _src, std::vector<cv::Mat> &_pc)
{
    Mat src = _src.getMat();

    CV_Assert(src.size() == size);

    const int width = size.width, height = size.height;

    Mat src64;
    src.convertTo(src64, MAT_TYPE_CNV, 1.0 / 255.0);

    const int dft_M_r = getOptimalDFTSize(src.rows) - src.rows;
    const int dft_N_c = getOptimalDFTSize(src.cols) - src.cols;

    _pc.resize(norient);

    //std::vector<Mat> eo(nscale);
    // THE eo. And here I expand it to a (o,s) vector according to former matlab version.
    //std::vector<vector<Mat> >eo(norient, vector<Mat>(nscale));
    // New update: defined in header file.
    //std::vector<vector<cv::Mat> >eo = std::vector<vector<cv::Mat> >(6, vector<Mat>(4));
    std::vector<cv::Mat> trans(nscale);

    Mat complex[2];
    Mat sumAn;
    Mat sumRe;
    Mat sumIm;
    Mat maxAn;
    Mat xEnergy;
    Mat tmp;
    Mat tmp1;
    Mat tmp2;
    Mat energy = Mat::zeros(size, MAT_TYPE);

    //expand input image to optimal size
    Mat padded;
    copyMakeBorder(src64, padded, 0, dft_M_r, 0, dft_N_c, BORDER_CONSTANT, Scalar::all(0));
    Mat planes[] = { Mat_<double>(padded), Mat::zeros(padded.size(), MAT_TYPE_CNV) };

    Mat dft_A;
    merge(planes, 2, dft_A);         // Add to the expanded another plane with zeros
    dft(dft_A, dft_A);            // this way the result may fit in the source matrix

    shiftDFT(dft_A, dft_A);

    for (unsigned o = 0; o < norient; o++)
    {
        double noise = 0;
        for (unsigned scale = 0; scale < nscale; scale++)
        {
            Mat filtered;
            mulSpectrums(dft_A, filter[nscale * o + scale], filtered, 0); // Convolution
            // filtered is the matrix for result
            dft(filtered, filtered, DFT_INVERSE);
            // then do the IFFT and still save in "filtered"
            //filtered(Rect(0, 0, width, height)).copyTo(eo[o][scale]);
            trans[scale] = filtered(Rect(0, 0, width, height)).clone();
            //filtered(Rect(0, 0, width, height)).copyTo(trans[scale]);
            //Rect() create a rectangle with the size of (width,height), size of the picture
            eo.push_back(trans[scale]);
            //printf("type of trans:%d, Line:%d.\n", trans[scale].type(), __LINE__);

            split(trans[scale], complex);
            Mat eo_mag;
            //sumAn_ThisOrient
            magnitude(complex[0], complex[1], eo_mag);
            //complex[0][1] are inputs which means the real and imagination parts.
            //eo_mag is the output

            if (scale == 0)
            {
                //here to do noise threshold calculation
                //===========================================================
                auto tau = mean(eo_mag);
                tau.val[0] = tau.val[0] / sqrt(log(4.0));
                auto mt = 1.0 * pow(pcc.mult, nscale);
                auto totalTau = tau.val[0] * (1.0 - 1.0 / mt) / (1.0 - 1.0 / pcc.mult);
                //totalTau:python Line 327
                auto m = totalTau * sqrt(M_PI / 2.0);
                //EstNoiseEnergyMean
                auto n = totalTau * sqrt((4 - M_PI) / 2.0);
                //EstNoiseEnergySigma: values of noise energy
                noise = m + pcc.k * n;
                // T: noise threshold. Python line 335

                //xnoise = 0;
                //complex[0] -= xnoise;
                //max(complex[0], 0.0, complex[0]);
                //===========================================================

                eo_mag.copyTo(maxAn);
                eo_mag.copyTo(sumAn);
                complex[0].copyTo(sumRe);
                complex[1].copyTo(sumIm);
            }
            else
            {
                //complex[0] -= xnoise;
                //max(complex[0], 0.0, complex[0]);
                add(sumAn, eo_mag, sumAn);
                max(eo_mag, maxAn, maxAn);
                add(sumRe, complex[0], sumRe);
                add(sumIm, complex[1], sumIm);
            }
            //eo.push_back(trans[scale]);
        } // next scale
        //eo.push_back(trans);

        magnitude(sumRe, sumIm, xEnergy);
        xEnergy += pcc.epsilon;
        divide(sumIm, xEnergy, sumIm);
        divide(sumRe, xEnergy, sumRe);
        energy.setTo(0);
        for (int scale = 0; scale < nscale; scale++)
        {
            split(trans[scale], complex);

            multiply(complex[0], sumIm, tmp1);
            multiply(complex[1], sumRe, tmp2);

            absdiff(tmp1, tmp2, tmp);
            subtract(energy, tmp, energy);

            multiply(complex[0], sumRe, complex[0]);
            add(energy, complex[0], energy);
            multiply(complex[1], sumIm, complex[1]);
            add(energy, complex[1], energy);
            /*if (o == 0 && scale == 2)
            {
                energy -= noise / norient;
                max(energy, 0.0, energy);
                normalize(energy, tmp, 0, 1, NORM_MINMAX);
                imshow("energy", tmp);
            }*/
        } //next scale
        trans.clear();

        energy -= Scalar::all(noise); // -noise
        max(energy, 0.0, energy);
        maxAn += pcc.epsilon;

        divide(sumAn, maxAn, tmp, -1.0 / static_cast<double>(nscale));

        tmp += pcc.cutOff;
        tmp = tmp * pcc.g;
        exp(tmp, tmp);
        tmp += 1.0; // 1 / weight

        //PC
        multiply(tmp, sumAn, tmp);
        divide(energy, tmp, _pc[o]);
    }//orientation
}

//Build up covariance data for every point
void PhaseCongruency::feature(std::vector<cv::Mat>& _pc, cv::OutputArray _edges, cv::OutputArray _corners)
{
    _edges.create(size, CV_8UC1);
    _corners.create(size, CV_8UC1);
    auto edges = _edges.getMat();
    auto corners = _corners.getMat();

    Mat covx2 = Mat::zeros(size, MAT_TYPE);
    Mat covy2 = Mat::zeros(size, MAT_TYPE);
    Mat covxy = Mat::zeros(size, MAT_TYPE);
    Mat cos_pc, sin_pc, mul_pc;

    const double angle_const = M_PI / static_cast<double>(norient);

    for (unsigned o = 0; o < norient; o++)
    {
        auto angl = static_cast<double>(o) * angle_const;
        cos_pc = _pc[o] * cos(angl);
        sin_pc = _pc[o] * sin(angl);
        multiply(cos_pc, sin_pc, mul_pc);
        add(covxy, mul_pc, covxy);
        pow(cos_pc, 2, cos_pc);
        add(covx2, cos_pc, covx2);
        pow(sin_pc, 2, sin_pc);
        add(covy2, sin_pc, covy2);
    } // next orientation

      //Edges calculations
    covx2 *= 2.0 / static_cast<double>(norient);
    covy2 *= 2.0 / static_cast<double>(norient);
    covxy *= 4.0 / static_cast<double>(norient);
    Mat sub;
    subtract(covx2, covy2, sub);

    //denom += Scalar::all(epsilon);
    Mat denom;
    magnitude(sub, covxy, denom); // denom;
    Mat sum;
    add(covy2, covx2, sum);

    // There are also an update: maxMoment defined in header file.
    //Mat minMoment, maxMoment;
    Mat minMoment;
    subtract(sum, denom, minMoment);//m = (covy2 + covx2 - denom) / 2;          % ... and minimum moment
    add(sum, denom, maxMoment); //M = (covy2+covx2 + denom)/2;          % Maximum moment

    maxMoment.convertTo(edges, CV_8U, 255);
    minMoment.convertTo(corners, CV_8U, 255);
}

//Build up covariance data for every point
void PhaseCongruency::feature(InputArray _src, cv::OutputArray _edges, cv::OutputArray _corners)
{
    std::vector<cv::Mat> pc;
    calc(_src, pc);
    feature(pc, _edges, _corners);
}

PhaseCongruencyConst::PhaseCongruencyConst()
{
    sigma = -1.0 / (2.0 * log(0.75) * log(0.75));
    //sigma = 0.65;
    //number in the log is the corresponding value in the python version
}

PhaseCongruencyConst::PhaseCongruencyConst(const PhaseCongruencyConst & _pcc)
{
    sigma = _pcc.sigma;
    mult = _pcc.mult;
    minwavelength = _pcc.minwavelength;
    epsilon = _pcc.epsilon;
    cutOff = _pcc.cutOff;
    g = _pcc.g;
    k = _pcc.k;
}

PhaseCongruencyConst& PhaseCongruencyConst::operator=(const PhaseCongruencyConst & _pcc)
{
    if (this == &_pcc) {
        return *this;
    }
    sigma = _pcc.sigma;
    mult = _pcc.mult;
    minwavelength = _pcc.minwavelength;
    epsilon = _pcc.epsilon;
    cutOff = _pcc.cutOff;
    g = _pcc.g;
    k = _pcc.k;

    return *this;
}
