#ifndef PHASE_H
#define PHASE_H

#endif // PHASE_H


#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <vector>

#define _USE_MATH_DEFINES
#include <math.h>

#include <opencv2/core/types.hpp>

using namespace std;
using namespace cv;

namespace cv
{
    class _InputArray;
    class _OutputArray;
    typedef const _InputArray& InputArray;
    typedef const _OutputArray& OutputArray;
}

struct PhaseCongruencyConst {
    double sigma;
    double mult = 1.6;  //1.6
    double minwavelength = 3;
    double epsilon = 0.0001;
    double cutOff = 0.45;
    double g = 3;  //3
    double k = 1.0;  //1
    PhaseCongruencyConst();
    PhaseCongruencyConst(const PhaseCongruencyConst& _pcc);
    PhaseCongruencyConst& operator=(const PhaseCongruencyConst& _pcc);
};

class PhaseCongruency
{
public:
    PhaseCongruency(cv::Size _img_size, size_t _nscale, size_t _norient);
    ~PhaseCongruency() {}
    void setConst(PhaseCongruencyConst _pcc);
    void calc(cv::InputArray _src, std::vector<cv::Mat> &_pc);
    void feature(std::vector<cv::Mat> &_pc, cv::OutputArray _edges, cv::OutputArray _corners);
    void feature(cv::InputArray _src, cv::OutputArray _edges, cv::OutputArray _corners);
    //std::vector<vector<cv::Mat> >eo = std::vector<vector<cv::Mat> >(6, vector<Mat>(4));
    //std::vector<vector<cv::Mat> > eo;
    std::vector<cv::Mat> eo;
    cv::Mat maxMoment;

private:
    cv::Size size;
    size_t norient;
    size_t nscale;

    PhaseCongruencyConst pcc;

    std::vector<cv::Mat> filter;

    //Mat s1, s2, s3, s4, d1, d2, d3, d4;
};
