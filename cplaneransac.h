#ifndef CPLANERANSAC_H
#define CPLANERANSAC_H

#include <QWidget>
#include "opencv2/opencv.hpp"

class CEstimatePlaneRansac {


public:

    CEstimatePlaneRansac(const std::vector<cv::Point3f>& x):m_x(x){};

    cv::Vec4f RunConsensus(size_t nsamples, float threshold, size_t& ninliers, QWidget* parent = 0);

private:

    bool EstimatePlane(std::vector<size_t> inds, cv::Vec4f& P);

    bool EstimatePlaneWithInliers(std::vector<size_t> inds, cv::Vec4f& P);

    std::vector<size_t> EvaluateHypothesis(const cv::Vec4f& P, float tolerance);

    const std::vector<cv::Point3f>& m_x;

};


#endif // CPLANERANSAC_H
