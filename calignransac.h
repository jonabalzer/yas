#ifndef CALIGNRANSAC_H
#define CALIGNRANSAC_H

#include <QWidget>
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

class CAlignRansac
{
public:

    CAlignRansac(vector<Vec3f>& x0, vector<Vec3f>& x1);

    Mat RunConcensus(size_t nosamples, double tol, size_t& ninliers, QWidget* parent = 0);

private:

    bool EstimateMotion(vector<size_t> inds, Mat& F);

    vector<size_t> EvaluateHypothesis(const Mat& F, double tolerance);

    vector<Vec3f>& m_x0;
    vector<Vec3f>& m_x1;

};

#endif // CALIGNRANSAC_H
