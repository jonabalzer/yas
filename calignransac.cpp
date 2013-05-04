#include "calignransac.h"

#include <opencv2/nonfree/nonfree.hpp>

#include <QProgressDialog>

#include <random>
#include <chrono>

CAlignRansac::CAlignRansac(vector<Vec3f>& x0, vector<Vec3f>&x1):
    m_x0(x0),
    m_x1(x1) {}

bool CAlignRansac::EstimateMotion(vector<size_t> inds, Mat& F) {

    // compute barycenters
    Vec3f bc0, bc1;
    bc0 *= 0;
    bc1 *= 0;

    for(size_t k=0; k<inds.size(); k++) {

        bc0 += m_x0[inds[k]];
        bc1 += m_x1[inds[k]];

    }

    float nc = 1.0/(float)inds.size();

    bc0 *= nc;
    bc1 *= nc;

    // compute matrix H
    Mat H = Mat::zeros(3,3,CV_32FC1);
    Mat temp = Mat::zeros(3,3,CV_32FC1);

    for(size_t k=0; k<inds.size(); k++) {

        Vec3f p =  m_x0[inds[k]] - bc0;
        Vec3f q =  m_x1[inds[k]] - bc1;

        temp.at<float>(0,0) = q[0]*p[0];
        temp.at<float>(0,1) = q[0]*p[1];
        temp.at<float>(0,2) = q[0]*p[2];
        temp.at<float>(1,0) = q[1]*p[0];
        temp.at<float>(1,1) = q[1]*p[1];
        temp.at<float>(1,2) = q[1]*p[2];
        temp.at<float>(2,0) = q[2]*p[0];
        temp.at<float>(2,1) = q[2]*p[1];
        temp.at<float>(2,2) = q[2]*p[2];

        H = H + temp;

    }

    // perform SVD decomposition
    Mat Sigma, U, Vt;
    SVD::compute(H, Sigma, U, Vt);

    // assemble result
    Mat Rt = U*Vt;
    Mat R = Rt.t();

    Mat bc01 = R*Mat(bc1);
    Mat t = Mat(bc0) - bc01;

    // erase F and copy result to it
    F *= 0;
    Rect rp(0,0,3,3);
    Mat Fr = F(rp);
    R.copyTo(Fr);

    F.at<float>(0,3) = t.at<float>(0,0);
    F.at<float>(1,3) = t.at<float>(1,0);
    F.at<float>(2,3) = t.at<float>(2,0);
    F.at<float>(3,3) = 1;

    // check determinant
    //float det = cv::determinant(R);
    return 0;

}

vector<size_t> CAlignRansac::EvaluateHypothesis(const Mat& F, double tolerance) {

    vector<size_t> inliers;

    // extract R, t from F
    Mat R = F(Range(0,3), Range(0, 3));
    Mat t = F(Range(0,3),Range(3,4));

    // test
    for(size_t k=0; k<m_x0.size(); k++) {

        // cast points to Mat
        Mat res = Mat(m_x0[k]) - R*Mat(m_x1[k]) - t;
        double resn = cv::norm(res);

        if(resn<tolerance)
            inliers.push_back(k);

    }

    return inliers;

}


Mat CAlignRansac::RunConcensus(size_t nosamples, double tol, size_t& ninliers, QWidget* parent) {

    // seed for random number generator
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    default_random_engine generator(seed);
    uniform_int_distribution<size_t> distribution(0,m_x0.size()-1);

    Mat result(4,4,CV_32FC1);
    size_t max = 0;
    vector<size_t> ilmax;

    QProgressDialog progress("Consensus in progress...", "Abort", 0,nosamples,parent);
    progress.setWindowTitle("KinectScan");
    progress.setWindowModality(Qt::WindowModal);

    for(size_t k=0; k<nosamples; k++) {

        progress.setValue(k);

        if (progress.wasCanceled())
            break;

        // draw three random indices of source points
        vector<size_t> inds;
        inds.push_back(distribution(generator));
        inds.push_back(distribution(generator));
        inds.push_back(distribution(generator));

        // make sure the indices are all different
        if(!(inds[0]==inds[1] && inds[1]==inds[2] && inds[0]==inds[2])) {

            // calculate hypothesis
            EstimateMotion(inds,result);

            // judge hypothesis
            vector<size_t> inliers = EvaluateHypothesis(result,tol);

            if(inliers.size()>max) {

                max = inliers.size();
                ilmax = inliers;            // keep the biggest inlier set

            }

        }


    }

    progress.setValue(nosamples);

    // estimate motion with the inlier set
    EstimateMotion(ilmax,result);

    ninliers = ilmax.size();

    return result;

}

