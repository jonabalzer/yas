#include "cplaneransac.h"
#include <QProgressDialog>

#include <fstream>
#include <random>
#include <chrono>

using namespace std;
using namespace cv;

Vec4f CEstimatePlaneRansac::RunConsensus(size_t nsamples, float threshold, size_t& ninliers, QWidget* parent) {

    // init random number generator
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    default_random_engine generator(seed);
    uniform_int_distribution<size_t> distribution(0,m_x.size()-1);

    Vec4f result;
    size_t max = 0;
    vector<size_t> ilmax;

    QProgressDialog progress("Consensus in progress...", "Abort", 0,nsamples,parent);
    progress.setWindowTitle("Ground plane estimation");
    progress.setWindowModality(Qt::WindowModal);

    for(size_t k=0; k<nsamples; k++) {

        progress.setValue(k);

        if (progress.wasCanceled())
            break;

        // draw three random indices of source points
        vector<size_t> inds;
        inds.push_back(distribution(generator));
        inds.push_back(distribution(generator));
        inds.push_back(distribution(generator));

        // make sure the indices are all different
        if(!(inds[0]==inds[1] && inds[1]==inds[2])) {

            // calculate hypothesis
            EstimatePlane(inds,result);

            // judge hypothesis
            vector<size_t> inliers = EvaluateHypothesis(result,threshold);

            if(inliers.size()>max) {

                max = inliers.size();
                ilmax = inliers;            // keep the biggest inlier set

            }

        }


    }

    progress.setValue(nsamples);

    // number of inliers
    ninliers = ilmax.size();

    // if there are too many inliers reduce the number
    if(ninliers>=1000) {

        unsigned int seed2 = std::chrono::system_clock::now().time_since_epoch().count();
        default_random_engine generator2(seed2);
        uniform_int_distribution<size_t> distribution2(0,ninliers-1);

        vector<size_t> ilfinal;

        // pick random samples
        for(size_t i=0; i<1000; i++)
            ilfinal.push_back(ilmax[distribution2(generator2)]);

        EstimatePlaneWithInliers(ilfinal,result);


    } else
        EstimatePlaneWithInliers(ilmax,result);

    return result;

}


bool CEstimatePlaneRansac::EstimatePlane(vector<size_t> inds, Vec4f& P) {

    if(inds.size()!=3)
        return 1;

    Vec3f t0 = m_x[inds[2]] - m_x[inds[0]];
    Vec3f t1 = m_x[inds[1]] - m_x[inds[0]];

    // estimate normal
    Vec3f n;
    n[0] = t0[1]*t1[2] - t0[2]*t1[1];
    n[1] = t0[2]*t1[0] - t0[0]*t1[2];
    n[2] = t0[0]*t1[1] - t0[1]*t1[0];

    float nnorm = cv::norm(n);

    // linear dependence
    if(nnorm==0)
        return 1;

    n = n*(1.0/nnorm);

    // make sure all hypotheses are aligned
    if(n[2]<0)
        n *= -1;

    // get d
    P[0] = n[0];
    P[1] = n[1];
    P[2] = n[2];
    P[3] = n[0]*m_x[inds[0]].x + n[1]*m_x[inds[0]].y + n[2]*m_x[inds[0]].z;

    return 0;

}

vector<size_t> CEstimatePlaneRansac::EvaluateHypothesis(const Vec4f& P, float tolerance) {

    vector<size_t> inliers;

    // test
    for(size_t k=0; k<m_x.size(); k++) {

        // cast points to Mat
        float resn = fabs(m_x[k].x*P[0]+m_x[k].y*P[1]+m_x[k].z*P[2]-P[3]);

        if(resn<tolerance)
            inliers.push_back(k);

    }

    return inliers;

}


bool CEstimatePlaneRansac::EstimatePlaneWithInliers(std::vector<size_t> inds, cv::Vec4f& P) {

    Mat T = Mat::zeros(inds.size(),3,CV_32FC1);

    Vec3f mean;
    mean *= 0;

    for(size_t i=0; i<inds.size(); i++) {

            T.at<float>(i,0) = m_x[inds[i]].x;
            T.at<float>(i,1) = m_x[inds[i]].y;
            T.at<float>(i,2) = m_x[inds[i]].z;
            mean[0] += m_x[inds[i]].x;
            mean[1] += m_x[inds[i]].y;
            mean[2] += m_x[inds[i]].z;

    }

    // first guess for origin (project onto dominant plane later)
    mean *= (1.0/(float)(inds.size()));

    // remove mean
    for(size_t i=0; i<(size_t)T.rows; i++) {

        for(size_t j=0; j<3; j++)
            T.at<float>(i,j) -= mean[j];

    }

    Mat Sigma, U, Vt;
    SVD::compute(T, Sigma, U, Vt);

    // set normal (already normalized)
    for(size_t j=0; j<3; j++)
        P[j] = Vt.at<float>(2,j);

    // best distance is the mean distance
    float meandist = 0;

    for(size_t i=0; i<inds.size(); i++)
        meandist += P[0]*m_x[inds[i]].x + P[1]*m_x[inds[i]].y + P[2]*m_x[inds[i]].z;

    P[3] = meandist/((float)(inds.size()));

    return 0;

}


