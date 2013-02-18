#include "calignransac.h"
#include <opencv2/nonfree/nonfree.hpp>
#include <QProgressDialog>

CAlignRansac::CAlignRansac(float fu, float fv, float cu, float cv):
    m_x0(),
    m_x1() {

    m_f[0] = fu;
    m_f[1] = fv;
    m_c[0] = cu;
    m_c[1] = cv;

}

Mat CAlignRansac::GenerateHypotheses(Mat& rgb0, Mat& rgb1, Mat& depth0, Mat& depth1, double threshold, double zmax) {

    // init non-free module
    initModule_nonfree();

    Ptr<FeatureDetector> detector = FeatureDetector::create("SIFT");
    //detector->set("threshold",50);

    // detect
    vector<KeyPoint> kp0, kp1;
    detector->detect(rgb0,kp0);
    detector->detect(rgb1,kp1);

    // compute descriptors
    Ptr<DescriptorExtractor> extractor = DescriptorExtractor::create("SIFT");
    Mat desc0, desc1;
    extractor->compute(rgb0, kp0, desc0);
    extractor->compute(rgb1, kp1, desc1);

    // matching
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-L1");
    vector<vector<DMatch> > matches;
    matcher->knnMatch(desc0,desc1,matches,2);

    // compute 3d points
    vector<DMatch> good_matches;
    for (size_t i = 0; i<matches.size(); i++) {

        // check if match is good
        if(matches[i][0].distance/matches[i][1].distance<threshold) {

            // get both locations
            Point2f u0, u1;
            u0 = kp0[matches[i][0].queryIdx].pt;
            u1 = kp1[matches[i][0].trainIdx].pt;

            // get depth values
            float z0, z1;
            z0 = (float)depth0.at<unsigned short>((size_t)u0.y,(size_t)u0.x);
            z1 = (float)depth1.at<unsigned short>((size_t)u1.y,(size_t)u1.x);

            // if both points have depth, compute their 3d location
            if(z0>0 && z0<(float)zmax && z1>0 && z1<(float)zmax) {

                Vec3f x0, x1;
                x0[0] = (z0/m_f[0])*(u0.x-m_c[0]);
                x0[1] = (z0/m_f[1])*(u0.y-m_c[1]);
                x0[2] = z0;
                x1[0] = (z1/m_f[0])*(u1.x-m_c[0]);
                x1[1] = (z1/m_f[1])*(u1.y-m_c[1]);
                x1[2] = z1;

                m_x0.push_back(x0);
                m_x1.push_back(x1);
                good_matches.push_back(matches[i][0]);

            }

        }

    }

    // create of good matches visualization
    Mat img_matches;
    drawMatches(rgb0,
                kp0,
                rgb1,
                kp1,
                good_matches,
                img_matches,
                Scalar::all(-1),
                Scalar::all(-1),
                vector<char>(),
                DrawMatchesFlags::DEFAULT);

    return img_matches;

}


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


Mat CAlignRansac::RunConcensus(size_t nosamples, double tol, QWidget* parent) {

    Mat result(4,4,CV_32FC1);
    size_t max = 0;
    vector<size_t> ilmax;

    QProgressDialog progress("Consensus in progress...", "Abort", 0,nosamples,parent);
    progress.setWindowModality(Qt::WindowModal);


    for(size_t k=0; k<nosamples; k++) {

        progress.setValue(k);

        if (progress.wasCanceled())
            break;

        // draw three random indices of source points
        vector<size_t> inds;
        inds.push_back(rand()%m_x0.size());
        inds.push_back(rand()%m_x0.size());
        inds.push_back(rand()%m_x0.size());

        // make sure the indices are all different
        if(!(inds[0]==inds[1] && inds[1]==inds[2])) {

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

    return result;

}

