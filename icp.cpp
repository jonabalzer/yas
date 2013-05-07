/*////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2013, Jonathan Balzer
//
// All rights reserved.
//
// This file is part of the R4R library.
//
// The R4R library is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// The R4R library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with the R4R library. If not, see <http://www.gnu.org/licenses/>.
//
////////////////////////////////////////////////////////////////////////////////*/

#include "icp.h"
#include "trafo.h"
#include "iter.h"

#include <random>
#include <chrono>

using namespace std;
using namespace cv;

CPointToPlaneICP::CPointToPlaneICP(vector<Point3f>& x0, vector<Point3f>& n0, vector<Point3f>& x1, Mat F0):
    m_x0(x0),
    m_n0(n0),
    m_x1(x1),
    m_corr(),
    m_g(6),
    m_maxpts(100000) {

    // data in col-major ordering
    double pdata[9];
    pdata[0] = F0.at<float>(0,0);
    pdata[1] = F0.at<float>(1,0);
    pdata[2] = F0.at<float>(2,0);
    pdata[3] = F0.at<float>(0,1);
    pdata[4] = F0.at<float>(1,1);
    pdata[5] = F0.at<float>(2,1);
    pdata[6] = F0.at<float>(0,2);
    pdata[7] = F0.at<float>(1,2);
    pdata[8] = F0.at<float>(2,2);

    // take the logarithm to convert rotation part into axis-angle form
    CRotation<double,3>::Log(pdata,m_g(0),m_g(1),m_g(2));

    // copy translations
    m_g(3) = F0.at<float>(0,3);
    m_g(4) = F0.at<float>(1,3);
    m_g(5) = F0.at<float>(2,3);

    // copy target points into ann data structure
    m_x0a = annAllocPts(x0.size(), 3);

    for(size_t i=0; i<x0.size(); i++) {

        ANNpoint vertex = annAllocPt(3, 0);
        vertex[0] = x0[i].x;
        vertex[1] = x0[i].y;
        vertex[2] = x0[i].z;

        m_x0a[i] = vertex;

    }

    // build tree
    m_tree = new ANNkd_tree(m_x0a,x0.size(),3);

}

CPointToPlaneICP::~CPointToPlaneICP() {

    delete m_tree;
    annDeallocPts(m_x0a);

}

bool CPointToPlaneICP::UpdateCorrespondences() {

    // temp variables for ANN
    ANNidxArray idx = new ANNidx[1];
    ANNdistArray dist = new ANNdist[1];

    // compute current transformation
    CRigidMotion<float,3> trafo(m_g.Get(0),m_g.Get(1),m_g.Get(2),m_g.Get(3),m_g.Get(4),m_g.Get(5));

    // if m_corr is empty fill it first
    if(m_corr.size()==0) {

        // subsample if necessary
        if(m_x1.size()>m_maxpts) {

            unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
            default_random_engine generator(seed);
            uniform_int_distribution<size_t> distribution(0,m_x1.size()-1);

            for(size_t i=0; i<m_maxpts; i++)
                m_corr.insert(pair<int,int>(distribution(generator),-1));

        }
        else {

            for(size_t i=0; i<m_x1.size(); i++)
                 m_corr.insert(pair<int,int>(i,-1));

        }

    }

    // update
    map<int,int>::iterator it;
    vec distances(m_corr.size());
    size_t i = 0;

    for(it=m_corr.begin(); it!=m_corr.end(); it++, i++) {

        vec3f temp = trafo.Transform(&((Vec3f)m_x1[it->first])[0]);

        ANNpoint vertex = annAllocPt(3, 0);
        vertex[0] = temp.Get(0);
        vertex[1] = temp.Get(1);
        vertex[2] = temp.Get(2);

        // find closest point
        m_tree->annkSearch(vertex,1,idx,dist,0);

        // store squared distance
        distances(i) = dist[0];

        // inject correspondence
        m_corr[it->first] = idx[0];

    }

    delete [] idx;
    delete [] dist;

    // estimate covariance
    double mad = distances.MAD();
    cout << "MAD: " << mad << endl;

    // delete outliers
    map<int,int> goodmatches;

    i=0;
    for(it=m_corr.begin(); it!=m_corr.end(); it++, i++) {

        if(distances(i)<1.4826*mad)
            goodmatches.insert(*it);

    }

    m_corr = goodmatches;

    return 0;

}

bool CPointToPlaneICP::UpdateMotion() {

    // init preconditioned solver
    mat M(0,0);
    CPreconditioner<mat,vec,double> precond = CPreconditioner<mat,vec,double>(M);
    CIterativeSolver<mat,vec,double> solver = CIterativeSolver<mat,vec,double>(precond,50,1e-12,true);

    // init least-squares problem
    CPointToPlaneICPFunctional problem(m_x0,m_n0,m_x1,m_corr);

    // init lm module
    CLevenbergMarquardt<mat> lms(problem,solver,0);

    // initialize iteration with actual motion estimate
    vec& model = problem.Get();
    memcpy(model.Data().get(),m_g.Data().get(),6*sizeof(double));

    // iterate
    lms.Iterate(100,1e-10,1e-10,false);
    //lms.Iterate(10,2,1e-10,true,false);

    // copy solution back
    memcpy(m_g.Data().get(),model.Data().get(),6*sizeof(double));

    return 0;

}

bool CPointToPlaneICP::Iterate(size_t n) {

    for(size_t i=0; i<n; i++) {

        UpdateCorrespondences();
        UpdateMotion();

    }

    return 0;

}

Mat CPointToPlaneICP::GetResult() {

    Mat result = Mat::eye(4,4,CV_32FC1);

    CRigidMotion<float,3> trafo(m_g.Get(0),m_g.Get(1),m_g.Get(2),m_g.Get(3),m_g.Get(4),m_g.Get(5));

    for(size_t i=0; i<3; i++) {

        for(size_t j=0; j<4; j++)
            result.at<float>(i,j) = trafo(i,j);

    }

    return result;

}

CPointToPlaneICPFunctional::CPointToPlaneICPFunctional(std::vector<cv::Point3f>& x0, vector<Point3f>& n0, vector<Point3f>& x1, std::map<int,int>& corr):
    CLeastSquaresProblem<mat>::CLeastSquaresProblem(corr.size(),6),
    m_x0(x0),
    m_n0(n0),
    m_x1(x1),
    m_corr(corr) {}

void CPointToPlaneICPFunctional::ComputeResidual(vec& r) {

    // compute actual transform
    CRigidMotion<float,3> F(m_model.Get(0),m_model.Get(1),m_model.Get(2),m_model.Get(3),m_model.Get(4),m_model.Get(5));

    // compute residual (warning, we are loosing precision here)
    map<int,int>::iterator it;
    size_t counter = 0;

    for(it=m_corr.begin(); it!=m_corr.end(); it++) {

        if(it->second>=0) {

            Point3f n0 = m_n0[it->second];
            Point3f x0 = m_x0[it->second];
            vec3f x1t = F.Transform(&((Vec3f)m_x1[it->first])[0]);

            r(counter) = n0.x*(x1t.Get(0)-x0.x) + n0.y*(x1t.Get(1)-x0.y) + n0.z*(x1t.Get(2)-x0.z);

        }
        else
            r(counter) = 0;

        counter++;

    }

}

void CPointToPlaneICPFunctional::ComputeResidualAndJacobian(vec& r, mat& J) {

    // compute actual transform
    CRigidMotion<float,3> F(m_model.Get(0),m_model.Get(1),m_model.Get(2),m_model.Get(3),m_model.Get(4),m_model.Get(5));

    // derivatives of R
    CDifferentialRotation<float,3> Rx(m_model.Get(0),m_model.Get(1),m_model.Get(2),0);
    CDifferentialRotation<float,3> Ry(m_model.Get(0),m_model.Get(1),m_model.Get(2),1);
    CDifferentialRotation<float,3> Rz(m_model.Get(0),m_model.Get(1),m_model.Get(2),2);

    map<int,int>::iterator it;
    size_t counter = 0;

    for(it=m_corr.begin(); it!=m_corr.end(); it++) {

        if(it->second>=0) {

            Vec3f x1 = (Vec3f)m_x1[it->first];

            // residual            
            Point3f n0 = m_n0[it->second];
            Point3f x0 = m_x0[it->second];
            vec3f x1t = F.Transform(&x1[0]);
            r(counter) = n0.x*(x1t.Get(0)-x0.x) + n0.y*(x1t.Get(1)-x0.y) + n0.z*(x1t.Get(2)-x0.z);

            // rotations first
            vec3f x1tx = Rx.Transform(&x1[0]);
            vec3f x1ty = Ry.Transform(&x1[0]);
            vec3f x1tz = Rz.Transform(&x1[0]);

            J(counter,0) = n0.x*x1tx.Get(0) + n0.y*x1tx.Get(1) + n0.z*x1tx.Get(2);
            J(counter,1) = n0.x*x1ty.Get(0) + n0.y*x1ty.Get(1) + n0.z*x1ty.Get(2);
            J(counter,2) = n0.x*x1tz.Get(0) + n0.y*x1tz.Get(1) +n0.z*x1tz.Get(2);

            // derivatives w.r.t. translation
            J(counter,3) = n0.x;
            J(counter,4) = n0.y;
            J(counter,5) = n0.z;



        } else {

            r(counter) = 0;
            J(counter,0) = 0;
            J(counter,1) = 0;
            J(counter,2) = 0;
            J(counter,3) = 0;
            J(counter,4) = 0;
            J(counter,5) = 0;

        }

        counter++;

    }

}

