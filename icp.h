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

#ifndef R4RICP_H
#define R4RICP_H

#include "lm.h"
#include "types.h"

#include <opencv2/opencv.hpp>
#include <ANN/ANN.h>

#include <vector>
#include <map>

/*! \brief iterative-closest point algorithm
 *
 * \details This class implements the point-to-plane variant.
 *
 *
*/
class CPointToPlaneICP {

public:

    //! Constructor.
    CPointToPlaneICP(std::vector<cv::Point3f>& x0, std::vector<cv::Point3f>& n0, std::vector<cv::Point3f>& x1, cv::Mat F0);

    //! Destructor.
    ~CPointToPlaneICP();

    //! Triggers execution of \f$n\f$ iterations.
    bool Iterate(size_t n);

    //! Converts the result into an OpenCV mat.
    cv::Mat GetResult();

private:

    ANNpointArray m_x0a;                 //!< target point cloud, FIXME: replace by OpenCV NN
    ANNkd_tree* m_tree;                 //!< kd-tree for target point cloud
    std::vector<cv::Point3f>& m_x0;     //!< target point cloud
    std::vector<cv::Point3f>& m_n0;     //!< normals on target point cloud
    std::vector<cv::Point3f>& m_x1;     //!< source point cloud
    std::map<int,int> m_corr;           //!< correspondences
    vec m_g;                            //!< parameters of the desired transformation
    size_t m_maxpts;                    //!< maximal number of points

    //! Step 1 one of the iteration.
    bool UpdateCorrespondences();

    //! Step 2 one of the iteration.
    bool UpdateMotion();

};


/*! \brief least-squares functional appearing in the point-to-plane ICP algorithm
 *
 *
 *
 *
*/
class CPointToPlaneICPFunctional:public CLeastSquaresProblem<mat> {

public:

    //! Constructor.
    CPointToPlaneICPFunctional(std::vector<cv::Point3f>& x0, std::vector<cv::Point3f>& n0, std::vector<cv::Point3f>& x1, std::map<int,int>& corr);

    //! \copydoc CLeastSquaresProblem::ComputeResidualAndJacobian(vec&,Matrix&)
    virtual void ComputeResidualAndJacobian(vec& r, mat& J);

    //! \copydoc CLeastSquaresProblem::ComputeResidual(vec&)
    virtual void ComputeResidual(vec& r);

private:

    std::vector<cv::Point3f>& m_x0;       //!< target point cloud
    std::vector<cv::Point3f>& m_n0;       //!< normals on target point cloud
    std::vector<cv::Point3f>& m_x1;       //!< source point cloud
    std::map<int,int>& m_corr;            //!< set of correpondences

};


#endif // ICP_H
