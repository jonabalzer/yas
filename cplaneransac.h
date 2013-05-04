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

#ifndef CPLANERANSAC_H
#define CPLANERANSAC_H

#include <QWidget>
#include "opencv2/opencv.hpp"

class CEstimatePlaneRansac {


public:

    CEstimatePlaneRansac(const std::vector<cv::Point3f>& x):m_x(x){}

    cv::Vec4f RunConsensus(size_t nsamples, float threshold, size_t& ninliers, QWidget* parent = 0);

private:

    bool EstimatePlane(std::vector<size_t> inds, cv::Vec4f& P);

    bool EstimatePlaneWithInliers(std::vector<size_t> inds, cv::Vec4f& P);

    std::vector<size_t> EvaluateHypothesis(const cv::Vec4f& P, float tolerance);

    const std::vector<cv::Point3f>& m_x;

};


#endif // CPLANERANSAC_H
