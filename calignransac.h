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
