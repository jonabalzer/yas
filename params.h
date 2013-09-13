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

#ifndef PARAMS_H
#define PARAMS_H

#include <QWidget>
#include "dsensor.h"

namespace Ui {
class Params;
}

class Params : public QWidget
{
    Q_OBJECT
    
public:

    //! Constructor.
    explicit Params(QWidget *parent = 0);

    //! Destructor.
    ~Params();

    //! Access to alignement parameters from main window.
    void get_alignment_parameters(size_t& nfeat, size_t& noctaves, double& pthresh, double& ethresh, double& ratio, size_t& nsamples, double& athresh);

    //! Access to Poisson parameter from main window.
    void get_reconstruction_parameters(int& depth, float& weight, float& samples, float& scale, int& niter, float& acc, bool& confidence, bool& polygon);

    //! Access to centering flag.
    bool center_wc();

    float get_triangulation_threshold();

    bool ply_binary();

    bool warp_to_rgb();

    unsigned short get_no_icp_steps();

signals:

    void cam_params_changed(const CCam& rgb, const CDepthCam& depth);

    void save_params_clicked();

    void load_params_clicked();

public slots:

    void on_applyButton_clicked();

private slots:

    void on_saveButton_clicked();

    void on_loadButton_clicked();

    void on_loadErrorPatternButton_clicked();

private:

    Ui::Params* ui;
    cv::Mat m_D;

};

#endif // PARAMS_H
