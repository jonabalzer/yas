#include "params.h"
#include "ui_params.h"
#include <opencv2/opencv.hpp>
#include <vector>

using namespace cv;
using namespace std;

Params::Params(CDepthColorSensor* psensor, QWidget *parent):
    QWidget(parent),
    ui(new Ui::Params)
{
    ui->setupUi(this);
}

Params::~Params()
{
    delete ui;
}

void Params::init() {

    CDepthColorSensor::ShowAvailableSensors();



}

void Params::configure_sensor(CDepthColorSensor* sensor) {

    // get params
    vector<size_t> srgb, sd;
    vector<float> frgb, fd, crgb, cd, krgb, kd, range, dd, da;
    srgb.push_back(640);
    srgb.push_back(480);
    frgb.push_back(ui->fuRgbEdit->text().toFloat());
    frgb.push_back(ui->fvRgbEdit->text().toFloat());
    crgb.push_back(ui->cuRgbEdit->text().toFloat());
    crgb.push_back(ui->cvRgbEdit->text().toFloat());
    krgb.push_back(ui->k0Edit->text().toFloat());
    krgb.push_back(ui->k1Edit->text().toFloat());
    krgb.push_back(ui->k2Edit->text().toFloat());
    krgb.push_back(ui->k3Edit->text().toFloat());
    krgb.push_back(ui->k4Edit->text().toFloat());

    Mat F = Mat::eye(4,4,CV_32FC1);
    F.at<float>(0,0) = ui->r11Edit->text().toFloat();
    F.at<float>(0,1) = ui->r12Edit->text().toFloat();
    F.at<float>(0,2) = ui->r13Edit->text().toFloat();
    F.at<float>(0,3) = ui->txEdit->text().toFloat();
    F.at<float>(1,0) = ui->r21Edit->text().toFloat();
    F.at<float>(1,1) = ui->r22Edit->text().toFloat();
    F.at<float>(1,2) = ui->r23Edit->text().toFloat();
    F.at<float>(1,3) = ui->tyEdit->text().toFloat();
    F.at<float>(2,0) = ui->r31Edit->text().toFloat();
    F.at<float>(2,1) = ui->r32Edit->text().toFloat();
    F.at<float>(2,2) = ui->r33Edit->text().toFloat();
    F.at<float>(2,3) = ui->tzEdit->text().toFloat();

    sensor->ConfigureRGB(srgb,frgb,crgb,0,krgb,F);

    sd.push_back(640);
    sd.push_back(480);
    fd.push_back(ui->fuEdit->text().toFloat());
    fd.push_back(ui->fvEdit->text().toFloat());
    cd.push_back(ui->cuEdit->text().toFloat());
    cd.push_back(ui->cvEdit->text().toFloat());
    kd.push_back(0);
    kd.push_back(0);
    kd.push_back(0);
    kd.push_back(0);
    kd.push_back(0);
    range.push_back(ui->zminEdit->text().toFloat());
    range.push_back(ui->zmaxEdit->text().toFloat());
    dd.push_back(ui->c0Edit->text().toFloat());
    dd.push_back(ui->c1Edit->text().toFloat());
    da.push_back(0);
    da.push_back(0);

    sensor->ConfigureDepth(sd,fd,cd,0,kd,Mat::eye(4,4,CV_32FC1),range,dd,Mat::zeros(sd[1],sd[0],CV_32FC1),da);

}

void Params::get_alignment_parameters(size_t& nfeat, size_t& noctaves, double& pthresh, double& ethresh, double& ratio, size_t& nsamples, double& athresh) {

    nfeat = ui->noFeatEdit->text().toInt();
    noctaves = ui->noOctavesEdit->text().toInt();
    pthresh = ui->pointThresholdEdit->text().toDouble();
    ethresh = ui->edgeThresholdEdit->text().toDouble();
    ratio = ui->goodMatchEdit->text().toDouble();
    nsamples = ui->nosamplesEdit->text().toInt();
    athresh = ui->acceptanceEdit->text().toDouble();

}


bool Params::center_wc() {

    return ui->centerCheckBox->isChecked();

}


float Params::get_triangulation_threshold() {

    return ui->triEdit->text().toFloat();

}

bool Params::triangulate() {

    return ui->triangulateCheckBox->isChecked();

}



