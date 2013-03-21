#include "params.h"
#include "ui_params.h"

Params::Params(CDepthColorSensor* psensor, QWidget *parent):
    m_psensor(psensor),
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
