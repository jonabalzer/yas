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
    explicit Params(CDepthColorSensor* psensor, QWidget *parent = 0);
    ~Params();

    void configure_sensor(CDepthColorSensor* sensor);

    void get_alignment_parameters(size_t& nfeat, size_t& noctaves, double& pthresh, double& ethresh, double& ratio, size_t& nsamples, double& athresh);

    bool center_wc();

    float get_triangulation_threshold();

    bool triangulate();

private slots:


private:


    Ui::Params* ui;


    void init();

};

#endif // PARAMS_H
