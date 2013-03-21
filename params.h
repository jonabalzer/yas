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

    Ui::Params* ui;

private:

    CDepthColorSensor* m_psensor;

    void init();

};

#endif // PARAMS_H
