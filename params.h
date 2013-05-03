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

    //! Access to centering flag.
    bool center_wc();

    float get_triangulation_threshold();

    bool triangulate();

    bool warp_to_rgb();

signals:

    void cam_params_changed(const CCam& rgb, const CDepthCam& depth);

    void save_params_clicked();

    void load_params_clicked();

public slots:

    void on_applyButton_clicked();

private slots:

    void on_saveButton_clicked();

    void on_loadButton_clicked();

private:

    Ui::Params* ui;


};

#endif // PARAMS_H
