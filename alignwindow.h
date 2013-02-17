#ifndef ALIGNWINDOW_H
#define ALIGNWINDOW_H

#include <QMainWindow>

namespace Ui {
class AlignWindow;
}

class AlignWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit AlignWindow(QWidget *parent = 0);
    ~AlignWindow();
    
    void show_image(QImage img);

private:
    Ui::AlignWindow *ui;

};

#endif // ALIGNWINDOW_H
