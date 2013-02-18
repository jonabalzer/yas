#ifndef VIEWERWINDOW_H
#define VIEWERWINDOW_H

#include <QMainWindow>

namespace Ui {
class ViewerWindow;
}

class ViewerWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit ViewerWindow(QWidget *parent = 0);
    ~ViewerWindow();
    
private slots:
    void on_pushButton_clicked();

private:
    Ui::ViewerWindow *ui;
};

#endif // VIEWERWINDOW_H