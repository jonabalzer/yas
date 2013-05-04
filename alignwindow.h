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
