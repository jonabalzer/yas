#-------------------------------------------------
#
# Project created by QtCreator 2013-02-13T09:35:45
#
#-------------------------------------------------

QT       += core gui opengl

TARGET = kinect_scan
TEMPLATE = app

SOURCES += main.cpp\
        mainwindow.cpp \
    viewerwindow.cpp \
    pcviewer.cpp \
    alignwindow.cpp \
    calignransac.cpp

HEADERS  += mainwindow.h \
    viewerwindow.h \
    pcviewer.h \
    alignwindow.h \
    calignransac.h

FORMS    += mainwindow.ui \
    viewerwindow.ui \
    alignwindow.ui

unix:!symbian: LIBS += -L/usr/local/lib/ -L/opt/local/lib/ -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_video -lopencv_features2d -lopencv_nonfree -lHalf -lIlmImf

INCLUDEPATH += /opt/local/include
INCLUDEPATH += /opt/local/include/OpenEXR
INCLUDEPATH += /opt/local/include
