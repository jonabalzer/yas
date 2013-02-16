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
    pcviewer.cpp

HEADERS  += mainwindow.h \
    viewerwindow.h \
    pcviewer.h

FORMS    += mainwindow.ui \
    viewerwindow.ui

unix:!symbian: LIBS += -L/usr/local/lib/ -lopencv_core -lopencv_highgui -lopencv_video -lHalf -lIlmImf -lGLU -lglut

INCLUDEPATH += /usr/local/include
INCLUDEPATH += /usr/include/OpenEXR
