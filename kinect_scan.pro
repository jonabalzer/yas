#-------------------------------------------------
#
# Project created by QtCreator 2013-02-13T09:35:45
#
#-------------------------------------------------

QT       += core gui opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = kinect_scan
TEMPLATE = app

SOURCES += main.cpp\
        mainwindow.cpp \
        viewerwindow.cpp \
        pcviewer.cpp \
        alignwindow.cpp \
        calignransac.cpp \
        cplaneransac.cpp \
        psensors.cpp \
    cam.cpp

HEADERS  += mainwindow.h \
            viewerwindow.h \
            pcviewer.h \
            alignwindow.h \
            calignransac.h \
            cplaneransac.h \
            psensors.h \
    cam.h

FORMS    += mainwindow.ui \
            viewerwindow.ui \
            alignwindow.ui

unix:!symbian: {

LIBS += -L$$PWD/../../OpenNI-2.1.0-x64/Redist \
        -L$$PWD/../../OpenNI-2.1.0-x64/Redist/OpenNI2/Drivers \
        -L/usr/local/lib/ \
        -lopencv_core \
        -lopencv_highgui \
        -lopencv_video \
        -lopencv_imgproc \
        -lopencv_features2d \
        -lopencv_nonfree \
        -lHalf \
        -lIlmImf \
        -lOpenNI2 \
        -lOniFile \
        -lPS1080

INCLUDEPATH += /usr/local/include \
               /usr/include/OpenEXR \
               $$PWD/../../OpenNI-2.1.0-x64/Include

}

mac:!symbian: {

    LIBS += -L/usr/local/lib/ -L/opt/local/lib/ -lopencv_core -lopencv_highgui -lopencv_video -lopencv_imgproc -lopencv_features2d -lopencv_nonfree -lHalf -lIlmImf

    INCLUDEPATH += /usr/local/include
    INCLUDEPATH += /usr/include/OpenEXR
    INCLUDEPATH += /opt/local/include
    INCLUDEPATH += /opt/local/include/OpenEXR


}
