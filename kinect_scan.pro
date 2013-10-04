#-------------------------------------------------
#
# Project created by QtCreator 2013-02-13T09:35:45
#
#-------------------------------------------------

QT       += core gui opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = yas
TEMPLATE = app

SOURCES += main.cpp\
        mainwindow.cpp \
        pcviewer.cpp \
        alignwindow.cpp \
        calignransac.cpp \
        cplaneransac.cpp \
        cam.cpp \
        dsensor.cpp \
        glwidget.cpp \
        params.cpp \
        darray.cpp \
        iter.cpp \
        precond.cpp \
        kernels.cpp \
        lm.cpp \
        icp.cpp \
        vecn.cpp \
        trafo.cpp \
        poisson/PlyFile.cpp \
        poisson/MarchingCubes.cpp \
        poisson/Geometry.cpp \
        poisson/Factor.cpp \
        poisson/Ply.cpp \
        poisson/Octree.cpp \
        tgCamera.cpp

HEADERS  += mainwindow.h \
            pcviewer.h \
            alignwindow.h \
            calignransac.h \
            cplaneransac.h \
            cam.h \
            dsensor.h \
            glwidget.h \
            params.h \
            darray.h \
            iter.h \
            precond.h \
            kernels.h \
            lm.h \
            types.h \
            icp.h \
            vecn.h \
            trafo.h \
            poisson/Vector.h \
            poisson/Time.h \
            poisson/SparseMatrix.h \
            poisson/PPolynomial.h \
            poisson/Polynomial.h \
            poisson/PointStream.h \
            poisson/PlyFile.h \
            poisson/Ply.h \
            poisson/Octree.h \
            poisson/MultiGridOctreeData.h \
            poisson/MAT.h \
            poisson/MarchingCubes.h \
            poisson/Hash.h \
            poisson/Geometry.h \
            poisson/FunctionData.h \
            poisson/Factor.h \
            poisson/BSplineData.h \
            poisson/BinaryNode.h \
            poisson/Array.h \
            poisson/Allocator.h \
            tgMathlib.h \
            tgCamera.h

FORMS    += mainwindow.ui \
            params.ui \
            alignwindow.ui

unix:!symbian {

QMAKE_CXXFLAGS += -fopenmp -fpermissive -std=c++0x -O3

QMAKE_LFLAGS += -Wl,-rpath=.

LIBS += -L$$OPENNI_DIR/Redist/ \
        -L$$OPENNI_DIR/Redist/OpenNI2/Drivers \
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
        -lPS1080 \
        -lann \
        -lgomp

INCLUDEPATH += /usr/local/include \
               /usr/include/OpenEXR \
               $$OPENNI_DIR/Include \

}

unix:!mac:!symbian {

    DEFINES += linux

}

mac:!linux:!symbian {

INCLUDEPATH += /opt/local/include \             # macports installs stuff in /opt/local
               /opt/local/include/OpenEXR

DEFINES += __APPLE__                            # for typedefs of unsigned types

}
