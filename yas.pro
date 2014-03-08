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
        alignwindow.cpp \
        calignransac.cpp \
        cplaneransac.cpp \
        cam.cpp \
        dsensor.cpp \
        glwidget.cpp \
        params.cpp \
        darray.cpp \
        iter.cpp \
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
        tgCamera.cpp \
        tgModel.cpp

HEADERS  += mainwindow.h \
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
            tgCamera.h \
            tgModel.h

FORMS    += mainwindow.ui \
            params.ui \
            alignwindow.ui

QMAKE_CXXFLAGS += -fopenmp -fpermissive -std=c++0x -O3

QMAKE_LFLAGS += -Wl,-rpath=.

# see whether we can find the driver
!exists($$OPENNI_DIR) {
    error("Could not configure project because the location of the OpenNI driver seems to be incorrect.")
}

unix:!symbian {

# see if all necessary packages exist
!packagesExist(opencv OpenEXR) {
    error("Some of the required dependencies are missing.")
}

!exists(/usr/lib/libann.so) {
    error("Some of the required dependencies are missing.")
}

CONFIG += link_pkgconfig

PKGCONFIG += opencv \
             OpenEXR

LIBS += -L$$OPENNI_DIR/Redist/ \
        -L$$OPENNI_DIR/Redist/OpenNI2/Drivers \
        -lOpenNI2 \
        -lOniFile \
        -lPS1080 \
        -lann \
        -lgomp

INCLUDEPATH += $$OPENNI_DIR/Include

}

mac:!linux:!symbian {

INCLUDEPATH += /opt/local/include \             # macports installs stuff in /opt/local
               /opt/local/include/OpenEXR \
               /usr/local/include

# pkg-config does not work on mac by default, so add these manually
LIBS += -L/usr/local/lib/ \
        -lopencv_core \
        -lopencv_highgui \
        -lopencv_video \
        -lopencv_imgproc \
        -lopencv_features2d \
        -lopencv_nonfree \
        -lHalf \
        -lIex \
        -lIlmThread \
        -lIlmImf \


DEFINES += __APPLE__   # for typedefs of unsigned types

}
