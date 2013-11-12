
set(V4R_CALIBRATION 1 CACHE BOOL "Calibration modules of v4r" )
if(V4R_CALIBRATION)

project(v4rCalibration)

#set(OPENNI_DIR "~/git-repositories/OpenNI-2.1.0-x64")


find_package(Qt4 REQUIRED)
find_package(OpenCV REQUIRED)

set(QT_USE_QTOPENGL TRUE)
include(${QT_USE_FILE})

add_definitions(${QT_DEFINITIONS})
add_definitions(-Dlinux -D__x86_64__)

include_directories(${QT_INCLUDE_DIR})
include_directories(${QT_INCLUDE_DIR}/QtOpenGL)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

include_directories(/usr/include/OpenEXR)
include_directories(${OPENNI_DIR}/Include)

message(STATUS "QT_INCLUDE_DIR: ${QT_INCLUDE_DIR}")
message(STATUS "QT_USE_FILE: ${QT_USE_FILE}")
message(STATUS "QT_DEFINITIONS: ${QT_DEFINITIONS}")
message(STATUS "QT_LIBRARIES: ${QT_LIBRARIES}")


set(SOURCES
        main.cpp
        mainwindow.cpp
        alignwindow.cpp
        calignransac.cpp
        cplaneransac.cpp
        cam.cpp
        dsensor.cpp
        glwidget.cpp
        params.cpp
        darray.cpp
        iter.cpp
        kernels.cpp
        lm.cpp
        icp.cpp
        vecn.cpp
        trafo.cpp
        poisson/PlyFile.cpp
        poisson/MarchingCubes.cpp
        poisson/Geometry.cpp
        poisson/Factor.cpp
        poisson/Ply.cpp
        poisson/Octree.cpp
        tgCamera.cpp
        tgModel.cpp
)

set(HEADERS
        mainwindow.h
        alignwindow.h
        calignransac.h
        cplaneransac.h
        cam.h
        dsensor.h
        glwidget.h
        params.h
        darray.h
        iter.h
        precond.h
        kernels.h
        lm.h
        types.h
        icp.h
        vecn.h
        trafo.h
        poisson/Vector.h
        poisson/Time.h
        poisson/SparseMatrix.h
        poisson/PPolynomial.h
        poisson/Polynomial.h
        poisson/PointStream.h
        poisson/PlyFile.h
        poisson/Ply.h
        poisson/Octree.h
        poisson/MultiGridOctreeData.h
        poisson/MAT.h
        poisson/MarchingCubes.h
        poisson/Hash.h
        poisson/Geometry.h
        poisson/FunctionData.h
        poisson/Factor.h
        poisson/BSplineData.h
        poisson/BinaryNode.h
        poisson/Array.h
        poisson/Allocator.h
        tgMathlib.h
        tgCamera.h
        tgModel.h
)

set(FORMS
        mainwindow.ui
        params.ui
        alignwindow.ui
)

QT4_WRAP_CPP(HEADERS_MOC ${HEADERS})
QT4_WRAP_UI(FORMS_MOC ${FORMS})


set(CMAKE_CXX_FLAGS "-fopenmp -fpermissive -std=c++0x -O3 ${CMAKE_CXX_FLAGS}")	#neccesary for R4R
#set(CMAKE_LFLAGS "-Wl,-rpath=.")

link_directories(${OPENNI_DIR}/Redist/)
link_directories(${OPENNI_DIR}/Redist/OpenNI2/Drivers)


add_executable(yas ${SOURCES} ${HEADERS_MOC} ${FORMS_MOC})
target_link_libraries(yas opencv_core opencv_highgui opencv_video opencv_imgproc opencv_features2d opencv_nonfree)
target_link_libraries(yas Half Iex IlmThread IlmImf OpenNI2 OniFile PS1080 ann gomp)
target_link_libraries(yas ${QT_LIBRARIES} GL)


endif(V4R_CALIBRATION)
