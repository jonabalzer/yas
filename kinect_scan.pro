#-------------------------------------------------
#
# Project created by QtCreator 2013-02-13T09:35:45
#
#-------------------------------------------------

QT       += core gui

TARGET = kinect_scan
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp

HEADERS  += mainwindow.h

FORMS    += mainwindow.ui

unix:!symbian: LIBS += -L$$PWD/../../../../../usr/local/lib/ -lopencv_core -lopencv_highgui -lopencv_video

INCLUDEPATH += $$PWD/../../../../../usr/local/include
DEPENDPATH += $$PWD/../../../../../usr/local/include

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../../usr/lib/release/ -lIlmImf
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../../usr/lib/debug/ -lIlmImf
else:unix:!symbian: LIBS += -L$$PWD/../../../../../usr/lib/ -lIlmImf

INCLUDEPATH += $$PWD/../../../../../usr/include/OpenEXR
DEPENDPATH += $$PWD/../../../../../usr/include/OpenEXR

win32:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/../../../../../usr/lib/release/IlmImf.lib
else:win32:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/../../../../../usr/lib/debug/IlmImf.lib
else:unix:!symbian: PRE_TARGETDEPS += $$PWD/../../../../../usr/lib/libIlmImf.a

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../../usr/lib/release/ -lHalf
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../../usr/lib/debug/ -lHalf
else:unix:!symbian: LIBS += -L$$PWD/../../../../../usr/lib/ -lHalf

INCLUDEPATH += $$PWD/../../../../../usr/include/OpenEXR
DEPENDPATH += $$PWD/../../../../../usr/include/OpenEXR

win32:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/../../../../../usr/lib/release/Half.lib
else:win32:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/../../../../../usr/lib/debug/Half.lib
else:unix:!symbian: PRE_TARGETDEPS += $$PWD/../../../../../usr/lib/libHalf.a
