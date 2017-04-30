#-------------------------------------------------
#
# Project created by QtCreator 2016-11-06T12:15:54
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

TARGET = ProjectionApp
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    calculation.cpp \
    iodata.cpp

HEADERS  += mainwindow.h \
    calculation.h \
    iodata.h

FORMS    += mainwindow.ui

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/lib/ann_1.1.2/lib/release/ -lANN
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/lib/ann_1.1.2/lib/debug/ -lANN
else:unix: LIBS += -L$$PWD/lib/ann_1.1.2/lib/ -lANN

INCLUDEPATH += $$PWD/lib/ann_1.1.2/include
DEPENDPATH += $$PWD/lib/ann_1.1.2/include

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/lib/ann_1.1.2/lib/release/libANN.a
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/lib/ann_1.1.2/lib/debug/libANN.a
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/lib/ann_1.1.2/lib/release/ANN.lib
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/lib/ann_1.1.2/lib/debug/ANN.lib
else:unix: PRE_TARGETDEPS += $$PWD/lib/ann_1.1.2/lib/libANN.a

INCLUDEPATH += $$PWD/lib/Eigen
