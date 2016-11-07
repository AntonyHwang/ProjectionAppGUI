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
        qcustomplot.cpp

HEADERS  += mainwindow.h \
            qcustomplot.h

FORMS    += mainwindow.ui

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../usr/local/Cellar/armadillo/7.500.0/lib/release/ -larmadillo.7.50.0
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../usr/local/Cellar/armadillo/7.500.0/lib/debug/ -larmadillo.7.50.0
else:unix: LIBS += -L$$PWD/../../../../usr/local/Cellar/armadillo/7.500.0/lib/ -larmadillo.7.50.0

INCLUDEPATH += $$PWD/../../../../usr/local/Cellar/armadillo/7.500.0/include
DEPENDPATH += $$PWD/../../../../usr/local/Cellar/armadillo/7.500.0/include

DISTFILES += \
    cameraData/00000000.txt \
    cameraData/00000001.txt \
    cameraData/00000002.txt \
    cameraData/00000003.txt \
    cameraData/00000004.txt \
    cameraData/00000005.txt \
    cameraData/00000006.txt \
    cameraData/00000007.txt \
    cameraData/00000008.txt \
    cameraData/00000009.txt \
    cameraData/00000010.txt \
    cameraData/00000011.txt \
    cameraData/00000012.txt \
    cameraData/00000013.txt \
    cameraData/00000014.txt \
    cameraData/00000015.txt \
    cameraData/00000016.txt \
    cameraData/00000017.txt \
    cameraData/00000018.txt \
    cameraData/00000019.txt \
    cameraData/00000020.txt \
    cameraData/00000021.txt \
    cameraData/00000022.txt \
    cameraData/00000023.txt \
    cameraData/00000024.txt \
    cameraData/00000025.txt \
    cameraData/00000026.txt \
    cameraData/00000027.txt \
    cameraData/00000028.txt \
    cameraData/00000029.txt \
    cameraData/00000030.txt \
    cameraData/00000031.txt \
    cameraData/00000032.txt \
    cameraData/00000033.txt \
    cameraData/00000034.txt \
    cameraData/00000035.txt \
    cameraData/00000036.txt \
    cameraData/00000037.txt \
    cameraData/00000038.txt \
    cameraData/00000039.txt \
    cameraData/00000040.txt \
    cameraData/00000041.txt \
    cameraData/00000042.txt \
    cameraData/00000043.txt \
    cameraData/00000044.txt \
    cameraData/00000045.txt \
    cameraData/00000046.txt \
    cameraData/00000047.txt \
    cameraData/00000048.txt \
    cameraData/00000049.txt \
    cameraData/00000050.txt \
    cameraData/00000051.txt \
    cameraData/00000052.txt \
    cameraData/00000053.txt \
    cameraData/00000054.txt \
    cameraData/00000055.txt \
    cameraData/00000056.txt \
    cameraData/00000057.txt \
    cameraData/00000058.txt \
    cameraData/00000059.txt \
    cameraData/00000060.txt \
    cameraData/00000061.txt \
    cameraData/00000062.txt \
    cameraData/00000063.txt \
    cameraData/00000064.txt \
    cameraData/00000065.txt \
    cameraData/00000066.txt \
    cameraData/00000067.txt \
    cameraData/00000068.txt \
    cameraData/00000069.txt \
    cameraData/00000070.txt \
    cameraData/00000071.txt \
    cameraData/00000072.txt \
    cameraData/00000073.txt \
    cameraData/00000074.txt \
    cameraData/00000075.txt \
    cameraData/00000076.txt \
    cameraData/00000077.txt \
    cameraData/00000078.txt \
    cameraData/00000079.txt \
    cameraData/00000080.txt \
    cameraData/00000081.txt \
    cameraData/00000082.txt \
    cameraData/00000083.txt \
    cameraData/00000084.txt \
    cameraData/00000085.txt \
    cameraData/00000086.txt \
    cameraData/00000087.txt \
    cameraData/00000088.txt \
    cameraData/00000089.txt \
    cameraData/00000090.txt \
    cameraData/00000091.txt \
    cameraData/00000092.txt \
    cameraData/00000093.txt \
    cameraData/00000094.txt \
    cameraData/00000095.txt \
    cameraData/cameras_v2.txt \
    cameraData/option-0000.patch \
    cameraData/option-0001.patch
