#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include "armadillo"
#include "mainwindow.h"
#include <QtGui>
#include <QApplication>

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    MainWindow w;
    w.show();

    return app.exec();
}
