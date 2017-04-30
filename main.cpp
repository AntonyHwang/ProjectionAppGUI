#include "mainwindow.h"
#include <QtGui>
#include <QApplication>
#include <string>
#include <stdio.h>

int main(int argc, char *argv[]) {

    remove("output/data/0.txt");
    remove("output/data/1.txt");
    remove("output/data/2.txt");
    remove("output/data/3.txt");
    remove("output/data/4.txt");
    remove("output/data/5.txt");
    remove("output/data/6.txt");
    remove("output/data/7.txt");
    remove("output/data/8.txt");
    remove("output/data/9.txt");
    remove("output/data/10.txt");

    QApplication app(argc, argv);
    MainWindow w;
    w.show();

    return app.exec();
}
