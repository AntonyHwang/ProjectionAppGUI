#include "iodata.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <QDebug>
#include <QFile>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QPainter>
#include <random>

double stringToDouble(string s) {
    double d = atof(s.c_str());
    return d;
}

bool checkFileExist(string fileName) {
    ifstream infile(fileName);
    return infile.good();
}

string getFileName(string fileType, int index) {
    string fileName;
    if (fileType == "PMatrix") {
        stringstream convert;
        convert << index;
        fileName = convert.str();
        fileName = fileName + ".txt";
        while (fileName.length() < 12) {
            fileName = '0' + fileName;
        }
        return fileName;
    }
    else if (fileType == "Patch" || fileType == "PLY") {
        //option-0001.patch
        stringstream convert;
        convert << index;
        fileName = convert.str();
        if (fileType == "Patch") {
            fileName = fileName + ".patch";
            while (fileName.length() < 10) {
                fileName = '0' + fileName;
            }
        }
        else {
            fileName = fileName + ".ply";
            while (fileName.length() < 8) {
                fileName = '0' + fileName;
            }
        }
        return "option-" + fileName;
    }
    else if (fileType == "Img") {
        stringstream convert;
        convert << index;
        fileName = convert.str();
        fileName = fileName + ".jpg";
        while (fileName.length() < 12) {
            fileName = '0' + fileName;
        }
        return fileName;
    }
    else {
        cout << "cannot find name for this filetype";
        return "file not exist";
    }
}

int readCamFile(cameraInfo camera[]) {
    int lineCount = 1;
    int numCameras = 0;
    int cameraIdx = 0;
    Vector3d r1;
    Vector3d r2;

    QFile camFile("cameraData/cameras_v2.txt");
    if (camFile.open(QIODevice::ReadOnly))
    {
       QTextStream in(&camFile);
       while (!in.atEnd())
       {
           QString line = in.readLine();
           QStringList numbers = line.split( " " );
           //qDebug() << "lineNum: " << lineCount << "\n";
           if (lineCount == 21) {
               //storing focal length
               //cout << "\n" << "Storing camera " << cameraIdx << "\n";
               camera[cameraIdx].focalLen = line.toDouble();
               camera[cameraIdx].K(0,0) = camera[cameraIdx].focalLen;
               camera[cameraIdx].K(1,1) = camera[cameraIdx].focalLen;
               //cout << "K: \n" << camera[cameraIdx].K << "\n";
               //qDebug() << line.toDouble() << "\n";
           }
           else if (lineCount == 22) {
               //storing image center
               camera[cameraIdx].imgCentreX = numbers.value(0).toDouble();
               camera[cameraIdx].imgCentreY = numbers.value(1).toDouble();
               camera[cameraIdx].K(0,2) = camera[cameraIdx].imgCentreX;
               camera[cameraIdx].K(1,2) = camera[cameraIdx].imgCentreY;
               camera[cameraIdx].K(2,2) = 1;
               //qDebug() << camera[cameraCount - 1].imgCentreX << " " << camera[cameraCount - 1].imgCentreY << "\n";
           }
           else if (lineCount == 23) {
               //storing Translation T
               camera[cameraIdx].T << numbers.value(0).toDouble(), numbers.value(1).toDouble(), numbers.value(2).toDouble();
               //cout << "T: \n" << camera[cameraIdx].T << "\n";
               //qDebug() << numbers.value(0).toDouble() << numbers.value(1).toDouble() << numbers.value(2).toDouble() << "\n";
           }
           else if (lineCount == 24) {
               //storing Camera position C
               camera[cameraIdx].C << numbers.value(0).toDouble(), numbers.value(1).toDouble(), numbers.value(2).toDouble();
               //cout << "C: \n" << camera[cameraIdx].C << "\n";
               //qDebug() << numbers.value(0).toDouble() << numbers.value(1).toDouble() << numbers.value(2).toDouble() << "\n";
           }
           else if (lineCount == 25) {
               //storing Axis angle format of R
               camera[cameraIdx].aR << numbers.value(0).toDouble(), numbers.value(1).toDouble(), numbers.value(2).toDouble();
               //cout << "aR: \n" << camera[cameraIdx].aR << "\n";
               //qDebug() << numbers.value(0).toDouble() << numbers.value(1).toDouble() << numbers.value(2).toDouble() << "\n";
           }
           else if (lineCount == 26) {
               //storign Quaternion format of R
               camera[cameraIdx].qR << numbers.value(0).toDouble(), numbers.value(1).toDouble(), numbers.value(2).toDouble(), numbers.value(3).toDouble();
               //cout << "qR: \n" << camera[cameraIdx].qR << "\n";
               //qDebug() << numbers.value(0).toDouble() << numbers.value(1).toDouble() << numbers.value(2).toDouble() << numbers.value(3).toDouble() << "\n";
           }
           else if (lineCount == 27) {
               //storing Matrix format of R
               r1 << numbers.value(0).toDouble(), numbers.value(1).toDouble(), numbers.value(2).toDouble();
               //qDebug() << numbers.value(0).toDouble() << numbers.value(1).toDouble() << numbers.value(2).toDouble() << "\n";
           }
           else if (lineCount == 28) {
               r2 << numbers.value(0).toDouble(), numbers.value(1).toDouble(), numbers.value(2).toDouble();
               //qDebug() << numbers.value(0).toDouble() << numbers.value(1).toDouble() << numbers.value(2).toDouble() << "\n";
           }
           else if (lineCount == 29) {
               camera[cameraIdx].R << r1(0,0), r1(1,0), r1(2,0),
                                      r2(0,0), r2(1,0), r2(2,0),
                                      numbers.value(0).toDouble(), numbers.value(1).toDouble(), numbers.value(2).toDouble();
               //cout << "R: \n" << camera[cameraIdx].R << "\n";
               //qDebug() << numbers.value(0).toDouble() << numbers.value(1).toDouble() << numbers.value(2).toDouble() << "\n";
           }
           else if (lineCount == 30) {
               //storing Normalized radial distortion
               camera[cameraIdx].rD = line.toDouble();;
               //cout << "rD: \n" << camera[cameraIdx].rD << "\n";
               //qDebug() << line.toDouble() << "\n";
           }
           else if (lineCount == 17) {
               numCameras = line.toInt();
               //qDebug() << line.toInt() << "\n";
           }

           lineCount++;

           if (lineCount >= 33) {
               lineCount = 19;
               cameraIdx++;
           }
       }
       camFile.close();
    }
    qDebug() << numCameras << "\n";
    return numCameras;
}

void readPFiles(cameraInfo camera[], int cameraCount) {
    for (int currentCamera = 0; currentCamera < cameraCount; currentCamera++) {
        string line;
        ifstream pfile ("cameraData/camera_matrix/" + getFileName("PMatrix", currentCamera));
        MatrixXd m(3,4);
        if (pfile.is_open()) {
            while (pfile >> line) {
                    double x1, x2, x3, x4,
                           y1, y2, y3, y4,
                           z1, z2, z3, z4;

                    pfile >> x1 >> x2 >> x3 >> x4
                          >> y1 >> y2 >> y3 >> y4
                          >> z1 >> z2 >> z3 >> z4;
                    camera[currentCamera].P = m;
                    camera[currentCamera].P << x1, x2, x3, x4,
                                               y1, y2, y3, y4,
                                               z1, z2, z3, z4;
            }
            pfile.close();
            //cout << "P: \n" << camera[currentCamera].P << "\n";
        }
        else cout << "Unable to open file";
    }
}

void processPLYFile() {
    string line;
    string x, y, z, nx, ny, nz, r, g, b;
    ofstream tempFile("output/temp.txt");
    int lineCount = 0;
    int currentFile = 0;
    string fileName = "cameraData/patch/" + getFileName("PLY", currentFile);
    while (checkFileExist(fileName) == 1) {
        fileName = "cameraData/patch/" + getFileName("PLY", currentFile);
        ifstream plyfile (fileName);
        if (plyfile.is_open()) {
            while (getline(plyfile, line)) {
                if (lineCount >= 12) {
                    plyfile >> x >> y >> z >> nx >> ny >> nz >> r >> g >> b;
                    tempFile << r << " " << g << " " << b << "\n";
                }
                lineCount++;
            }
            if (plyfile.eof()) {
                plyfile.close();
                currentFile++;
            }
        }
    }
    tempFile.close();
}

void store2DPoint (cameraInfo camera[], int camIndex, double x, double y, RGB point_RGB, Vector4d point_3D) {
    int emptyIndex = 0;
    for (int i = 0; i < MAX_POINTS; i++) {
        if (camera[camIndex].image2DPoint[i].x == 0 && camera[camIndex].image2DPoint[i].y == 0) {
            emptyIndex = i;
            break;
        }
    }
    camera[camIndex].p3DPoint[emptyIndex].x = point_3D(0, 0);
    camera[camIndex].p3DPoint[emptyIndex].y = point_3D(1, 0);
    camera[camIndex].p3DPoint[emptyIndex].z = point_3D(2, 0);
    camera[camIndex].image2DPoint[emptyIndex].x = x;
    camera[camIndex].image2DPoint[emptyIndex].y = y;
    camera[camIndex].image2DPoint[emptyIndex].pointRGB.r = point_RGB.r;
    camera[camIndex].image2DPoint[emptyIndex].pointRGB.g = point_RGB.g;
    camera[camIndex].image2DPoint[emptyIndex].pointRGB.b = point_RGB.b;
    //cout << camera[camIndex - 1].image2DPoint[emptyIndex].pointRGB.r << " " << camera[camIndex - 1].image2DPoint[emptyIndex].pointRGB.g << " " << camera[camIndex - 1].image2DPoint[emptyIndex].pointRGB.b << "\n";
}

void calculate2DPoint(cameraInfo camera[], int index, Vector4d point_3D, RGB point_RGB) {
    //calculate points
    Vector3d point_2D;
    double x;
    double y;
    point_2D = camera[index].P * point_3D;
    x = point_2D(0, 0) / (point_2D(2, 0));
    y = point_2D(1, 0) / (point_2D(2, 0));
    //cout << "X: " << x << "  Y: " << y << "\n";
    //writeToFile(index, x, y, point_RGB);
    store2DPoint(camera, index, x, y, point_RGB, point_3D);
}

void readPatchFile(cameraInfo camera[]) {
    string line;
    string lineRGB;
    string firstWord;
    int pointNum = 0;
    int currentFile = 0;
    string fileName = "cameraData/patch/" + getFileName("Patch", currentFile);
    Vector4d point_3D;
    RGB point_RGB;
    int lineCount = 0;
    ifstream RGBfile ("output/temp.txt");
    while (checkFileExist(fileName) == 1) {
        fileName = "cameraData/patch/" + getFileName("Patch", currentFile);
        ifstream patchfile (fileName);
        if (patchfile.is_open()) {
            while (getline(patchfile, line)) {
                patchfile >> firstWord;
                if (firstWord == "PATCHS") {
                    lineCount = 2;
                }
                else if (lineCount == 3) {
                    double x = stringToDouble(firstWord);
                    double y, z, t;
                    patchfile >> y >> z >> t;
                    point_3D << x, y, z, t;
                }
                else if (lineCount == 6) {
                    getline(RGBfile, lineRGB);
                    double r, g, b;
                    RGBfile >> r >> g >> b;
                    point_RGB.r = r;
                    point_RGB.g = g;
                    point_RGB.b = b;
                }
                else if (lineCount == 7 || lineCount == 9) {
                    calculate2DPoint(camera, stringToDouble(firstWord), point_3D, point_RGB);
                    getline(patchfile, line);
                    stringstream stream(line);
                    while(1) {
                        int n;
                        stream >> n;
                        if(!stream)
                            break;
                        calculate2DPoint(camera, n, point_3D, point_RGB);
                    }
                    pointNum++;
                }
                else if (lineCount == 8 && stringToDouble(firstWord) == 0) {
                    lineCount++;
                }
                lineCount++;
                if (patchfile.eof()) {
                    patchfile.close();
                    currentFile++;
                }
            }
        }
    }
    RGBfile.close();
    cout << "\ncalculation completed\n";
}

void writeQueryToFile(int maxX, int maxY) {
    QString pointData = "output/query.pts";
    QFile file(pointData);
    if (file.open(QIODevice::ReadWrite | QIODevice::Truncate)) {
        QTextStream stream(&file);
        for (int x = 0; x < maxX; x++) {
            for (int y = 0; y < maxY; y++) {
                stream << x << "\t" << y << endl;
            }
        }
    }
}

void writeQueryToFileImproved(int maxX, int maxY) {
    QString pointData = "output/queryimp.pts";
    QFile file(pointData);
    int xOdd = 0;
    int yOdd = 0;
    if (file.open(QIODevice::ReadWrite | QIODevice::Truncate)) {
        QTextStream stream(&file);
        for (int x = 0; x < maxX; x++) {
            if (x % 2) {
                xOdd = 1;
            }
            else {
                xOdd = 0;
            }
            for (int y = 0; y < maxY; y++) {
                if (y % 2) {
                    yOdd = 1;
                }
                else {
                    yOdd = 0;
                }
                if (xOdd == yOdd ) {
                    stream << x << "\t" << y << endl;
                }
            }
        }
    }
}

void writeToFile(int mode, QString fileName, double x, double y) {
    if (mode == 1) {
        QString pointData = fileName;
        QFile file(pointData);
        if (file.open(QIODevice::WriteOnly | QIODevice::Append)) {
            QTextStream stream(&file);
            stream << x << " " << y << endl;
        }
        file.close();
    }
    else if (mode == 0) {
        QString pointData = fileName;
        QFile file(pointData);
        if (file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
            QTextStream stream(&file);
            stream << x << " " << y << endl;
        }
        file.close();
    }
}

QImage showRGBImg(int camIndex, cameraInfo camera[]) {
    QImage image = QImage(camera[0].imgCentreX, camera[0].imgCentreY, QImage::Format_RGB32);
    image.fill(QColor(Qt::white).rgb());
    QString filePath = QString(("visualize/" + getFileName("Img", camIndex)).c_str());
    QImage oImage = QImage(filePath);
    QFile inputFile("output/dv.txt");
    if (inputFile.open(QIODevice::ReadOnly))
    {
       QTextStream in(&inputFile);
       while (!in.atEnd())
       {
           for (int ix = 0; ix < camera[0].imgCentreX * 2; ix++) {
               for (int iy = 0; iy < camera[0].imgCentreY * 2; iy++) {
                   QString x, y;
                   int px, py;
                   QString line = in.readLine();

                   in >> x >> y;

                   px = ix - x.toDouble();
                   py = iy - y.toDouble();
                   if (px < 0) {
                       px = 0;
                   }
                   if (px >= camera[0].imgCentreX * 2) {
                       px = camera[0].imgCentreX * 2 - 1;
                   }
                   if (py < 0) {
                       py = 0;
                   }
                   if (py >= camera[0].imgCentreY * 2) {
                       py = camera[0].imgCentreY * 2 - 1;
                   }
                   QColor c = QColor::fromRgb (oImage.pixel(px,py) );
                   if (x == "10000" && y == "10000") {
                       image.setPixel(ix / 2, iy / 2, QColor(Qt::white).rgb());
                   }
                   else {
                       image.setPixel(ix / 2, iy / 2, c.rgb());
                   }
               }
           }
       }
       inputFile.close();
    }
    return image;
}

void save_interpolated_data(QImage image, int frame) {
    int x = image.width();
    int y = image.height();
    QFile saveFile("output/data/" + QString::number(frame) + ".txt");
    saveFile.open(QIODevice::WriteOnly | QIODevice::Truncate);
    QTextStream stream(&saveFile);
    for (int ix = 0; ix < x; ix++) {
        for (int iy = 0; iy < y; iy++) {
            QColor colour = QColor::fromRgb (image.pixel(ix, iy));
            stream << ix << "\t" << iy << "\t" << colour.red() << "\t" << colour.green() << "\t" << colour.blue() << endl;
        }
    }
}

QImage show_displacement(QImage image, int frame) {
    QFile start_end_file("output/drawing/frame_" + QString::number(frame) + ".txt");
    start_end_file.open(QIODevice::ReadOnly);
    QTextStream start_end(&start_end_file);

    QPainter painter;
    painter.begin(&image);

    QString from_x, from_y, to_x, to_y, r, g, b;
    QColor color;

    while (!start_end.atEnd()) {
        start_end >> from_x >> from_y >> to_x >> to_y >> r >> g >> b;
        painter.drawLine(from_x.toInt(),from_y.toInt(),to_x.toInt(),to_y.toInt());
        painter.drawEllipse(to_x.toInt(),to_y.toInt(), 2, 2);
    }
    painter.end();
    start_end_file.close();
    return image;
}

//improved runtime by interpolating the RGB values
QImage showRGBImgImproved(QString method, int camIndex, cameraInfo camera[], int frame) {
    int xOdd = 0;
    int yOdd = 0;
    QString filePath;
    QImage image = QImage(camera[0].imgCentreX, camera[0].imgCentreY, QImage::Format_RGB32);
    image.fill(QColor(Qt::white).rgb());
    QImage full_image = QImage(camera[0].imgCentreX * 2, camera[0].imgCentreY * 2, QImage::Format_RGB32);
    full_image.fill(QColor(Qt::white).rgb());

    QFile start_end_file("output/drawing/frame_" + QString::number(frame) + ".txt");

    if (method == QString::fromStdString("First Last Frame")) {
        //inerpolate by first and last frame
        /**/
        filePath = QString(("visualize/" + getFileName("Img", camIndex)).c_str());
        start_end_file.open(QIODevice::WriteOnly | QIODevice::Truncate);
        /**/
    }
    else {
        //inerpolate by previous frame
        /**/
        start_end_file.open(QIODevice::WriteOnly | QIODevice::Truncate);

        if (frame == 0 || frame == 1 || frame == 9 || frame == 10) {
            filePath = QString(("visualize/" + getFileName("Img", camIndex)).c_str());
        }
        else {
            if (frame <= 5) {
                filePath = QString("output/visualize/frame_" + QString::number(frame - 1) + ".jpg");
            }
            else {
                filePath = QString("output/visualize/frame_" + QString::number(frame + 1) + ".jpg");
            }
        }
        /**/
    }
    QTextStream start_end(&start_end_file);
    QImage oImage = QImage(filePath);
    QFile inputFile("output/dv.txt");
    if (inputFile.open(QIODevice::ReadOnly))
    {
       QTextStream in(&inputFile);
       while (!in.atEnd())
       {
           for (int ix = 0; ix < camera[0].imgCentreX * 2; ix++) {
               if (ix % 2) {
                   xOdd = 1;
               }
               else {
                   xOdd = 0;
               }
               for (int iy = 0; iy < camera[0].imgCentreY * 2; iy++) {
                   if (iy % 2) {
                       yOdd = 1;
                   }
                   else {
                       yOdd = 0;
                   }
                   if (xOdd == yOdd) {
                       QString x, y, r, g, b;
                       int px, py;
                       QString line = in.readLine();
                       in >> x >> y >> r >> g >> b;
                       if ((x == "10000" && y == "10000") || (x == "nan" && y == "nan")) {
                           image.setPixel(ix / 2, iy / 2, QColor(Qt::white).rgb());
                           full_image.setPixel(ix, iy, QColor(Qt::white).rgb());
                       }
                       else {
                           px = ix - x.toDouble();
                           py = iy - y.toDouble();
                           if (px < 0) {
                               px = 0;
                           }
                           if (px >= camera[0].imgCentreX * 2) {
                               px = camera[0].imgCentreX * 2 - 1;
                           }
                           if (py < 0) {
                               py = 0;
                           }
                           if (py >= camera[0].imgCentreY * 2) {
                               py = camera[0].imgCentreY * 2 - 1;
                           }

                           QColor c;
                           c = QColor::fromRgb (oImage.pixel(px, py));

                           /**/
                           //debugging by visual
                           if (ix % 30 == 0 && iy % 30 == 0) {
                               start_end << px << " " << py << " " << ix << " " << iy << " " << r << " " << g << " " << b << endl;
                           }
                           image.setPixel(ix / 2, iy / 2, c.rgb());
                           full_image.setPixel(ix, iy, c.rgb());
                       }
                   }
               }
           }
       }
       inputFile.close();
       start_end_file.close();
    }
    for (int ix = 0; ix < camera[0].imgCentreX * 2; ix++) {
        if (ix % 2) {
            xOdd = 1;
        }
        else {
            xOdd = 0;
        }
        for (int iy = 0; iy < camera[0].imgCentreY * 2; iy++) {
            if (iy % 2) {
                yOdd = 1;
            }
            else {
                yOdd = 0;
            }
            if (xOdd != yOdd) {
                int nPixel = 4;
                QColor avgRGB;
                QColor n1;
                QColor n2;
                QColor n3;
                QColor n4;
                if (ix - 1 < 0) {
                    nPixel--;
                    n1.setRed(0);
                    n1.setGreen(0);
                    n1.setBlue(0);
                }
                else {
                    n1 = QColor::fromRgb (full_image.pixel(ix - 1, iy)); //left
                }
                if (ix + 1 >= camera[0].imgCentreX * 2) {
                    nPixel--;
                    n3.setRed(0);
                    n3.setGreen(0);
                    n3.setBlue(0);
                }
                else {
                    n3 = QColor::fromRgb (full_image.pixel(ix + 1,iy)); //right
                }
                if (iy - 1 < 0) {
                    nPixel--;
                    n2.setRed(0);
                    n2.setGreen(0);
                    n2.setBlue(0);
                }
                else {
                    n2 = QColor::fromRgb (full_image.pixel(ix, iy - 1)); //bottom
                }
                if (iy + 1 >= camera[0].imgCentreY * 2) {
                    nPixel--;
                    n4.setRed(0);
                    n4.setGreen(0);
                    n4.setBlue(0);
                }
                else {
                    n4 = QColor::fromRgb (full_image.pixel(ix, iy + 1)); //top
                }
                avgRGB.setAlpha(1);
                avgRGB.setRed((n1.red() + n2.red() + n3.red() + n4.red()) / nPixel);
                avgRGB.setGreen((n1.green() + n2.green() + n3.green() + n4.green()) / nPixel);
                avgRGB.setBlue((n1.blue() + n2.blue() + n3.blue() + n4.blue()) / nPixel);
                image.setPixel(ix / 2, iy / 2, avgRGB.rgb());
                full_image.setPixel(ix, iy, avgRGB.rgb());
            }
        }
    }
    save_interpolated_data(full_image, frame);
    full_image.save("output/visualize/frame_" + QString::number(frame) + ".jpg");
    QImage debug_image = QImage(camera[0].imgCentreX * 2, camera[0].imgCentreY * 2, QImage::Format_RGB32);
    debug_image = show_displacement(full_image, frame);
    debug_image.save("output/visualize/debug/frame_" + QString::number(frame) + ".jpg");

    return image;
}
