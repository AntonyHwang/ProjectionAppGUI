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
    string line;
    ifstream camfile ("cameraData/cameras_v2.txt");
    string RMatrix;
    int cameraCount = 0;
    Vector3d r1;
    Vector3d r2;
    if (camfile.is_open()) {
        while (getline(camfile, line)) {
            if (lineCount == 3) {
                //storing focal length
                cameraCount++;
                //cout << "\n" << "Storing camera " << cameraCount << "\n";
                camera[cameraCount - 1].focalLen = stringToDouble(line);
                camera[cameraCount - 1].K(0,0) = camera[cameraCount - 1].focalLen;
                camera[cameraCount - 1].K(1,1) = camera[cameraCount - 1].focalLen;
                //cout << "K: \n" << camera[cameraCount - 1].K << "\n";
            }
            else if (lineCount == 4) {
                //storing Translation T
                double x, y, z;
                camfile >> x >> y >> z;
                camera[cameraCount - 1].T << x, y, z;
                //cout << "T: \n" << camera[cameraCount - 1].T << "\n";
            }
            else if (lineCount == 5) {
                //storing Camera position C
                double x, y, z;
                camfile >> x >> y >> z;
                camera[cameraCount - 1].C << x, y, z;
                //cout << "C: \n" << camera[cameraCount - 1].C << "\n";
            }
            else if (lineCount == 6) {
                //storing Axis angle format of R
                double x, y, z;
                camfile >> x >> y >> z;
                camera[cameraCount - 1].aR << x, y, z;
                //cout << "aR: \n" << camera[cameraCount - 1].aR << "\n";
            }
            else if (lineCount == 7) {
                //storign Quaternion format of R
                double x, y, z, t;
                camfile >> x >> y >> z >> t;
                camera[cameraCount - 1].qR << x, y, z, t;
                //cout << "qR: \n" << camera[cameraCount - 1].qR << "\n";
            }
            else if (lineCount == 8) {
                //storing Matrix format of R
                double x1, x2, x3;
                camfile >> x1 >> x2 >> x3;
                r1 << x1, x2, x3;
            }
            else if (lineCount == 9) {
                double y1, y2, y3;
                camfile >> y1 >> y2 >> y3;
                r2 << y1, y2, y3;
                //r3 = join_cols(r1, r2);
            }
            else if (lineCount == 10) {
                double z1, z2, z3;
                camfile >> z1 >> z2 >> z3;
                camera[cameraCount - 1].R << r1(0,0), r1(1,0), r1(2,0),
                                             r2(0,0), r2(1,0), r2(2,0),
                                             z1, z2, z3;
                //cout << "R: \n" << camera[cameraCount - 1].R << "\n";
            }
            else if (lineCount == 12) {
                //storing Normalized radial distortion
                stringstream convert(line);
                convert >> camera[cameraCount - 1].rD;
                //cout << "rD: \n" << camera[cameraCount - 1].rD << "\n";
            }
            if (lineCount > 14) {
                lineCount = 1;
            }
            lineCount++;
            if (camfile.eof()) {
                camfile.close();
            }
        }
    }
    else cout << "Unable to open file";
    return cameraCount;
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
    ofstream tempFile("temp.txt");
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
    ifstream RGBfile ("temp.txt");
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
                    //cout << r << " " << g << " " << b << "\n";
                }
                else if (lineCount == 7 || lineCount == 9) {
                    calculate2DPoint(camera, stringToDouble(firstWord) - 1, point_3D, point_RGB);
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
                    //cout << pointNum << " 3D points calculation complete\n";
                }
                //cout << "\n";
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
    QString pointData = "query.pts";
    QFile file(pointData);
    if (file.open(QIODevice::ReadWrite)) {
        QTextStream stream(&file);
        for (int x = 0; x <= maxX; x++) {
            for (int y = 0; y <= maxY; y++) {
                stream << x << "\t" << y << endl;
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

QImage showRGBImg(int camIndex) {
    double imageCentreX = 414.0;
    double imageCentreY = 646.0;
    QImage image = QImage(imageCentreX * 2, imageCentreY * 2, QImage::Format_RGB32);
    image.fill(QColor(Qt::black).rgb());
    QString filePath = QString(("visualize/" + getFileName("Img", camIndex)).c_str());
    QImage oImage = QImage(filePath);
    QFile inputFile("dv.txt");
    if (inputFile.open(QIODevice::ReadOnly))
    {
       QTextStream in(&inputFile);
       while (!in.atEnd())
       {
           for (int ix = 0; ix <= imageCentreX * 2; ix++) {
               for (int iy = 0; iy <= imageCentreY * 2; iy++) {
                   QString x, y;
                   int px, py;
                   QString line = in.readLine();
                   //qDebug() << line << "\n";
                   in >> x >> y;
                   //qDebug() << x.toDouble() << "\t" << y.toDouble() << "\n";
                   px = ix - x.toDouble();
                   py = iy - y.toDouble();
                   QColor c = QColor::fromRgb (oImage.pixel(px,py) );
                   //qDebug() << ix << "\t" << iy << ": " << px << "\t" << py << "\n";
                   image.setPixel(ix, iy, c.rgb());
               }
           }
       }
       inputFile.close();
    }
    return image;
}
