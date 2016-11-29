#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "calculation.h"

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include "armadillo"

#include <QDebug>
#include <QFile>
#include <QMessageBox>
#include <QTextStream>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QPointF>
#include <QVector>
#include <QSlider>
#include <QSpinBox>

using namespace arma;
using namespace std;

const int MAX_CAM_NUM = 200;
const int MAX_POINTS = 150000;

double imageCentreX = 540.0;
double imageCentreY = 960.0;

int cameraNum;
int cameraCount = 0;
int lineCount = 1;

struct RGB {
    double r;
    double g;
    double b;
};

struct point2D {
    double x = 0;
    double y = 0;
    RGB pointRGB;
};

struct point3D {
    double x;
    double y;
    double z;
};

struct cameraInfo {
    double focalLen;
    mat K;//IntrinsicMatrix 3x3
    mat T;//Translation 3x1
    mat C;//CameraPosition 3x1
    mat aR;//AxisAngleR 3x1
    mat qR;//QuaternionR 4x1
    mat R;//3x3R
    double rD;//RadialDistortion
    mat P;//CameraMatrix
    point3D p3DPoint [MAX_POINTS];
    point2D image2DPoint [MAX_POINTS];//2D points
};

cameraInfo camera [MAX_CAM_NUM];
cameraInfo iCamera;
mat i3DPoint[MAX_POINTS];

void initialiseK(){
    int i =0;
    for (i = 0; i < MAX_CAM_NUM; i++) {
        camera[i].K << 0 << 0 << imageCentreX << endr
                    << 0 << 0 << imageCentreY << endr
                    << 0 << 0 << 1;
    }
}

double stringToDouble(string s) {
    double d = atof(s.c_str());
    return d;
}

void readFile() {
    std::string line;
    ifstream camfile ("cameraData/cameras_v2.txt");
    string RMatrix;
    mat r1;
    mat r2;
    mat r3;
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
                camera[cameraCount - 1].T << x << endr << y << endr << z;
                //cout << "T: \n" << camera[cameraCount - 1].T << "\n";
            }
            else if (lineCount == 5) {
                //storing Camera position C
                double x1, y1, z1;
                camfile >> x1 >> y1 >> z1;
                camera[cameraCount - 1].C << x1 << endr << y1 << endr << z1;
                //cout << "C: \n" << camera[cameraCount - 1].C << "\n";
            }
            else if (lineCount == 6) {
                //storing Axis angle format of R
                double x, y, z;
                camfile >> x >> y >> z;
                camera[cameraCount - 1].aR << x << endr << y << endr << z;
                //cout << "aR: \n" << camera[cameraCount - 1].aR << "\n";
            }
            else if (lineCount == 7) {
                //storign Quaternion format of R
                double x, y, z, t;
                camfile >> x >> y >> z >> t;
                camera[cameraCount - 1].qR << x << endr << y << endr << z << endr << t;
                //cout << "qR: \n" << camera[cameraCount - 1].qR << "\n";
            }
            else if (lineCount == 8) {
                //storing Matrix format of R
                double x1, x2, x3;
                camfile >> x1 >> x2 >> x3;
                r1 << x1 << x2 << x3;
            }
            else if (lineCount == 9) {
                double y1, y2, y3;
                camfile >> y1 >> y2 >> y3;
                r2 << y1 << y2 << y3;
                r3 = join_cols(r1, r2);
            }
            else if (lineCount == 10) {
                double z1, z2, z3;
                camfile >> z1 >> z2 >> z3;
                r2 << z1 << z2 << z3;
                camera[cameraCount - 1].R = join_cols(r3, r2);
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
    else {
        cout << "cannot find name for this filetype";
        return "file not exist";
    }
}

void readPFiles() {
    for (int currentCamera = 0; currentCamera < cameraCount; currentCamera++) {
        std::string line;
        ifstream pfile ("cameraData/camera_matrix/" + getFileName("PMatrix", currentCamera));
        if (pfile.is_open()) {
            while (pfile >> line) {
                    double x1, x2, x3, x4,
                           y1, y2, y3, y4,
                           z1, z2, z3, z4;

                    pfile >> x1 >> x2 >> x3 >> x4
                          >> y1 >> y2 >> y3 >> y4
                          >> z1 >> z2 >> z3 >> z4;

                    camera[currentCamera].P << x1 << x2 << x3 << x4 << endr
                                            << y1 << y2 << y3 << y4 << endr
                                            << z1 << z2 << z3 << z4;
            }
            pfile.close();
            //cout << "P: \n" << camera[currentCamera].P << "\n";
        }
        else cout << "Unable to open file";
    }
}

bool is_file_exist(string fileName)
{
    std::ifstream infile(fileName);
    return infile.good();
}

void writeToFile (int camIndex, double x, double y, RGB RGBVal) {
    std::ofstream outputFile;
    string index;
    ostringstream convert;
    convert << camIndex;
    index = convert.str();
    if (is_file_exist("output/" + index + ".txt"))
    {
        //open and write to file
        outputFile.open("output/" + index + ".txt", std::ios_base::app);
    }
    else {
        std::ofstream outputFile("output/" + index + ".txt");
    }
    outputFile << x << " " << y << " " << RGBVal.r << " " << RGBVal.g << " " << RGBVal.b << "\n";
    outputFile.close();
}

void store2DPoint (int camIndex, double x, double y, RGB point_RGB, mat point_3D){
    int emptyIndex = 0;
    for (int i = 0; i < MAX_POINTS; i++) {
        if (camera[camIndex - 1].image2DPoint[i].x == 0 && camera[camIndex - 1].image2DPoint[i].y == 0) {
            emptyIndex = i;
            i = MAX_POINTS;
        }
    }
    camera[camIndex - 1].p3DPoint[emptyIndex].x = point_3D(0, 0);
    camera[camIndex - 1].p3DPoint[emptyIndex].y = point_3D(1, 0);
    camera[camIndex - 1].p3DPoint[emptyIndex].z = point_3D(2, 0);
    camera[camIndex - 1].image2DPoint[emptyIndex].x = x;
    camera[camIndex - 1].image2DPoint[emptyIndex].y = y;
    camera[camIndex - 1].image2DPoint[emptyIndex].pointRGB.r = point_RGB.r;
    camera[camIndex - 1].image2DPoint[emptyIndex].pointRGB.g = point_RGB.g;
    camera[camIndex - 1].image2DPoint[emptyIndex].pointRGB.b = point_RGB.b;
    //cout << camera[camIndex - 1].image2DPoint[emptyIndex].pointRGB.r << " " << camera[camIndex - 1].image2DPoint[emptyIndex].pointRGB.g << " " << camera[camIndex - 1].image2DPoint[emptyIndex].pointRGB.b << "\n";
}

void calculate2DPoint(int index, mat point_3D, RGB point_RGB){
    //calculate points
    mat point_2D;
    double x;
    double y;
    point_2D = camera[index].P * point_3D;
    x = point_2D(0, 0) / (point_2D(2, 0));
    y = point_2D(1, 0) / (point_2D(2, 0));
    //cout << "X: " << x << "  Y: " << y << "\n";
    //writeToFile(index, x, y, point_RGB);
    store2DPoint(index, x, y, point_RGB, point_3D);
}

void MainWindow :: readPatchFile() {
    std::string line;
    std::string lineRGB;
    std::string firstWord;
    int pointNum = 0;
    int currentFile = 0;
    string fileName = "cameraData/patch/" + getFileName("Patch", currentFile);
    mat point_3D;
    RGB point_RGB;
    lineCount = 0;
    ifstream RGBfile ("temp.txt");
    while (is_file_exist(fileName) == 1) {
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
                    point_3D << x << endr << y << endr << z << endr << t;
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
                    calculate2DPoint(stringToDouble(firstWord), point_3D, point_RGB);
                    getline(patchfile, line);
                    std::stringstream stream(line);
                    while(1) {
                        int n;
                        stream >> n;
                        if(!stream)
                            break;
                        calculate2DPoint(n, point_3D, point_RGB);
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

void processPLYFile() {
    std::string line;
    std::string x, y, z, nx, ny, nz, r, g, b;
    std::ofstream tempFile("temp.txt");
    lineCount = 0;
    int currentFile = 0;
    string fileName = "cameraData/patch/" + getFileName("PLY", currentFile);
    while (is_file_exist(fileName) == 1) {
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

void MainWindow::runCalc() {
    initialiseK();
    readFile();//contains camera information
    readPFiles();//contains the camera matrix of each camera
    processPLYFile();
    readPatchFile();//contains the the 3D coordinates
    //cout << "\nreached\n";
}

void MainWindow::setCameraBox() {
    int i = 0;
    for (i = 1; i <= cameraCount; i++) {
        if (camera[i - 1].image2DPoint[0].x != 0.0 && camera[i - 1].image2DPoint[0].y != 0.0) {
          ui->cameraBox->addItem(QString::number(i));
          ui->cam1Box->addItem(QString::number(i));
          ui->cam2Box->addItem(QString::number(i));
        }
    }
    ui->cam1Box->setEnabled(true);
    ui->cameraBox->setEnabled(true);
    ui->showButton->setEnabled(true);
    ui->showButton2->setEnabled(true);
    ui->resetButton->setEnabled(true);
}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->showButton->setEnabled(false);
    ui->showButton2->setEnabled(false);
    ui->cameraBox->setEnabled(false);
    //ui->cam1Box->setEnabled(false);
    //ui->cam2Box->setEnabled(false);
    ui->interpolateSlider->setRange(0, 100);
    ui->sliderSpinner->setRange(0, 100);
    QObject::connect(ui->interpolateSlider, SIGNAL(valueChanged(int)), ui->sliderSpinner, SLOT(setValue(int)));
    QObject::connect(ui->sliderSpinner, SIGNAL(valueChanged(int)), ui->interpolateSlider, SLOT(setValue(int)));
    //ui->interpolateSlider->setEnabled(false);
    ui->resetButton->setEnabled(false);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_calcButton_clicked()
{
    runCalc();
    ui->calcButton->setEnabled(false);
    setCameraBox();
}

//RGB view button
//display 2D camera views with RGB values
void MainWindow::on_showButton_clicked()
{
    QString Index = ui->cameraBox->currentText();
    int cameraIndex = Index.toInt() - 1;
    QVector<QPointF> points;
    int numPoints = 0;

    QImage image = QImage(2000, 2000, QImage::Format_RGB888);
    image.fill(QColor(Qt::white).rgb());

    for(int i = 0; i < MAX_POINTS; i++) {
        if (camera[cameraIndex].image2DPoint[i].x == 0.0 && camera[cameraIndex].image2DPoint[i].y == 0.0) {
            i = MAX_POINTS;
            break;
        }
        else {
            image.setPixel(camera[cameraIndex].image2DPoint[i].x / 2.0, camera[cameraIndex].image2DPoint[i].y / 2.0,
                           qRgb(camera[cameraIndex].image2DPoint[i].pointRGB.r, camera[cameraIndex].image2DPoint[i].pointRGB.g, camera[cameraIndex].image2DPoint[i].pointRGB.b));
            numPoints++;
        }
    }

    QGraphicsScene * scene = new QGraphicsScene();
    ui->graphicsView->setScene(scene);

    scene->addPixmap(QPixmap::fromImage(image));

}

//normal view button
//display 2D camera views
void MainWindow::on_showButton2_clicked()
{
    QString Index = ui->cameraBox->currentText();
    int cameraIndex = Index.toInt() - 1;
    QVector<QPointF> points;
    int numPoints = 0;

    for(int i = 0; i< 100; i++) {
       points.append(QPointF(i*5, i*5));
    }

    //QGraphicsView * view = new QGraphicsView();
    QGraphicsScene * scene = new QGraphicsScene();
    ui->graphicsView->setScene(scene);

    for(int i = 0; i < MAX_POINTS; i++) {
        if (camera[cameraIndex].image2DPoint[i].x == 0.0 && camera[cameraIndex].image2DPoint[i].y == 0.0) {
            i = MAX_POINTS;
            break;
        }
        else {
            double rad = 1;
            scene->addEllipse(camera[cameraIndex].image2DPoint[i].x / 2.0, camera[cameraIndex].image2DPoint[i].y / 2.0, rad, rad,
                        QPen(), QBrush(Qt::SolidPattern));
                        cout << "x: " << camera[cameraIndex].image2DPoint[i].x << " y: " << camera[cameraIndex].image2DPoint[i].y << "\n";
            //points.append(QPointF(camera[cameraIndex].image2DPoint[i].x, camera[cameraIndex].image2DPoint[i].y));
            numPoints++;
        }
    }
    ui->graphicsView->show();

}

void MainWindow::on_resetButton_clicked()
{
    ui->calcButton->setEnabled(true);
    ui->cameraBox->setEnabled(false);
    ui->showButton->setEnabled(false);
    ui->showButton2->setEnabled(false);
}

//save button function
//saves camera data locallto a file "ImagePointData.txt"
void MainWindow::on_saveButton_clicked ()
{
    int cameraIndex = 0;
    int pointIndex = 0;
    //stores focalLen, R, T, 2D points
    QString IMPointData = "ImagePointData.txt";
    QFile file(IMPointData);
    if (file.open(QIODevice::ReadWrite)) {
        QTextStream stream(&file);
        for(cameraIndex = 0; cameraIndex < cameraCount; cameraIndex++) {
            stream << " " << cameraIndex << endl;
            stream << camera[cameraIndex].focalLen << endl;
            stream << camera[cameraIndex].R(0, 0) << " " << camera[cameraIndex].R(0, 1) << " " << camera[cameraIndex].R(0, 2) << endl;
            stream << camera[cameraIndex].R(1, 0) << " " << camera[cameraIndex].R(1, 1) << " " << camera[cameraIndex].R(1, 2) << endl;
            stream << camera[cameraIndex].R(2, 0) << " " << camera[cameraIndex].R(2, 1) << " " << camera[cameraIndex].R(2, 2) << endl;
            stream << camera[cameraIndex].qR(0, 0) << " " << camera[cameraIndex].qR(1, 0) << " " << camera[cameraIndex].qR(2, 0) <<  " " << camera[cameraIndex].qR(3, 0) << endl;
            stream << camera[cameraIndex].T(0, 0) << " " << camera[cameraIndex].T(1, 0) << " " << camera[cameraIndex].T(2, 0) << endl;
            for(pointIndex = 0; pointIndex < MAX_POINTS; pointIndex++) {
                if(camera[cameraIndex].image2DPoint[pointIndex].x == 0 && camera[cameraIndex].image2DPoint[pointIndex].y == 0 && camera[cameraIndex].image2DPoint[pointIndex].pointRGB.r == 0 && camera[cameraIndex].image2DPoint[pointIndex].pointRGB.g == 0 && camera[cameraIndex].image2DPoint[pointIndex].pointRGB.b == 0) {
                    pointIndex = MAX_POINTS;
                }
                else {
                    stream << camera[cameraIndex].image2DPoint[pointIndex].x << " " << camera[cameraIndex].image2DPoint[pointIndex].y << " " << camera[cameraIndex].image2DPoint[pointIndex].pointRGB.r << " " << camera[cameraIndex].image2DPoint[pointIndex].pointRGB.g << " " << camera[cameraIndex].image2DPoint[pointIndex].pointRGB.b << endl;
                }
            }
            stream << endl;
        }
    }
    //stores 3D points
    QString Cam3DData = "3DPointData.txt";
    QFile file2(Cam3DData);
    if (file2.open(QIODevice::ReadWrite)) {
        QTextStream stream(&file2);
        for(cameraIndex = 0; cameraIndex < cameraCount; cameraIndex++) {
            stream << cameraIndex << endl;
            for(pointIndex = 0; pointIndex < MAX_POINTS; pointIndex++) {
                if(camera[cameraIndex].p3DPoint[pointIndex].x == 0 && camera[cameraIndex].p3DPoint[pointIndex].y == 0 && camera[cameraIndex].p3DPoint[pointIndex].y == 0) {
                    pointIndex = MAX_POINTS;
                }
                else {
                    stream << camera[cameraIndex].p3DPoint[pointIndex].x << " " << camera[cameraIndex].p3DPoint[pointIndex].y << " " << camera[cameraIndex].p3DPoint[pointIndex].z << endl;
                }
            }
            stream << endl;
        }
    }
}

//load camera data locally from the text file saved "ImagePointData.txt"
void loadIMPFile()
{
    int lineCounter = 1;
    int cameraIndex = 0;
    int pointIndex = 0;
    mat r1;
    mat r2;
    mat r3;
    cameraCount = 0;
    QFile IMPFile("ImagePointData.txt");
    if(!IMPFile.open(QIODevice::ReadOnly)) {
        QMessageBox::information(0, "data file not found", IMPFile.errorString());
    }

    QTextStream in(&IMPFile);

    while(!in.atEnd()) {
        QString line = in.readLine();
        if(lineCounter == 1) {
            //camera index
            cameraIndex = line.toInt();
            cameraCount++;
        }
        else if(lineCounter == 2) {
            //focal length
            camera[cameraIndex].focalLen = line.toDouble();
        }
        else if(lineCounter == 3) {
            //R matrix r1
            r1 << line.split(" ")[0].toDouble() << line.split(" ")[1].toDouble() << line.split(" ")[2].toDouble() << endr;
        }
        else if(lineCounter == 4) {
            //R matrix r2
            r2 << line.split(" ")[0].toDouble() << line.split(" ")[1].toDouble() << line.split(" ")[2].toDouble() << endr;
            r3 = join_cols(r1, r2);
        }
        else if(lineCounter == 5) {
            //R matrix r3
            camera[cameraIndex].R << line.split(" ")[0].toDouble() << line.split(" ")[1].toDouble() << line.split(" ")[2].toDouble() << endr;
            camera[cameraIndex].R = join_cols(r3, r2);
        }
        else if(lineCounter == 6) {
            //T matrix
            camera[cameraIndex].qR << line.split(" ")[0].toDouble() << endr
                                   << line.split(" ")[1].toDouble() << endr
                                   << line.split(" ")[2].toDouble() << endr
                                   << line.split(" ")[3].toDouble() << endr;
        }
        else if(lineCounter == 7) {
            //T matrix
            camera[cameraIndex].T << line.split(" ")[0].toDouble() << endr
                                  << line.split(" ")[1].toDouble() << endr
                                  << line.split(" ")[2].toDouble() << endr;
        }
        else if(line.length() == 0) {
            lineCounter = 0;
            pointIndex = 0;
        }
        else {
            camera[cameraIndex].image2DPoint[pointIndex].x = line.split(" ")[0].toDouble();
            camera[cameraIndex].image2DPoint[pointIndex].y = line.split(" ")[1].toDouble();
            camera[cameraIndex].image2DPoint[pointIndex].pointRGB.r = line.split(" ")[2].toDouble();
            camera[cameraIndex].image2DPoint[pointIndex].pointRGB.g = line.split(" ")[3].toDouble();
            camera[cameraIndex].image2DPoint[pointIndex].pointRGB.b = line.split(" ")[4].toDouble();
            pointIndex++;
        }
        lineCounter++;
    }

    IMPFile.close();
}

//load all the 3D points locally from the text file saved "3DPointData.txt"
void load3DPFile()
{
    int lineCounter = 1;
    int cameraIndex = 0;
    int pointIndex = 0;
    QFile PFile("3DPointData.txt");
    if(!PFile.open(QIODevice::ReadOnly)) {
        QMessageBox::information(0, "data file not found", PFile.errorString());
    }

    QTextStream in(&PFile);

    while(!in.atEnd()) {
        QString line = in.readLine();
        //qDebug() << line << "\n";
        if(lineCounter == 1) {
            cameraIndex = line.toInt();
        }
        else if(line.length() == 0) {
            lineCounter = 0;
            pointIndex = 0;
        }
        else {
            camera[cameraIndex].p3DPoint[pointIndex].x = line.split(" ")[0].toDouble();
            camera[cameraIndex].p3DPoint[pointIndex].y = line.split(" ")[1].toDouble();
            camera[cameraIndex].p3DPoint[pointIndex].z = line.split(" ")[2].toDouble();
            pointIndex++;
        }
        lineCounter++;
    }
    PFile.close();
}

void MainWindow::on_loadButton_clicked()
{
    loadIMPFile();
    load3DPFile();
    setCameraBox();
}

//display interpolated image when interpolate button is clicked
void MainWindow::showIImage()
{
    QVector<QPointF> points;
    int numPoints = 0;

    for(int i = 0; i< 100; i++) {
       points.append(QPointF(i*5, i*5));
    }

    //QGraphicsView * view = new QGraphicsView();
    QGraphicsScene * scene = new QGraphicsScene();
    ui->graphicsView->setScene(scene);

    for(int i = 0; i < MAX_POINTS; i++) {
        if (iCamera.image2DPoint[i].x == 0.0 && iCamera.image2DPoint[i].y == 0.0) {
            i = MAX_POINTS;
            break;
        }
        else {
            double rad = 1;
            scene->addEllipse(iCamera.image2DPoint[i].x / 2.0, iCamera.image2DPoint[i].y / 2.0, rad, rad,
                        QPen(), QBrush(Qt::SolidPattern));
            numPoints++;
        }
    }
    ui->graphicsView->show();

}

//interpolate button function
//interpolate two different 2D views fro two separate cameras
void MainWindow::on_interpolateButton_clicked()
{
    QString c1Index = ui->cam1Box->currentText();
    QString c2Index = ui->cam2Box->currentText();
    mat RT;
    mat iqR;
    eulerAngles iEA;
    double h = ui->sliderSpinner->value();
    h = h / 100.0;
    mat rx, ry, rz;
    int cam1 = c1Index.toInt();
    int cam2 = c2Index.toInt();

    mat point2D;
    int pointIndex = 0;
    double fL = focalLen(camera[cam1 - 1].focalLen, camera[cam2 - 1].focalLen, h);
    iCamera.K << fL << 0 << imageCentreX << endr
              << 0 << fL << imageCentreY << endr
              << 0 << 0 << 1;

    //quaternion interpolation method
    //iqR = interpolateQR(camera[cam1 - 1].qR, camera[cam2 - 1].qR, h);
    //iCamera.R = qRToRotation(iqR);

    //euler angle interpolation method
    iEA = interpolateEuler(computeEuler(camera[cam1 - 1].R), computeEuler(camera[cam2 - 1].R), h);

    double c1 = cos(iEA.theta_x);
    double s1 = sin(iEA.theta_x);
    double c2 = cos(iEA.theta_z);
    double s2 = sin(iEA.theta_z);
    double c3 = cos(iEA.theta_y);
    double s3 = sin(iEA.theta_y);

    double R_00 = c1 * c3;
    double R_01 = -c1 * s3 * c2 + s1 * s2;
    double R_02 = c1 * s3 * s2 + s1 * c2;
    double R_10 = s3;
    double R_11 = c3 * c2;
    double R_12 = -c3 * s2;
    double R_20 = -s1 * c3;
    double R_21 = s1 * s3 * c2 + c1 * s2;
    double R_22 = -s1 * s3 * s2 + c1 * c2;

    iCamera.R << R_00 << R_01 << R_02 << endr
              << R_10 << R_11 << R_12 << endr
              << R_20 << R_21 << R_22;



    iCamera.T = interpolateTranslation(camera[cam1 - 1].T, camera[cam2 - 1].T, h);
    RT = join_rows(iCamera.R, iCamera.T);

    iCamera.P = iCamera.K * RT;

    cout << "\n R: \n" << iCamera.R << "\n\n";
    cout << "\n T: \n" << iCamera.T << "\n\n";
    cout << "\n P: \n" << iCamera.P << "\n\n";

    for (pointIndex = 0; pointIndex < MAX_POINTS; pointIndex++) {
        if (iCamera.p3DPoint[pointIndex].x == 0 && iCamera.p3DPoint[pointIndex].y == 0 && iCamera.p3DPoint[pointIndex].z == 0) {
            pointIndex = MAX_POINTS;
            break;
        }
        else {
            mat p3D;
            p3D << iCamera.p3DPoint[pointIndex].x << endr << iCamera.p3DPoint[pointIndex].y << endr << iCamera.p3DPoint[pointIndex].z << endr << 1;
            point2D = iCamera.P * p3D;

            iCamera.image2DPoint[pointIndex].x = point2D(0, 0) / point2D(2, 0);
            iCamera.image2DPoint[pointIndex].y = point2D(1, 0) / point2D(2, 0);
        }
    }
    showIImage();
}

//match button function
//find all the matching 3D points between two cameras
void MainWindow::on_matchButton_clicked()
{
    QString c1Index = ui->cam1Box->currentText();
    QString c2Index = ui->cam2Box->currentText();
    int cam1 = c1Index.toInt();
    int cam2 = c2Index.toInt();
    int pointIndex = 0;
    int pointIndex2 = 0;
    int matchIndex = 0;
    int matchNum = 0;

    for(pointIndex = 0; pointIndex < MAX_POINTS; pointIndex++) {
        if(camera[cam1 - 1].p3DPoint[pointIndex].x == 0 && camera[cam1 - 1].p3DPoint[pointIndex].y == 0 && camera[cam1 - 1].p3DPoint[pointIndex].z == 0) {
                qDebug() << matchNum << " points matched\n";
                pointIndex = MAX_POINTS;
                break;
        }
        else {
            for(pointIndex2 = 0; pointIndex2 < MAX_POINTS; pointIndex2++) {
                if(camera[cam2 - 1].p3DPoint[pointIndex2].x == 0 && camera[cam2 - 1].p3DPoint[pointIndex2].y == 0 && camera[cam2 - 1].p3DPoint[pointIndex2].z == 0) {
                    pointIndex2 = MAX_POINTS;
                }
                else {
                    if(camera[cam1 - 1].p3DPoint[pointIndex].x == camera[cam2 - 1].p3DPoint[pointIndex2].x && camera[cam1 - 1].p3DPoint[pointIndex].y == camera[cam2 - 1].p3DPoint[pointIndex2].y && camera[cam1 - 1].p3DPoint[pointIndex].z == camera[cam2 - 1].p3DPoint[pointIndex2].z) {
                        iCamera.p3DPoint[matchIndex].x = camera[cam1 - 1].p3DPoint[pointIndex].x;
                        iCamera.p3DPoint[matchIndex].y = camera[cam1 - 1].p3DPoint[pointIndex].y;
                        iCamera.p3DPoint[matchIndex].z = camera[cam1 - 1].p3DPoint[pointIndex].z;
                        matchIndex++;
                        matchNum++;
                    }
                }
            }
        }
    }
}
