#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include "armadillo"

#include <QGraphicsView>
#include <QGraphicsScene>
#include <QPointF>
#include <QVector>

using namespace arma;
using namespace std;

const int MAX_CAM_NUM = 200;
const int MAX_2D_POINTS = 100000;

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

    point2D image2DPoint [MAX_2D_POINTS];//2D points
};

cameraInfo camera [MAX_CAM_NUM];

void initialiseK(){
    double imageCentreX = 540.0;
    double imageCentreY = 960.0;
    for (int i; i < MAX_CAM_NUM; i++) {
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
                //camera[cameraCount - 1].R = join_cols(r3, r2);
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
    else if (fileType == "Patch") {
        //option-0001.patch
        stringstream convert;
        convert << index;
        fileName = convert.str();
        fileName = fileName + ".patch";
        while (fileName.length() < 10) {
            fileName = '0' + fileName;
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

void store2DPoint (int camIndex, double x, double y){
    int emptyIndex = 0;
    for (int i = 0; i < MAX_2D_POINTS; i++) {
        if (camera[camIndex].image2DPoint[i].x == 0 && camera[camIndex].image2DPoint[i].y == 0) {
            emptyIndex = i;
            i = MAX_2D_POINTS;
        }
    }
    camera[camIndex].image2DPoint[emptyIndex].x = x;
    camera[camIndex].image2DPoint[emptyIndex].y = y;
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
    store2DPoint(index, x, y);
}

void MainWindow :: readPatchFile() {
    std::string line;
    std::string firstWord;
    int pointNum = 0;
    int currentFile = 0;
    string fileName = "cameraData/patch/" + getFileName("Patch", currentFile);
    mat point_3D;
    RGB point_RGB;
    lineCount = 0;
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
                else if (lineCount == 5) {
                    double r = stringToDouble(firstWord);
                    double g, b;
                    patchfile >> g >> b;
                    point_RGB.r = r;
                    point_RGB.g = g;
                    point_RGB.b = b;
                    cout << r << " " << g << " " << b << "\n";
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
    cout << "\ncalculation completed\n";
}

void MainWindow::runCalc() {
    initialiseK();
    readFile();//contains camera information
    readPFiles();//contains the camera matrix of each camera
    readPatchFile();//contains the the 3D coordinates
    //cout << "\nreached\n";
}

void MainWindow::setCameraBox() {
    for (int i = 1; i <= cameraCount; i++) {
        ui->cameraBox->addItem(QString::number(i));
    }
    ui->cameraBox->setEnabled(true);
    ui->showButton->setEnabled(true);
    ui->resetButton->setEnabled(true);
}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->showButton->setEnabled(false);
    ui->cameraBox->setEnabled(false);
    ui->resetButton->setEnabled(false);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_calcButton_clicked()
{
    ui->statusLabel->setText("Processing camera data...");
    runCalc();
    ui->statusLabel->setText("Completed");
    ui->calcButton->setEnabled(false);
    setCameraBox();
}

void MainWindow::on_showButton_clicked()
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

    for(int i = 0; i < MAX_2D_POINTS; i++) {
        if (camera[cameraIndex].image2DPoint[i].x == 0.0 && camera[cameraIndex].image2DPoint[i].y == 0.0) {
            i = MAX_2D_POINTS;
            break;
        }
        else {
            double rad = 1;
            scene->addEllipse(camera[cameraIndex].image2DPoint[i].x, camera[cameraIndex].image2DPoint[i].y, rad, rad,
                        QPen(), QBrush(Qt::SolidPattern));
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
    ui->statusLabel->setText("");
}
