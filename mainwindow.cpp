#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "calculation.h"
//#include "data.h"

#include <math.h>
#include <iostream>
#include <fstream>
#include <sstream>

#include <ANN/ANN.h>

#include <QDebug>
#include <QImage>
#include <QFile>
#include <QMessageBox>
#include <QTextStream>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QPointF>
#include <QVector>
#include <QSlider>
#include <QSpinBox>
#include <QQuaternion>
#include <QGenericMatrix>

double imageCentreX = 414.0;
double imageCentreY = 646.0;

cameraInfo camera [MAX_CAM_NUM];
cameraInfo iCamera;//interpolated camera
cameraInfo oCamera;//original camera at t = 0
Vector4d i3DPoint[MAX_POINTS];
MatrixXd dV[MAX_POINTS];

int oCamInd;
int cameraNum = 0;
int cameraCount = 0;
int pointCount = 0;
int numOfPoint = 0;

void initialiseK(){
    int i =0;
    for (i = 0; i < MAX_CAM_NUM; i++) {
        camera[i].K << 0, 0, imageCentreX,
                       0, 0, imageCentreY,
                       0, 0, 1;
    }
}

void writeToFile (int camIndex, double x, double y, RGB RGBVal) {
    ofstream outputFile;
    string index;
    ostringstream convert;
    convert << camIndex;
    index = convert.str();
    if (checkFileExist("output/" + index + ".txt"))
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

void MainWindow::runCalc() {
    //int cameraCount = 0;
    initialiseK();
    qDebug() << "K initialised";
    cameraCount = readCamFile(camera);//contains camera information
    qDebug() << "cam file read";
    readPFiles(camera, cameraCount);//contains the camera matrix of each camera
    qDebug() << "P file read";
    processPLYFile();
    qDebug() << "processed";
    readPatchFile(camera);//contains the the 3D coordinates
    qDebug() << "Patch file read";
    //cout << "\nreached\n";
}

void MainWindow::setCameraBox() {
    int i = 0;
    for (i = 1; i <= cameraCount; i++) {
        if (camera[i - 1].image2DPoint[0].x != 0.0 && camera[i - 1].image2DPoint[0].y != 0.0) {
          ui->cameraBox->addItem(QString::number(i));
          ui->cam1Box->addItem(QString::number(i));
        }
    }
    ui->cam1Box->setEnabled(true);
    ui->cameraBox->setEnabled(true);
    ui->showButton->setEnabled(true);
    ui->showButton2->setEnabled(true);
    ui->resetButton->setEnabled(true);
}

void MainWindow::on_calcButton_clicked()
{
    runCalc();
    ui->calcButton->setEnabled(false);
    ui->saveButton->setEnabled(true);
    ui->loadButton->setEnabled(false);

    setCameraBox();

    ui->cameraBox->setEnabled(true);
    ui->showButton->setEnabled(true);
    ui->showButton2->setEnabled(true);
    ui->resetButton->setEnabled(true);
    ui->cam1Box->setEnabled(true);
    ui->selectButton->setEnabled(true);
}

//RGB view button
//display 2D camera views with RGB values
void MainWindow::on_showButton_clicked()
{
    QString Index = ui->cameraBox->currentText();
    int cameraIndex = Index.toInt() - 1;
    QVector<QPointF> points;
    int numPoints = 0;

    QImage image = QImage(imageCentreX * 2, imageCentreY * 2, QImage::Format_RGB888);
    image.fill(QColor(Qt::black).rgb());

    for(int i = 0; i < MAX_POINTS; i++) {
        if (camera[cameraIndex].image2DPoint[i].x == 0.0 && camera[cameraIndex].image2DPoint[i].y == 0.0) {
            i = MAX_POINTS;
            break;
        }
        else {
            image.setPixel(camera[cameraIndex].image2DPoint[i].x, camera[cameraIndex].image2DPoint[i].y,
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
                        //cout << "x: " << camera[cameraIndex].image2DPoint[i].x << " y: " << camera[cameraIndex].image2DPoint[i].y << "\n";
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
            stream << camera[cameraIndex].C(0, 0) << " " << camera[cameraIndex].C(1, 0) << " " << camera[cameraIndex].C(2, 0) << endl;
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
    Vector3d r1;
    Vector3d r2;
    Matrix3d r;
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
            //camera position
            camera[cameraIndex].C << line.split(" ")[0].toDouble(),
                                     line.split(" ")[1].toDouble(),
                                     line.split(" ")[2].toDouble();
        }
        else if(lineCounter == 4) {
            //R matrix r1
            r1 << line.split(" ")[0].toDouble(), line.split(" ")[1].toDouble(), line.split(" ")[2].toDouble();
        }
        else if(lineCounter == 5) {
            //R matrix r2
            r2 << line.split(" ")[0].toDouble(), line.split(" ")[1].toDouble(), line.split(" ")[2].toDouble();
            //r3 = join_cols(r1, r2);
        }
        else if(lineCounter == 6) {
            //R matrix r3
            camera[cameraIndex].R << r1, r2, line.split(" ")[0].toDouble(), line.split(" ")[1].toDouble(), line.split(" ")[2].toDouble();
            //camera[cameraIndex].R = join_cols(r3, r2);
        }
        else if(lineCounter == 7) {
            //qR matrix
            camera[cameraIndex].qR << line.split(" ")[0].toDouble(),
                                      line.split(" ")[1].toDouble(),
                                      line.split(" ")[2].toDouble(),
                                      line.split(" ")[3].toDouble();
        }
        else if(lineCounter == 8) {
            //T matrix
            camera[cameraIndex].T << line.split(" ")[0].toDouble(),
                                     line.split(" ")[1].toDouble(),
                                     line.split(" ")[2].toDouble();
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
    ui->cam1Box->setEnabled(true);
    ui->selectButton->setEnabled(true);
}

//display interpolated image when interpolate button is clicked
void MainWindow::showIImage(int numOfPoints)
{
    QImage image = QImage(imageCentreX, imageCentreY, QImage::Format_RGB32);
    image.fill(QColor(Qt::black).rgb());

    for(int i = 0; i < numOfPoints; i++) {
        if(iCamera.image2DPoint[i].x != 0 && iCamera.image2DPoint[i].y != 0 && iCamera.image2DPoint[i].x != 10000 && iCamera.image2DPoint[i].y != 10000) {
            image.setPixel(int(iCamera.image2DPoint[i].x) / 2, int(iCamera.image2DPoint[i].y) / 2,
                            qRgb(iCamera.image2DPoint[i].pointRGB.r, iCamera.image2DPoint[i].pointRGB.g, iCamera.image2DPoint[i].pointRGB.b));
        }
    }
    QGraphicsScene * scene = new QGraphicsScene();
    ui->graphicsView->setScene(scene);

    scene->addPixmap(QPixmap::fromImage(image));
}

//interpolate button function
//interpolate two different 2D views fro two separate cameras
void MainWindow::on_interpolateButton_clicked()
{
    QString c1Index = ui->cam1Box->currentText();
    QString c2Index = ui->cam2Box->currentText();
    int cam1 = c1Index.toInt();
    int cam2 = c2Index.toInt();

    double t = ui->sliderSpinner->value() / 10.0;

    MatrixXd RT(3,4);
    MatrixXd xy(2, 1);

    double fL = focalLen(camera[cam1 - 1].focalLen, camera[cam2 - 1].focalLen, t);
    iCamera.K << fL, 0, imageCentreX,
                 0, fL, imageCentreY,
                 0, 0, 1;

    iCamera.R = interpolateQR(camera, cam1, cam2, t);

    iCamera.T = interpolateTranslation(camera[cam1 - 1].T, camera[cam2 - 1].T, t);

    RT << iCamera.R(0, 0), iCamera.R(0, 1), iCamera.R(0, 2), iCamera.T(0, 0),
          iCamera.R(1, 0), iCamera.R(1, 1), iCamera.R(1, 2), iCamera.T(1, 0),
          iCamera.R(2, 0), iCamera.R(2, 1), iCamera.R(2, 2), iCamera.T(2, 0);

    iCamera.P = iCamera.K * RT;

    xy(0, 0) = 0;
    xy(1, 0) = 0;

    for(int idx = 0; idx < MAX_POINTS; idx++) {
        iCamera.image2DPoint[idx].x = 0;
        iCamera.image2DPoint[idx].y = 0;
        iCamera.image2DPoint[idx].pointRGB.r = 0;
        iCamera.image2DPoint[idx].pointRGB.g = 0;
        iCamera.image2DPoint[idx].pointRGB.b = 0;
        dV[idx] = xy;
    }

    QString fileName = "output/data.pts";
    QFile file(fileName);
    file.open(QIODevice::WriteOnly | QIODevice::Truncate);
    QTextStream stream(&file);
    for (int i = 0; i < numOfPoint; i++) {
        if (!(iCamera.p3DPoint[i].x == 0 && iCamera.p3DPoint[i].y == 0 && iCamera.p3DPoint[i].z == 0)) {
            Vector4d p3D;
            Vector3d p2D;
            p3D << camera[cam1 - 1].p3DPoint[i].x, camera[cam1 - 1].p3DPoint[i].y, camera[cam1 - 1].p3DPoint[i].z, 1;
            p2D = iCamera.P * p3D;

            iCamera.image2DPoint[i].x = p2D(0, 0) / p2D(2, 0);
            iCamera.image2DPoint[i].y = p2D(1, 0) / p2D(2, 0);
            iCamera.image2DPoint[i].pointRGB = camera[cam1 - 1].image2DPoint[i].pointRGB;
            xy(0, 0) = iCamera.image2DPoint[i].x - camera[cam1 - 1].image2DPoint[i].x;
            xy(1, 0) = iCamera.image2DPoint[i].y - camera[cam1 - 1].image2DPoint[i].y;
            dV[i] = xy;
        }
        else {
            iCamera.image2DPoint[i].x = 10000;
            iCamera.image2DPoint[i].y = 10000;
            xy(0, 0) = 10000;
            xy(1, 0) = 10000;
            dV[i] = xy;
        }
        stream << iCamera.image2DPoint[i].x << " " << iCamera.image2DPoint[i].y << endl;
    }
    showIImage(numOfPoint);
}

//match button function
//find all the matching 3D points between two cameras
void MainWindow::on_matchButton_clicked()
{
    writeQueryToFile(imageCentreX * 2, imageCentreY * 2);
    ui->matchButton->setEnabled(false);
    QString c1Index = ui->cam1Box->currentText();
    QString c2Index = ui->cam2Box->currentText();
    int cam1 = c1Index.toInt();
    int cam2 = c2Index.toInt();
    numOfPoint = 0;
    for(int idx = 0; idx < MAX_POINTS; idx++) {
        iCamera.p3DPoint[idx].x = 0;
        iCamera.p3DPoint[idx].y = 0;
        iCamera.p3DPoint[idx].z = 0;
    }
    for(int pointIndex = 0; pointIndex < MAX_POINTS; pointIndex++) {
        if(camera[cam1 - 1].p3DPoint[pointIndex].x == 0 && camera[cam1 - 1].p3DPoint[pointIndex].y == 0 && camera[cam1 - 1].p3DPoint[pointIndex].z == 0) {
            qDebug() << "number of points: " << pointIndex << "\n";
            break;
        }
        for(int pointIndex2 = 0; pointIndex2 < MAX_POINTS; pointIndex2++) {
            if(camera[cam2 - 1].p3DPoint[pointIndex2].x == 0 && camera[cam2 - 1].p3DPoint[pointIndex2].y == 0 && camera[cam2 - 1].p3DPoint[pointIndex2].z == 0) {
                break;
            }
            else if(camera[cam1 - 1].p3DPoint[pointIndex].x == camera[cam2 - 1].p3DPoint[pointIndex2].x &&
                    camera[cam1 - 1].p3DPoint[pointIndex].y == camera[cam2 - 1].p3DPoint[pointIndex2].y &&
                    camera[cam1 - 1].p3DPoint[pointIndex].z == camera[cam2 - 1].p3DPoint[pointIndex2].z) {
                iCamera.p3DPoint[pointIndex].x = camera[cam1 - 1].p3DPoint[pointIndex].x;
                iCamera.p3DPoint[pointIndex].y = camera[cam1 - 1].p3DPoint[pointIndex].y;
                iCamera.p3DPoint[pointIndex].z = camera[cam1 - 1].p3DPoint[pointIndex].z;
                break;
            }
            else {
                iCamera.p3DPoint[pointIndex].x = 0;
                iCamera.p3DPoint[pointIndex].y = 0;
                iCamera.p3DPoint[pointIndex].z = 0;
            }
        }
        numOfPoint++;
    }
    ui->interpolateSlider->setEnabled(true);
    ui->sliderSpinner->setEnabled(true);
    ui->interpolateButton->setEnabled(true);
    on_interpolateButton_clicked();
}

void MainWindow::on_selectButton_clicked()
{
    ui->matchButton->setEnabled(true);
    ui->interpolateSlider->setEnabled(false);
    ui->sliderSpinner->setEnabled(false);
    ui->interpolateButton->setEnabled(false);
    QString cIndex = ui->cam1Box->currentText();
    int camIndex = cIndex.toInt() - 1;
    oCamInd = camIndex;
    int i = 0;
    int neighbourCam1 = -1;
    int neighbourCam2 = -1;
    Vector3d temp;
    double dist = 0;
    double minDist = 1000;
    double minDist2 = 1000;

    QString filePath = QString(("visualize/" + getFileName("Img", camIndex)).c_str());
    QPixmap image(filePath);
    QGraphicsScene * scene = new QGraphicsScene();
    ui->imgView->setScene(scene);
    scene->addPixmap(image.scaled(imageCentreX, imageCentreY));

    for(i = 0; i < cameraCount; i++) {
        if ((camera[i].image2DPoint[0].x != 0.0) && (camera[i].image2DPoint[0].y != 0.0)) {
            temp = camera[i].C - camera[camIndex].C;
            dist = sqrt(temp.dot(temp));
            if (dist != 0) {
                if (dist < minDist) {
                    minDist = dist;
                    neighbourCam1 = i;
                }
                else if (dist < minDist2) {
                    minDist2 = dist;
                    neighbourCam2 = i;
                }
            }
        }
    }
    ui->cam2Box->clear();
    if (neighbourCam2 != -1) {
        ui->cam2Box->addItem(QString::number(neighbourCam2 + 1));
    }
    if (neighbourCam1 != -1) {
        ui->cam2Box->addItem(QString::number(neighbourCam1 + 1));
    }
    ui->cam2Box->setEnabled(true);
    ui->matchButton->setEnabled(true);
}

void MainWindow::on_getRGBValButton_clicked()
{
    QImage image = pixelMapping(numOfPoint, dV, oCamInd);

    QGraphicsScene * scene = new QGraphicsScene();
    ui->colorView->setScene(scene);

    scene->addPixmap(QPixmap::fromImage(image));
}

void MainWindow::on_getRGBValImpButton_clicked()
{
    QImage image = pixelMappingImproved(numOfPoint, dV, oCamInd);

    QGraphicsScene * scene = new QGraphicsScene();
    ui->colorView->setScene(scene);

    scene->addPixmap(QPixmap::fromImage(image));
}


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->saveButton->setEnabled(false);
    ui->showButton->setEnabled(false);
    ui->showButton2->setEnabled(false);
    ui->cameraBox->setEnabled(false);
    ui->cam1Box->setEnabled(false);
    ui->cam2Box->setEnabled(false);
    ui->selectButton->setEnabled(false);
    ui->matchButton->setEnabled(false);
    ui->interpolateSlider->setEnabled(false);
    ui->interpolateSlider->setRange(0, 10);
    ui->sliderSpinner->setEnabled(false);
    ui->sliderSpinner->setRange(0, 10);
    ui->interpolateButton->setVisible(false);
    ui->interpolateButton->setEnabled(false);
    QObject::connect(ui->interpolateSlider, SIGNAL(valueChanged(int)), ui->sliderSpinner, SLOT(setValue(int)));
    QObject::connect(ui->sliderSpinner, SIGNAL(valueChanged(int)), ui->interpolateSlider, SLOT(setValue(int)));
    //ui->interpolateSlider->setEnabled(false);
    ui->resetButton->setEnabled(false);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_interpolateSlider_actionTriggered(int action)
{
    //on_interpolateButton_clicked();
}

void MainWindow::on_sliderSpinner_valueChanged(int arg1)
{
    on_interpolateButton_clicked();
}

void MainWindow::on_cam2Box_currentTextChanged(const QString &arg1)
{
    ui->matchButton->setEnabled(true);
}
