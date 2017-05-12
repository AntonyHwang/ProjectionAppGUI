#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "calculation.h"
#include <math.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>      /* printf, NULL */
#include <stdlib.h>     /* srand, rand */
#include <ctime>
#include <cstdlib>
#include <time.h>
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
#include <QApplication>
#include <QProcess>
#include <QElapsedTimer>

QString method;
QString clustering;

cameraInfo camera [MAX_CAM_NUM];
cameraInfo iCamera;//interpolated camera
cameraInfo oCamera;//original camera at t = 0
Vector4d i3DPoint[MAX_POINTS];
point2D temp_2D_points[MAX_POINTS];
MatrixXd dV[MAX_POINTS];
//variables for clustering
int cluster_num = 0;
MatrixXd cluster_dV[MAX_POINTS];
int point_cluster_num[MAX_POINTS];
QColor clusterRGB[MAX_POINTS];

int oCamInd;
int cameraNum = 0;
int cameraCount = 0;
int pointCount = 0;
int numOfPoint = 0;

void MainWindow::disable_all_ui() {
    ui->showButton->setEnabled(false);
    ui->showButton2->setEnabled(false);
    ui->cameraBox->setEnabled(false);
    ui->cam1Box->setEnabled(false);
    ui->cam2Box->setEnabled(false);
    ui->selectButton->setEnabled(false);
    ui->matchButton->setEnabled(false);
    ui->interpolateSlider->setEnabled(false);
    ui->full_interpolation->setEnabled(false);
    ui->resetButton->setEnabled(false);
}

void MainWindow::enable_all_ui() {
    ui->showButton->setEnabled(true);
    ui->showButton2->setEnabled(true);
    ui->cameraBox->setEnabled(true);
    ui->cam1Box->setEnabled(true);
    ui->cam2Box->setEnabled(true);
    ui->selectButton->setEnabled(true);
    ui->matchButton->setEnabled(true);
    ui->interpolateSlider->setEnabled(true);
    ui->full_interpolation->setEnabled(true);
    ui->resetButton->setEnabled(true);
}

bool check_cluster(int queryPt_idx, int nearbyPt_idx) {

    if((abs(dV[queryPt_idx](0,0) - dV[nearbyPt_idx](0,0)) + abs(dV[queryPt_idx](1,0) - dV[nearbyPt_idx](1,0))) > 10) {
        return 0;
    }
    else {
        return 1;
    }
}

void cluster_point (double x, double y, int queryPt_idx, int mark_clustered[]) {
    int	k = 150;
    int	dim	= 2;
    double eps = 0;
    int maxPts = numOfPoint;

    istream* dataIn	= NULL;

    int					nPts;
    ANNpointArray		dataPts;
    ANNpoint			queryPt;
    ANNidxArray			nnIdx;
    ANNdistArray		dists;
    ANNkd_tree*			kdTree;

    queryPt = annAllocPt(dim);
    dataPts = annAllocPts(maxPts, dim);
    nnIdx = new ANNidx[k];
    dists = new ANNdist[k];

    queryPt[0] = x;
    queryPt[1] = y;

    static ifstream dataStream;

    dataStream.open("output/data.pts");
    dataIn = &dataStream;

    nPts = 0;
    while (nPts < maxPts && readPt(*dataIn, dataPts[nPts], dim)) {
        nPts++;
    }

    kdTree = new ANNkd_tree(
                    dataPts,
                    nPts,
                    dim);

    kdTree->annkSearch(
            queryPt,
            k,
            nnIdx,
            dists,
            eps);

    double maxDist = 150;
    int nearbyPt_found = 0;

    for (int i = 0; i < k; i++) {
        dists[i] = sqrt(dists[i]);
        if(dists[i] <= maxDist) {
            if (check_cluster(queryPt_idx, nnIdx[i]) && dataPts[nnIdx[i]][0] != queryPt[0] && dataPts[nnIdx[i]][1] != queryPt[1]) {
                nearbyPt_found = 1;
                if (mark_clustered[nnIdx[i]] == 0 && mark_clustered[queryPt_idx] == 0) {
                    cluster_num++;

                    point_cluster_num[queryPt_idx] = cluster_num;
                    mark_clustered[queryPt_idx] = 1;

                    point_cluster_num[nnIdx[i]] = cluster_num;
                    mark_clustered[nnIdx[i]] = 1;
                }
                else if (mark_clustered[queryPt_idx] == 0 && mark_clustered[nnIdx[i]] == 1){
                    point_cluster_num[queryPt_idx] = point_cluster_num[nnIdx[i]];
                    mark_clustered[queryPt_idx] = 1;
                }
                else if (mark_clustered[queryPt_idx] == 1 && mark_clustered[nnIdx[i]] == 0){
                    point_cluster_num[nnIdx[i]] = point_cluster_num[queryPt_idx];
                    mark_clustered[nnIdx[i]] = 1;
                }
                else {
                    //do nothing
                }
            }
        }
        qApp->processEvents(QEventLoop::ExcludeSocketNotifiers,1);
    }
    if (nearbyPt_found == 0) {
        mark_clustered[queryPt_idx] = 1;
        point_cluster_num[queryPt_idx] = 0;
    }
    dataStream.close();
    delete [] nnIdx;
    delete [] dists;
    delete kdTree;
    annClose();
}

void MainWindow::cluster_all_points (int numOfPoint) {
    int mark_clustered[numOfPoint];
    cluster_num = 0;
    //initialisation
    for (int i = 0; i < numOfPoint; i++) {
        point_cluster_num[i] = 0;
        mark_clustered[i] = 0;
    }
    for (int queryPt_idx = 0; queryPt_idx < numOfPoint; queryPt_idx++) {
        //check if it is already clustered
        if (mark_clustered[queryPt_idx] == 1) {
            //do nothing
        }
        //skip the missing pixels
        else if ((dV[queryPt_idx](0,0) == 10000 && dV[queryPt_idx](1,0) == 10000) || (dV[queryPt_idx](0,0) == 0 && dV[queryPt_idx](1,0) == 0) || (dV[queryPt_idx](0,0) != dV[queryPt_idx](0,0))) {
            mark_clustered[queryPt_idx] = 1;
        }
        //cluster point
        else if (mark_clustered[queryPt_idx] == 0) {
            cluster_point(iCamera.image2DPoint[queryPt_idx].x, iCamera.image2DPoint[queryPt_idx].y, queryPt_idx, mark_clustered);
        }
    }
    //calculate cluster average displacement
    for (int cluster = 1; cluster <= cluster_num; cluster++) {
        double point_sum = 0;
        MatrixXd dV_sum (2,1);
        dV_sum(0,0) = 0;
        dV_sum(1,0) = 0;
        for (int i = 0; i < numOfPoint; i++) {
            if (point_cluster_num[i] == cluster) {
                dV_sum += dV[i];
                //DEBUG print translation vector at query point
                /*double a = dV[i](0,0);
                double b = dV[i](1,0);
                qDebug() << a << " " << b << "\n";*/
                point_sum++;
            }
        }
        double x = 0;
        double y = 0;
        if (point_sum != 0) {
            x = dV_sum(0,0) / point_sum;
            y = dV_sum(1,0) / point_sum;
            MatrixXd temp_dV(2,1);
            temp_dV << x, y;
            cluster_dV[cluster] = temp_dV;
        }
    }
    //DEBUG PRINT OUT CLUSTERS AND DV
    /*QFile tempfile("output/tempfile.txt");
    tempfile.open(QIODevice::WriteOnly);
    QTextStream tempFile(&tempfile);
    for (int cluster = 1; cluster <= cluster_num; cluster++) {
        double x = cluster_dV[cluster](0,0);
        double y = cluster_dV[cluster](1,0);
        tempFile << "cluster num: " << cluster << " dv: " << x << " " << y << endl;
    }
    tempFile << "--------------------------------------------------------------" << endl;
    tempfile.close();*/
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
    //qDebug() << "K initialised";
    cameraCount = readCamFile(camera);//contains camera information
    qDebug() << "cam file read";
    readPFiles(camera, cameraCount);//contains the camera matrix of each camera
    qDebug() << "P file read";
    processPLYFile();
    qDebug() << "processed";
    readPatchFile(camera);//contains the the 3D coordinates
    qDebug() << "Patch file read";
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

    QImage image = QImage(camera[0].imgCentreX, camera[0].imgCentreY, QImage::Format_RGB888);
    image.fill(QColor(Qt::black).rgb());

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
    int numPoints = 0;

    QImage image = QImage(camera[0].imgCentreX, camera[0].imgCentreY, QImage::Format_RGB888);
    image.fill(QColor(Qt::black).rgb());

    for(int i = 0; i < MAX_POINTS; i++) {
        if (camera[cameraIndex].image2DPoint[i].x == 0.0 && camera[cameraIndex].image2DPoint[i].y == 0.0) {
            i = MAX_POINTS;
            break;
        }
        else {
            image.setPixel(camera[cameraIndex].image2DPoint[i].x / 2.0, camera[cameraIndex].image2DPoint[i].y / 2.0, qRgb(255,255,255));
            numPoints++;
        }
    }

    QGraphicsScene * scene = new QGraphicsScene();
    ui->graphicsView->setScene(scene);

    scene->addPixmap(QPixmap::fromImage(image));
}

void MainWindow::on_resetButton_clicked()
{
    qApp->quit();
    QProcess::startDetached(qApp->arguments()[0], qApp->arguments());
}

//display interpolated image when interpolate button is clicked
void MainWindow::showIImage(int numOfPoints)
{
    QImage image = QImage(camera[0].imgCentreX, camera[0].imgCentreY, QImage::Format_RGB32);
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
void MainWindow::interpolate_frames()
{
    int c1_idx = (ui->cam1Box->currentText()).toInt() - 1;
    int c2_idx = (ui->cam2Box->currentText()).toInt() - 1;
    int start_cam_idx;
    int end_cam_idx;

    int spinnerVal = ui->sliderSpinner->value();
    double t = ui->sliderSpinner->value() / 10.0;

    if (t > 0.5) {
        start_cam_idx = c2_idx;
        end_cam_idx = c1_idx;
        oCamInd = c2_idx;
        t = 1 - t;
    }
    else {
        start_cam_idx = c1_idx;
        end_cam_idx = c2_idx;
        oCamInd = c1_idx;
    }

    MatrixXd RT(3,4);
    MatrixXd xy(2, 1);

    double fL = focalLen(camera[start_cam_idx].focalLen, camera[end_cam_idx].focalLen, t);
    iCamera.K << fL, 0, camera[start_cam_idx].imgCentreX,
                 0, fL, camera[start_cam_idx].imgCentreY,
                 0, 0, 1;

    iCamera.R = interpolateQR(camera[start_cam_idx].qR, camera[end_cam_idx].qR, t);

    iCamera.T = interpolateTranslation(camera[start_cam_idx].T, camera[end_cam_idx].T, t);

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

    QFile ann_data_file("output/data.pts");
    //QFile points_data_file("output/points_data/" + QString::number(spinnerVal) + ".txt");
    ann_data_file.open(QIODevice::WriteOnly | QIODevice::Truncate);
    //points_data_file.open(QIODevice::WriteOnly | QIODevice::Truncate);
    QTextStream ann_data(&ann_data_file);
    //QTextStream points_data(&points_data_file);
    for (int i = 0; i < numOfPoint; i++) {
        if (!(iCamera.p3DPoint[i].x == 0 && iCamera.p3DPoint[i].y == 0 && iCamera.p3DPoint[i].z == 0)) {
            Vector4d p3D;
            Vector3d p2D;
            p3D << camera[start_cam_idx].p3DPoint[i].x, camera[start_cam_idx].p3DPoint[i].y, camera[start_cam_idx].p3DPoint[i].z, 1;
            p2D = iCamera.P * p3D;
            //reprojection error (x/z - (1 + r2) mx, y /z - (1 + r2) my)
            iCamera.image2DPoint[i].x = p2D(0, 0) / p2D(2, 0);
            iCamera.image2DPoint[i].y = p2D(1, 0) / p2D(2, 0);
            iCamera.image2DPoint[i].pointRGB = camera[start_cam_idx].image2DPoint[i].pointRGB;


            //inerpolate by first and last frame
            /**/

            if (method == QString::fromStdString("First Last Frame")) {
                xy(0, 0) = iCamera.image2DPoint[i].x - camera[start_cam_idx].image2DPoint[i].x;
                xy(1, 0) = iCamera.image2DPoint[i].y - camera[start_cam_idx].image2DPoint[i].y;
            }
            else {
                //improved interpolate by previous frame
                /**/
                if (spinnerVal == 0 || spinnerVal == 10) {
                    xy << 0, 0;
                    temp_2D_points[i].x = iCamera.image2DPoint[i].x;
                    temp_2D_points[i].y = iCamera.image2DPoint[i].y;
                }
                else if (spinnerVal == 1 || spinnerVal == 9){
                    xy(0, 0) = iCamera.image2DPoint[i].x - camera[start_cam_idx].image2DPoint[i].x;
                    xy(1, 0) = iCamera.image2DPoint[i].y - camera[start_cam_idx].image2DPoint[i].y;
                }
                else{
                    xy(0, 0) = iCamera.image2DPoint[i].x - temp_2D_points[i].x;
                    xy(1, 0) = iCamera.image2DPoint[i].y - temp_2D_points[i].y;
                }
                temp_2D_points[i].x = iCamera.image2DPoint[i].x;
                temp_2D_points[i].y = iCamera.image2DPoint[i].y;
                /**/
            }

            /**/

            dV[i] = xy;
        }
        else {
            iCamera.image2DPoint[i].x = 10000;
            iCamera.image2DPoint[i].y = 10000;

            //improved interpolate by previous frame
            /**/

            temp_2D_points[i].x = iCamera.image2DPoint[i].x;
            temp_2D_points[i].y = iCamera.image2DPoint[i].y;

            /**/

            xy(0, 0) = 10000;
            xy(1, 0) = 10000;
            dV[i] = xy;
        }
        ann_data << iCamera.image2DPoint[i].x << " " << iCamera.image2DPoint[i].y << endl;
        //points_data << iCamera.image2DPoint[i].x << " " << iCamera.image2DPoint[i].y << endl;
    }
    //INTERPOLATION BY CLUSTERING
    /**/
    if (clustering == QString::fromStdString("Yes")) {
        cluster_all_points(numOfPoint);
    }
    /**/
    showIImage(numOfPoint);
}

//match button function
//find all the matching 3D points between two cameras
void MainWindow::on_matchButton_clicked()
{
    disable_all_ui();
    method = ui->methodBox->currentText();
    clustering = ui->clusterBox->currentText();
    writeQueryToFile(camera[0].imgCentreX * 2, camera[0].imgCentreY * 2);
    writeQueryToFileImproved(camera[0].imgCentreX * 2, camera[0].imgCentreY * 2);
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

    for (int i = 10; i > 5; i--) {
        QElapsedTimer timer;
        timer.start();
        ui->sliderSpinner->setValue(i);
        qDebug() << "Interpolation runtime of frame " << i << ": " << timer.elapsed() * 0.001 << "\n";
    }
    for (int i = 0; i <= 5; i++) {
        QElapsedTimer timer;
        timer.start();
        ui->sliderSpinner->setValue(i);
        qDebug() << "Interpolation runtime of frame " << i << ": " << timer.elapsed() * 0.001 << "\n";
    }
    enable_all_ui();
}

void MainWindow::on_selectButton_clicked()
{
    ui->matchButton->setEnabled(true);
    ui->full_interpolation->setEnabled(true);
    ui->interpolateSlider->setEnabled(false);
    ui->sliderSpinner->setEnabled(false);
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
    scene->addPixmap(image.scaled(camera[0].imgCentreX, camera[0].imgCentreY));

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

void MainWindow::getRGBVal()
{
    QImage image = QImage(camera[0].imgCentreX, camera[0].imgCentreY, QImage::Format_RGB32);;
    int frame = ui->sliderSpinner->value();
    if (clustering == QString::fromStdString("Yes")) {
        image = pixelMappingImproved(method, clustering, clusterRGB, numOfPoint, dV, oCamInd, camera, frame, cluster_dV, point_cluster_num, cluster_num);
    }
    else {
        image = pixelMapping(method, numOfPoint, dV, oCamInd, camera, frame);
    }
    ui->interpolateSlider->setValue(frame);
}


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->methodBox->addItem("First Last Frame");
    ui->methodBox->addItem("Previous Frame");
    ui->clusterBox->addItem("Yes");
    ui->clusterBox->addItem("No");
    ui->showButton->setEnabled(false);
    ui->showButton2->setEnabled(false);
    ui->cameraBox->setEnabled(false);
    ui->cam1Box->setEnabled(false);
    ui->cam2Box->setEnabled(false);
    ui->selectButton->setEnabled(false);
    ui->matchButton->setEnabled(false);
    ui->interpolateSlider->setEnabled(false);
    ui->interpolateSlider->setRange(0, 10);
    ui->sliderSpinner->setVisible(false);
    ui->sliderSpinner->setRange(0, 10);
    ui->full_interpolation->setEnabled(false);
    ui->resetButton->setEnabled(false);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_interpolateSlider_valueChanged()
{
    int frame = ui->interpolateSlider->value();
    QString filePath = QString("output/visualize/frame_" + QString::number(frame) + ".jpg");
    QPixmap image(filePath);
    QGraphicsScene * scene = new QGraphicsScene();
    ui->colorView->setScene(scene);
    scene->addPixmap(image.scaled(camera[0].imgCentreX, camera[0].imgCentreY));
}

void MainWindow::on_sliderSpinner_valueChanged()
{
    interpolate_frames();
    getRGBVal();
}

void MainWindow::on_cam2Box_currentTextChanged()
{
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
    ui->matchButton->setEnabled(true);
}

void MainWindow::on_full_interpolation_clicked()
{
    disable_all_ui();

    int cam_visited[MAX_CAM_NUM];

    for (int cam = 0; cam < MAX_CAM_NUM; cam++) {
        cam_visited[cam] = 0;
    }

    int first_cam = (ui->cam1Box->currentText()).toInt();
    int current_cam = (ui->cam1Box->currentText()).toInt();
    int next_cam = (ui->cam2Box->currentText()).toInt();
    int frame_count = 0;
    int next_cam_index = ui->cam1Box->findText(QString::number(next_cam));
    on_selectButton_clicked();
    cam_visited[current_cam] = 1;

    while (1) {
        on_selectButton_clicked();
        current_cam = (ui->cam1Box->currentText()).toInt();
        next_cam = (ui->cam2Box->currentText()).toInt();

        if (next_cam == first_cam) {
            break;
        }
        else {
            on_matchButton_clicked();

            for (int num = 0; num <= 10; num++) {
                QFile::copy("output/visualize/frame_" + QString::number(num) + ".jpg", "output/visualize/full_interpolation/" + QString::number(frame_count) + ".jpg");
                frame_count += 1;
            }
            cam_visited[current_cam] = 1;
            next_cam_index = ui->cam1Box->findText(QString::number(next_cam));
            ui->cam1Box->setCurrentIndex(next_cam_index);
        }
    }
    enable_all_ui();
    qDebug() << "ended\n";
}

void MainWindow::on_cam1Box_currentIndexChanged(int index)
{
    ui->cam2Box->setEnabled(false);
    ui->matchButton->setEnabled(false);
    ui->full_interpolation->setEnabled(false);
}
