#include <Geometry>
#include <Dense>
#include <iostream>
#include <fstream>
#include <sstream>
#include <QDebug>
#include <ANN/ANN.h>
#include "iodata.h"

using namespace std;
using namespace Eigen;

Matrix3d interpolateQR(Vector4d cam1_qR, Vector4d cam2_qR, double t);
Vector3d interpolateTranslation(Vector3d a, Vector3d b, double t);
double focalLen(double a, double b, double t);
bool readPt(istream &in, ANNpoint p, int dim);
void printPt(ostream &out, ANNpoint p, int dim);
QImage pixelMapping(int numOfPoint, MatrixXd dV[], int camIndex, cameraInfo camera[]);
QImage pixelMappingImproved(int numOfPoint, MatrixXd dV[], int camIndex, cameraInfo camera[], int frame);
