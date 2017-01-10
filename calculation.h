#include <Geometry>
#include <Dense>
#include <iostream>
#include <fstream>
#include <sstream>
#include <QDebug>
#include <ANN/ANN.h>

using namespace std;
using namespace Eigen;

Vector4d rToQR(Matrix3d R);
Vector4d interpolateQR(Vector4d qR1, Vector4d qR2, double t);
Matrix3d qRToRotation(Vector4d iqR);
Vector3d interpolateTranslation(Vector3d a, Vector3d b, double h);
double focalLen(double a, double b, double h);
bool readPt(istream &in, ANNpoint p, int dim);
void printPt(ostream &out, ANNpoint p, int dim);
QImage pixelMapping(int numOfPoint, MatrixXd dV[], int x, int y, int camIndex);
