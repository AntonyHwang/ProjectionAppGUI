#ifndef CALCULATION_H
#define CALCULATION_H

#include "mainwindow.h"
#include "armadillo"

using namespace arma;

mat rToQR(mat R);
mat interpolateQR(mat qR1, mat qR2, double t);
mat qRToRotation(mat iqR);
eulerAngles computeEuler(mat R);
eulerAngles interpolateEuler(eulerAngles a, eulerAngles b, double h);
mat interpolateTranslation(mat a, mat b, double h);
double focalLen(double a, double b, double h);

#endif // CALCULATION_H
