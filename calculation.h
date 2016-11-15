#ifndef CALCULATION_H
#define CALCULATION_H

#include "mainwindow.h"
#include "armadillo"

using namespace arma;

eulerAngles computeEuler(mat R);
eulerAngles interpolateEuler(eulerAngles a, eulerAngles b);

#endif // CALCULATION_H
