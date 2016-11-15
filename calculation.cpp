#include "calculation.h"
#include "mainwindow.h"
#include "armadillo"
#include <math.h>

using namespace arma;

eulerAngles computeEuler(mat R) {
    double R_31 = R(2,0);
    double R_32 = R(2,1);
    double R_33 = R(2,2);
    double R_21 = R(1,0);
    double R_11 = R(0,0);

    //2 sets of possible euler angles
    eulerAngles eA;
    eA.theta_1 = -asin(R_31);
    eA.psi_1 = atan2(R_32/cos(eA.theta_1), R_33/cos(eA.theta_1));
    eA.phi_1 = atan2(R_21/cos(eA.theta_1), R_11/cos(eA.theta_1));

    eA.theta_2 = M_PI - eA.theta_1;
    eA.psi_2 = atan2(R_32/cos(eA.theta_2), R_33/cos(eA.theta_2));
    eA.phi_2 = atan2(R_21/cos(eA.theta_2), R_11/cos(eA.theta_2));

    return eA;
}

eulerAngles interpolateEuler(eulerAngles a, eulerAngles b) {
    eulerAngles iE;
    iE.theta_1 = (a.theta_1 + b.theta_1) / 2;
    iE.psi_1 = (a.psi_1 + b.psi_1) / 2;
    iE.phi_1 = (a.phi_1 + b.phi_1) / 2;

    iE.theta_2 = (a.theta_2 + b.theta_2) / 2;
    iE.psi_2 = (a.psi_2 + b.psi_2) / 2;
    iE.phi_2 = (a.phi_2 + b.phi_2) / 2;

    return iE;
}

mat interpolateTranslation(mat a, mat b) {
    mat iT;
    iT(0,0) = b(0,0) - a(0,0);
    iT(1,0) = b(1,0) - a(1,0);
    iT(2,0) = b(2,0) - a(2,0);

    return iT;
}

