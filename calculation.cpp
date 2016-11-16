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
    double R_12 = R(0,1);
    double R_13 = R(0,2);

    //2 sets of possible euler angles
    eulerAngles eA;
    if (R_31 != 1.0 || R_31 != -1.0) {
        eA.theta_1 = -asin(R_31);
        eA.psi_1 = atan2(R_32/cos(eA.theta_1), R_33/cos(eA.theta_1));
        eA.phi_1 = atan2(R_21/cos(eA.theta_1), R_11/cos(eA.theta_1));

        eA.theta_2 = M_PI - eA.theta_1;
        eA.psi_2 = atan2(R_32/cos(eA.theta_2), R_33/cos(eA.theta_2));
        eA.phi_2 = atan2(R_21/cos(eA.theta_2), R_11/cos(eA.theta_2));
    }
    else {
        eA.phi_1 = 0.0;
        if (R_31 == -1) {
            eA.theta_1 = M_PI / 2;
            eA.psi_1 = eA.phi_1 + atan2(R_12, R_13);
        }
        else {
            eA.theta_1 = -M_PI / 2;
            eA.psi_1 = -eA.phi_1 + atan2(-R_12, -R_13);
        }
    }

    return eA;
}

eulerAngles interpolateEuler(eulerAngles a, eulerAngles b, double h) {
    eulerAngles hEA;
    hEA.theta_1 = a.theta_1 + h * (b.theta_1 - a.theta_1);
    hEA.psi_1 = a.psi_1 + h * (b.psi_1 - a.psi_1);
    hEA.phi_1 = a.phi_1 + h * (b.phi_1 - a.phi_1);

    hEA.theta_2 = a.theta_2 + h * (b.theta_2 - a.theta_2);
    hEA.psi_2 = a.psi_2 + h * (b.psi_2 - a.psi_2);
    hEA.phi_2 = a.phi_2 + h * (b.phi_2 - a.phi_2);

    return hEA;
}

mat interpolateTranslation(mat a, mat b, double h) {
    mat iT;
    iT = a + h * (b - a);

    return iT;
}

double focalLen(double a, double b, double h) {
    double hFocalLen;
    hFocalLen = a + h * (b - a);

    return hFocalLen;
}



