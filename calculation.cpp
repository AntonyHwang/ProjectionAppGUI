#include "calculation.h"
#include "mainwindow.h"
#include "armadillo"
#include <math.h>

using namespace arma;

mat interpolateQR(mat qR1, mat qR2, double t) {
    double w1 = qR1(0, 0);
    double x1 = qR1(1, 0);
    double y1 = qR1(2, 0);
    double z1 = qR1(3, 0);

    double w2 = qR1(0, 0);
    double x2 = qR1(1, 0);
    double y2 = qR1(2, 0);
    double z2 = qR1(3, 0);

    mat iqR;
    // calculate angle difference
    double cosHalfAng = dot(qR1, qR2);
    if (fabs(cosHalfAng) >= 1.0) {
        return qR1;
    }
    double HalfAng = acos(cosHalfAng);
    double sinHalfAng = sqrt(1.0 - cosHalfAng * cosHalfAng);
    if (fabs(sinHalfAng) < 0.001) {
        iqR << w1 * 0.5 + w2 * 0.5 << endr
            << x1 * 0.5 + x2 * 0.5 << endr
            << y1 * 0.5 + y2 * 0.5 << endr
            << z1 * 0.5 + z2 * 0.5;
        return iqR;
    }
    double ratioA = sin((1 - t) * HalfAng) / sinHalfAng;
    double ratioB = sin(t * HalfAng) / sinHalfAng;

    iqR << w1 * ratioA + w2 * ratioB << endr
        << x1 * ratioA + x2 * ratioB << endr
        << y1 * ratioA + y2 * ratioB << endr
        << z1 * ratioA + z2 * ratioB;
    return iqR;
}

mat qRToRotation(mat iqR) {
    mat rotation;

    double w = iqR(0, 0);
    double x = iqR(1, 0);
    double y = iqR(2, 0);
    double z = iqR(3, 0);

    double R00 = 1 - (2 * y * y) - (2 * z * z);
    double R01 = (2 * x * y) - (2 * z * w);
    double R02 = (2 * x * z) + (2 * y * w);
    double R10 = (2 * x * y) + (2 * z * w);
    double R11 = 1 - (2 * x * x) - (2 * z * z);
    double R12 = (2 * y * z) - (2 * x * w);
    double R20 = (2 * x * z) - (2 * y * w);
    double R21 = (2 * y * z) + (2 * x * w);
    double R22 = 1 - (2 * x * x) - (2 * y * y);

    rotation << R00 << R01 << R02 << endr
             << R10 << R11 << R12 << endr
             << R20 << R21 << R22;
    return rotation;
}

eulerAngles computeEuler(mat R) {
    double R_00 = R(0,0);
    double R_01 = R(0,1);
    double R_02 = R(0,2);
    double R_10 = R(1,0);
    double R_11 = R(1,1);
    double R_12 = R(1,2);
    double R_20 = R(2,0);
    double R_21 = R(2,1);
    double R_22 = R(2,2);

    //2 sets of possible euler angles
    eulerAngles eA;
    /*if (R_20 != 1.0 || R_20 != -1.0) {
        eA.theta_x = -asin(R_20);
        eA.theta_y = atan2(R_21/cos(eA.theta_x), R_22/cos(eA.theta_x));
        eA.theta_z = atan2(R_10/cos(eA.theta_x), R_00/cos(eA.theta_x));
    }
    else {
        eA.theta_z = 0.0;
        if (R_20 == -1) {
            eA.theta_x = M_PI / 2;
            eA.theta_y = eA.theta_z + atan2(R_01, R_02);
        }
        else {
            eA.theta_x = -M_PI / 2;
            eA.theta_y = -eA.theta_z + atan2(-R_01, -R_02);
        }
    }*/
    //eA.theta_x = atan2(R_12, R_22);
    //double c2 = sqrt(pow(R_00, 2) + pow(R_01, 2));
    //eA.theta_y = atan2(-R_01, c2);
    //double s1 = sin(eA.theta_x);
    //double c1 = cos(eA.theta_x);
    //eA.theta_z = atan2(s1 * R_20 - c1 * R_10, c1 * R_11 - s1 * R_21);

    if (R_10 > 0.999) {
        eA.theta_x = atan2(R_02, R_22);
        eA.theta_y = M_PI / 2;
        eA.theta_z = 0;
        return eA;
    }
    if (R_10 < -0.999) {
        eA.theta_x = atan2(R_02, R_22);
        eA.theta_y = -M_PI / 2;
        eA.theta_z = 0;
        return eA;
    }
    eA.theta_x = atan2(-R_20, R_00);
    eA.theta_z = atan2(-R_12, R_11);
    eA.theta_y = asin(R_10);

    return eA;
}

double correctAngles(double b, double a) {
    double angDiff = b - a;
    if(angDiff > M_PI) {
        return M_PI - 2 * M_PI;
    }
    else if(angDiff < -M_PI) {
        return M_PI + 2 * M_PI;
    }
    else {
        return angDiff;
    }
}

eulerAngles interpolateEuler(eulerAngles a, eulerAngles b, double h) {
    eulerAngles hEA;
    //fix angles
    hEA.theta_x = a.theta_x + h * (correctAngles(b.theta_x, a.theta_x));
    hEA.theta_y = a.theta_y + h * (correctAngles(b.theta_y, a.theta_y));
    hEA.theta_z = a.theta_z + h * (correctAngles(b.theta_z, a.theta_z));

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



