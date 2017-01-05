#include "calculation.h"

using namespace std;

Vector4d rToQR(Matrix3d R) {
    double R_00 = R(0,0);
    double R_01 = R(0,1);
    double R_02 = R(0,2);
    double R_10 = R(1,0);
    double R_11 = R(1,1);
    double R_12 = R(1,2);
    double R_20 = R(2,0);
    double R_21 = R(2,1);
    double R_22 = R(2,2);

    double w = sqrt(1.0 + R_00 + R_11 + R_22) / 2.0;
    double w4 = (4.0 * w);
    double x = (R_21 - R_12) / w4 ;
    double y = (R_02 - R_20) / w4 ;
    double z = (R_10 - R_01) / w4 ;

    Vector4d qR;
    qR << w, x, y, z;
    return qR;
}


Vector4d interpolateQR(Vector4d qR1, Vector4d qR2, double t) {
    double w1 = qR1(0, 0);
    double x1 = qR1(1, 0);
    double y1 = qR1(2, 0);
    double z1 = qR1(3, 0);

    double w2 = qR1(0, 0);
    double x2 = qR1(1, 0);
    double y2 = qR1(2, 0);
    double z2 = qR1(3, 0);

    Vector4d iqR;
    // calculate angle difference
    double cosHalfAng = qR1.dot(qR2);
    if (fabs(cosHalfAng) >= 1.0) {
        return qR1;
    }
    if (cosHalfAng < 0) {
        w2 = -w2;
        x2 = -x2;
        y2 = -y2;
    }

    double HalfAng = acos(cosHalfAng);
    double sinHalfAng = sqrt(1.0 - cosHalfAng * cosHalfAng);
    if (fabs(sinHalfAng) < 0.001) {
        iqR << w1 * 0.5 + w2 * 0.5,
               x1 * 0.5 + x2 * 0.5,
               y1 * 0.5 + y2 * 0.5,
               z1 * 0.5 + z2 * 0.5;
        return iqR;
    }
    double ratioA = sin((1 - t) * HalfAng) / sinHalfAng;
    double ratioB = sin(t * HalfAng) / sinHalfAng;

    iqR << w1 * ratioA + w2 * ratioB,
           x1 * ratioA + x2 * ratioB,
           y1 * ratioA + y2 * ratioB,
           z1 * ratioA + z2 * ratioB;
    return iqR;
}

Matrix3d qRToRotation(Vector4d iqR) {
    Matrix3d rotation;

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

    rotation << R00, R01, R02,
                R10, R11, R12,
                R20, R21, R22;
    return rotation;
}

Vector3d interpolateTranslation(Vector3d a, Vector3d b, double h) {
    Vector3d iT;
    iT = a + h * (b - a);

    return iT;
}

double focalLen(double a, double b, double h) {
    double hFocalLen;
    hFocalLen = a + h * abs(b - a);

    return hFocalLen;
}

