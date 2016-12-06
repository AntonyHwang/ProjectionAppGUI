#include "calculation.h"
#include "mainwindow.h"
#include <cstdlib>						// C standard library
#include <cstdio>						// C I/O (for sscanf)
#include <cstring>						// string manipulation
#include <fstream>						// file I/O
#include <QDebug>

#include "armadillo"
#include <ANN/ANN.h>
#include <math.h>

using namespace std;
using namespace arma;

mat rToQR(mat R) {
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

    mat qR;
    qR << w << endr
       << x << endr
       << y << endr
       << z << endr;
    return qR;
}


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

/*eulerAngles computeEuler(mat R) {
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
    if (R_20 != 1.0 || R_20 != -1.0) {
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
    }
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
}*/

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

bool readPt(istream &in, ANNpoint p, int dim)			// read point (false on EOF)
{
    for (int i = 0; i < dim; i++) {
        if(!(in >> p[i])) return false;
    }
    return true;
}

void printPt(ostream &out, ANNpoint p, int dim)			// print point
{
    out << "(" << p[0];
    for (int i = 1; i < dim; i++) {
        out << ", " << p[i];
    }
    out << ")\n";
}

void getRGBVal(int numPoint) {
    int	k = 20;                      // number of nearest neighbors
    int	dim	= 2;                    // dimension
    double eps = 0;                 // error bound
    int maxPts = numPoint;			// maximum number of data points

    istream* dataIn	= NULL;			// input for data points
    istream* queryIn = NULL;        // input for query points

    int					nPts;					// actual number of data points
    ANNpointArray		dataPts;				// data points
    ANNpoint			queryPt;				// query point
    ANNidxArray			nnIdx;					// near neighbor indices
    ANNdistArray		dists;					// near neighbor distances
    ANNkd_tree*			kdTree;					// search structure

    queryPt = annAllocPt(dim);					// allocate query point
    dataPts = annAllocPts(maxPts, dim);			// allocate data points
    nnIdx = new ANNidx[k];						// allocate near neigh indices
    dists = new ANNdist[k];						// allocate near neighbor dists

    static ifstream dataStream;					// data file stream
    static ifstream queryStream;				// query file stream

    dataStream.open("data.pts");
    dataIn = &dataStream;
    queryStream.open("query.pts");
    queryIn = &queryStream;

    nPts = 0;									// read data points
    cout << "Data Points:\n";

    //qDebug() << "Reached\n";

    while (nPts < maxPts && readPt(*dataIn, dataPts[nPts], dim)) {
        printPt(cout, dataPts[nPts], dim);
        nPts++;
    }

    kdTree = new ANNkd_tree(					// build search structure
                    dataPts,					// the data points
                    nPts,						// number of points
                    dim);						// dimension of space

    while (readPt(*queryIn, queryPt, dim)) {			// read query points
        cout << "Query point: ";				// echo query point
        printPt(cout, queryPt, dim);

        kdTree->annkSearch(						// search
                queryPt,						// query point
                k,								// number of near neighbors
                nnIdx,							// nearest neighbors (returned)
                dists,							// distance (returned)
                eps);							// error bound

        cout << "\tNN:\tIndex\tDistance\n";
        for (int i = 0; i < k; i++) {			// print summary
            dists[i] = sqrt(dists[i]);			// unsquare distance
            cout << "\t" << i << "\t" << nnIdx[i] << "\t" << dists[i] << "\n";
        }
    }
    delete [] nnIdx;							// clean things up
    delete [] dists;
    delete kdTree;
    annClose();									// done with ANN
}

