#include "calculation.h"
#include <QDebug>
#include <QFile>
#include <QElapsedTimer>

using namespace std;

Matrix3d interpolateQR(Vector4d cam1_qR, Vector4d cam2_qR, double t) {
    Quaterniond qR1, qR2, iqR;
    qR1 = {cam1_qR(0, 0), cam1_qR(1, 0), cam1_qR(2, 0), cam1_qR(3, 0)};
    qR2 = {cam2_qR(0, 0), cam2_qR(1, 0), cam2_qR(2, 0), cam2_qR(3, 0)};
    iqR = qR1.slerp(t, qR2);
    return iqR.toRotationMatrix();
}

Vector3d interpolateTranslation(Vector3d a, Vector3d b, double t) {
    Vector3d iT;
    iT = a + t * (b - a);

    return iT;
}

double focalLen(double a, double b, double t) {
    double tFocalLen;
    if (a < b) {
        tFocalLen = a + t * abs(b - a);
    }
    else if (a > b) {
        tFocalLen = a - t * abs(b - a);
    }
    else {
        return a;
    }
    return tFocalLen;
}

bool readPt(istream &in, ANNpoint p, int dim)			// read point (false on EOF)
{
    for (int i = 0; i < dim; i++) {
        if(!(in >> p[i])) return false;
    }
    return true;
}

QImage pixelMapping(int numOfPoint, MatrixXd dV[], int camIndex, cameraInfo camera[]) {
    QElapsedTimer timer;
    timer.start();
    QString fileName = "output/dv.txt";
    double wi = 0;
    double wiSum = 0;
    Vector2d dp;

    int	k = 80;
    int	dim	= 2;
    double eps = 0;
    int maxPts = numOfPoint;

    istream* dataIn	= NULL;
    istream* queryIn = NULL;

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

    static ifstream dataStream;
    static ifstream queryStream;

    dataStream.open("output/data.pts");
    dataIn = &dataStream;
    queryStream.open("output/query.pts");
    queryIn = &queryStream;

    nPts = 0;
    //cout << "Data Points:\n";

    while (nPts < maxPts && readPt(*dataIn, dataPts[nPts], dim)) {
        nPts++;
    }

    kdTree = new ANNkd_tree(
                    dataPts,
                    nPts,
                    dim);


    QFile file(fileName);
    file.open(QIODevice::WriteOnly | QIODevice::Truncate);
    QTextStream dVFile(&file);

    int numofpoint = 0;
    while (readPt(*queryIn, queryPt, dim)) {
        Vector2d widiSum;
        widiSum << 0, 0;
        wiSum = 0;
        //qDebug() << "Query point: ";
        //printPt(cout, queryPt, dim);

        kdTree->annkSearch(
                queryPt,
                k,
                nnIdx,
                dists,
                eps);

        //qDebug() << "\tNN:\tIndex\tDistance\n";
        /*int points = 0;
        for (int i = 0; i < k; i++) {
            dists[i] = sqrt(dists[i]);
            if (dists[i] <= 300) {
                points++;
                wi = exp(-0.008 * dists[i]);
                wiSum += wi;
                widiSum += wi * dV[nnIdx[i]];
            }
            //qDebug() << "\t" << i << "\t" << nnIdx[i] << "\t" << dists[i] << "\n";
            //qDebug() << "\t" << dists[i] << "\n";
            //qDebug() << wi << "\n";
        }*/

        int points = 0;
        double maxDist = 200;
        double ld = 2.5 / maxDist;
        for (int i = 0; i < k; i++) {
            dists[i] = sqrt(dists[i]);
            if (dists[i] <= maxDist) {
                points++;
                wi = exp(-ld * dists[i]);
                if (wi < 0.1) {
                    break;
                }
                wiSum += wi;
                widiSum += wi * dV[nnIdx[i]];
            }
            //qDebug() << "\t" << i << "\t" << nnIdx[i] << "\t" << dists[i] << "\n";
            //qDebug() << "\t" << dists[i] << "\n";
            //qDebug() << wi << "\n";
        }
        //qDebug() << "sampled points: " << points << "\n";
        if (points == 0) {
            dVFile << 10000 << " " << 10000 << endl;
        }
        else {
            dp(0,0) = widiSum(0,0) / wiSum;
            dp(1,0) = widiSum(1,0) / wiSum;
            //qDebug() << dp(0,0) << "\n";
            dVFile << dp(0,0) << " " << dp(1,0) << endl;
        }
        numofpoint++;
    }
    //qDebug() << "numofpoint: " << numofpoint << "\n";
    file.close();
    dataStream.close();
    queryStream.close();
    delete [] nnIdx;
    delete [] dists;
    delete kdTree;
    annClose();
    qDebug() << "pixelMapping runtime: " << timer.elapsed() * 0.001 << "\n";
    return showRGBImg(camIndex, camera);
}

QImage pixelMappingImproved(int numOfPoint, MatrixXd dV[], int camIndex, cameraInfo camera[], int frame, MatrixXd cluster_dV[], int point_cluster_num[], int cluster_num) {
    //qDebug() << "reached\n";
    QElapsedTimer timer;
    timer.start();
    QString fileName = "output/dv.txt";
    double wi = 0;
    double wiSum = 0;
    Vector2d dp;

    int	k = 20;
    int	dim	= 2;
    double eps = 0;
    int maxPts = numOfPoint;

    istream* dataIn	= NULL;
    istream* queryIn = NULL;

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

    static ifstream dataStream;
    static ifstream queryStream;

    dataStream.open("output/data.pts");
    dataIn = &dataStream;
    queryStream.open("output/queryimp.pts");
    queryIn = &queryStream;

    nPts = 0;
    //cout << "Data Points:\n";

    while (nPts < maxPts && readPt(*dataIn, dataPts[nPts], dim)) {
        nPts++;
    }

    kdTree = new ANNkd_tree(
                    dataPts,
                    nPts,
                    dim);


    QFile file(fileName);
    file.open(QIODevice::WriteOnly | QIODevice::Truncate);
    QTextStream dVFile(&file);

    int numofpoint = 0;
    while (readPt(*queryIn, queryPt, dim)) {
        //Vector2d widiSum;
        //widiSum << 0, 0;
        //wiSum = 0;
        //qDebug() << "Query point: ";
        //printPt(cout, queryPt, dim);

        kdTree->annkSearch(
                queryPt,
                k,
                nnIdx,
                dists,
                eps);

        //qDebug() << "\tNN:\tIndex\tDistance\n";
        int points = 0;
        double maxDist = 150;
        //CODE FOR INETRPOLATION BY AVERAGE TRANSLATION OF NEARBY POINTS
        /*double ld = 2.5 / maxDist;
        for (int i = 0; i < k; i++) {
            dists[i] = sqrt(dists[i]);
            if (dists[i] <= maxDist) {
                points++;
                wi = exp(-ld * dists[i]);
                if (wi < 0.1) {
                    break;
                }
                wiSum += wi;
                //wiSum = i;
                widiSum += wi * dV[nnIdx[i]];
            }
            //qDebug() << "\t" << i << "\t" << nnIdx[i] << "\t" << dists[i] << "\n";
            //qDebug() << "\t" << dists[i] << "\n";
            //qDebug() << wi << "\n";
        }*/
        //CODE FOR INETRPOLATION BY CLUSTERS
        int cluster_count[cluster_num];
        int point_cluster = 0;
        for (int cluster = 0; cluster < cluster_num; cluster++) {
            cluster_count[cluster] = 0;
        }
        for (int i = 0; i < k; i++) {
            dists[i] = sqrt(dists[i]);
            if (dists[i] <= maxDist) {
                points++;
                point_cluster = point_cluster_num[nnIdx[i]];
                cluster_count[point_cluster]++;
                if (point_cluster_num[nnIdx[i]] != 0) {
                    //qDebug() << nnIdx[i] << "\n";
                }
            }
        }

        //qDebug() << "sampled points: " << points << "\n";
        if (points == 0) {
            dVFile << 10000 << " " << 10000 << endl;
        }
        else {
            int point_cluster = 1;
            int largest_num_point = 0;
            for (int cluster = 1; cluster <= cluster_num; cluster++) {
                if (cluster_count[cluster] > largest_num_point) {
                    largest_num_point = cluster_count[cluster];
                    point_cluster = cluster;
                }
            }
            if (largest_num_point != 0) {
                double x = cluster_dV[point_cluster](0,0);
                double y = cluster_dV[point_cluster](1,0);
                MatrixXd temp_dp(2,1);
                temp_dp << x, y;
                dp = temp_dp;
                //qDebug() << dp(0,0) << "\n";
                dVFile << dp(0,0) << " " << dp(1,0) << endl;
            }
            else {
                dVFile << 0 << " " << 0 << endl;
            }
            //dp(0,0) = cluster_dV[point_cluster](0,0);
            //dp(1,0) = cluster_dV[point_cluster](1,0);
            //qDebug() << dp(0,0) << "\n";
        }
        numofpoint++;
    }
    //qDebug() << "numofpoint: " << numofpoint << "\n";
    file.close();
    dataStream.close();
    queryStream.close();
    delete [] nnIdx;
    delete [] dists;
    delete kdTree;
    annClose();
    qDebug() << "pixelMapping runtime: " << timer.elapsed() * 0.001 << "\n";
    return showRGBImgImproved(camIndex, camera, frame);
}
