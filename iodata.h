#include <string>
#include <QString>
#include <Geometry>
#include <Dense>
#include <QImage>
//#include "data.h"

using namespace std;
using namespace Eigen;

const int MAX_CAM_NUM = 200;
const int MAX_POINTS = 150000;

struct RGB {
    double r;
    double g;
    double b;
};

struct point2D {
    double x = 0;
    double y = 0;
    RGB pointRGB;
};

struct point3D {
    double x;
    double y;
    double z;
};

/*struct cameraInfo {
    double focalLen;
    mat K;//IntrinsicMatrix 3x3
    mat T;//Translation 3x1
    mat C;//CameraPosition 3x1
    mat aR;//AxisAngleR 3x1
    mat qR;//QuaternionR 4x1
    mat R;//3x3R
    double rD;//RadialDistortion
    mat P;//CameraMatrix
    point3D p3DPoint [MAX_POINTS];
    point2D image2DPoint [MAX_POINTS];//2D points
};*/

struct cameraInfo {
    double focalLen;
    Matrix3d K;//IntrinsicMatrix 3x3
    Vector3d T;//Translation 3x1
    Vector3d C;//CameraPosition 3x1
    Vector3d aR;//AxisAngleR 3x1
    Vector4d qR;//QuaternionR 4x1
    Matrix3d R;//3x3R
    double rD;//RadialDistortion
    MatrixXd P;//CameraMatrix
    point3D p3DPoint [MAX_POINTS];
    point2D image2DPoint [MAX_POINTS];//2D points
};

double stringToDouble(string s);
bool checkFileExist(string fileName);
string getFileName(string fileType, int index);
int readCamFile(struct cameraInfo camera[]);
void readPFiles(cameraInfo camera[], int cameraCount);
void processPLYFile();
void store2DPoint (cameraInfo camera[], int camIndex, double x, double y, RGB point_RGB, Vector4d point_3D);
void calculate2DPoint(cameraInfo camera[], int index, Vector3d point_3D, RGB point_RGB);
void readPatchFile(cameraInfo camera[]);
void writeQueryToFile(int maxX, int maxY);
void writeToFile(int mode, QString fileName, double x, double y);
QImage showRGBImg(int camIndex);
