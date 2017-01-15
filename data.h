/*#include "armadillo"
#include <Geometry>
#include <Dense>

using namespace arma;
using namespace Eigen;

const int MAX_CAM_NUM = 200;
const int MAX_POINTS = 150000;

double imageCentreX = 414.0;
double imageCentreY = 646.0;

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

struct cameraInfo {
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
