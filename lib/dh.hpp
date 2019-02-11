#define _USE_MATH_DEFINES
#include <iostream>
#include<cmath>
#include "Eigen/Core"
#include <vector>
#define DEG(rad) rad*180./M_PI
#define RAD(deg) deg*M_PI/180.
#define MAX_N 100
using namespace std;
using namespace Eigen;
typedef Matrix<double, 4, 4> tform; // 同次変換行列
typedef vector<double> vd;

class dh
{
public:
    dh();
    ~dh();
    unsigned int n;
    vector<double> param[MAX_N]; //{{a_0, d_0, alpha_0}, {a_1, d_1, alpha_1}, ...}
    tform h(vd param, double theta);
};

dh::dh(){}
dh::~dh(){}

tform dh::h(vd param, double theta){
    tform ret;
    ret(0, 0) = cos(theta);
    ret(0, 1) = -sin(theta)*cos(param[2]);
    ret(0, 2) = sin(theta)*sin(param[2]);
    ret(0, 3) = param[0]*cos(theta);
    ret(1, 0) = sin(theta);
    ret(1, 1) = cos(theta)*cos(param[2]);
    ret(1, 2) = -cos(theta)*sin(param[2]);
    ret(1, 3) = param[0]*sin(theta);
    ret(2, 0) = 0;
    ret(2, 1) = sin(param[0]);
    ret(2, 2) = cos(param[0]);
    ret(2, 3) = param[1];
    ret(3, 0) = 0;
    ret(3, 1) = 0;
    ret(3, 2) = 0;
    ret(3, 3) = 1;
    return ret;
}

dh make_puma(double l1, double l2, double l3){
    dh puma;
    puma.n = 6;
    for(unsigned int i=0; i<puma.n; i++) puma.param[i].resize(3);
    puma.param[0][0] = 0; puma.param[0][1] = -RAD(90); puma.param[0][2] = 0;
    puma.param[1][0] = l1; puma.param[1][1] = RAD(0); puma.param[1][2] = 0;
    puma.param[2][0] = 0; puma.param[2][1] = RAD(180); puma.param[2][2] = 0;
    puma.param[3][0] = 0; puma.param[3][1] = -RAD(90); puma.param[3][2] = l2;
    puma.param[4][0] = 0; puma.param[4][1] = RAD(90); puma.param[4][2] = 0;
    puma.param[5][0] = 0; puma.param[5][1] = RAD(0); puma.param[5][2] = l3;
    return puma;
}
