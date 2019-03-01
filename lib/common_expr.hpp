#pragma once
#define _USE_MATH_DEFINES
#include "Eigen/Core"
#include <cmath>
#include <vector>
using namespace std;
using namespace Eigen;
using tform = Matrix<double, 4, 4>; // 同次変換行列
using vd = vector<double>;
#define DEG(rad) rad*180./M_PI
#define RAD(deg) deg*M_PI/180.