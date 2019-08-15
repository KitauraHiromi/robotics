#include "common_expr.hpp"
#include "dh.hpp"
#include <random>
#include <fstream>

// void init_random(mt19937 mt, uniform_real_distribution<double> th, double lbound, double ubound);
tform fk(dh, vd);
vd puma_ik(dh, tform);
void zyz_eular(tform, double&, double&, double&);
void test_ik();
void test_ik2();
void test_fk();
void zyz_eular(tform h, double& alpha, double& beta, double& gamma);
void xyz_eular(tform h, double& alpha, double& beta, double& gamma);
tform rot_x(double x);
tform rot_y(double y);
tform rot_z(double z);
tform pos_to_tform_xyz_eular(vd pos);
