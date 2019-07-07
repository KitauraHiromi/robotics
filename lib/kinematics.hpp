#pragma once
#include "common_expr.hpp"
#include "dh.hpp"
#include <random>
#include <fstream>

void init_random();
tform fk(dh, vd);
vd puma_ik(dh, tform);
void zyz_eular(tform, double&, double&, double&);
void test_fk();