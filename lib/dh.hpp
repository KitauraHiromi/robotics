#pragma once
#include <iostream>
#include "common_expr.hpp"
const int MAX_N = 100;

class dh{
public:
    unsigned int n;
    vector<double> param[MAX_N]; //{{a0, alpha0, d0}, {a1, alpha1, d1}, ...}
    tform h(vd param, double theta);
};

dh make_puma(double l1, double l2, double l3);
