#pragma once
#include <iostream>
#include "common_expr.hpp"
const int MAX_N = 100;

class dh{
public:
    unsigned int n;
    vector<double> param[MAX_N]; //{{a_0, d_0, alpha_0}, {a_1, d_1, alpha_1}, ...}
    tform h(vd param, double theta);
};

dh make_puma(double l1, double l2, double l3);
