#include "kinematics.hpp"

mt19937 mt;
uniform_real_distribution<double> th;

void init_random(){
    std::random_device rd;
    mt = mt19937(rd());
    th = uniform_real_distribution<double>(-180, 180);
}

tform fk(dh link, vd theta){
    if(theta.size() != (unsigned int)link.n) throw invalid_argument("dimension must be same");
    tform end_effector_pos = tform::Identity();
    for(unsigned int i=0; i<link.n; i++){
        end_effector_pos *= link.h(link.param[i], theta[i]);
    }
    vd ret(6, 0);
    return end_effector_pos;
}

vd puma_ik(dh link, tform pos){
    vd ret(6, 0);
    double x, y, z, l1, l2, l;
    x = pos(0, 3);
    y = pos(1, 3);
    z = pos(2, 3);
    l1 = link.param[1][0];
    l2 = link.param[3][2];
    l = sqrt(x*x + y*y);

    // θ1, θ2, θ3: defined from ps(xs, ys, zs)
    vd ps(3, 0);
    ps[0] = x - pos(0, 2);
    ps[1] = y - pos(1, 2);
    ps[2] = z - pos(2, 2);

    // debug
    // cout << x << " " << y << " " << z  << " " << l << endl;
    // cout << ps[0] << " " << ps[1] << " " << ps[2] << endl;
    // cout << asin(-1.0 / (2*l1*sqrt(l*l+ps[2]*ps[2])) * (l*l+l1*l1-l2*l2+ps[2]*ps[2])) << endl;
    // cout << atan2(l, ps[2]) << endl;
    // cout << -1.0 / (2*l1*sqrt(l*l+ps[2]*ps[2])) * (l*l+l1*l1-l2*l2+ps[2]*ps[2]) << endl;
    ret[0] = atan2(y, x);
    ret[1] = asin(-1.0 / (2*l1*sqrt(l*l+ps[2]*ps[2])) * (l*l+l1*l1-l2*l2+ps[2]*ps[2])) + atan2(l, ps[2]);
    ret[2] = acos(ps[2]/l2 + l1/l2*sin(ret[1]));

    zyz_eular(pos, ret[3], ret[4], ret[5]);

    return ret;
}

void zyz_eular(tform h, double& alpha, double& beta, double& gamma){
    alpha = atan2(h(1, 2), h(0, 2));
    beta = atan2(h(0, 2)*cos(alpha) + h(1, 2)*sin(alpha), h(2, 2));
    gamma = atan2(-h(1, 1)*sin(alpha) + h(1, 0)*cos(alpha), -h(0, 1)*sin(alpha) + h(1, 1)*cos(alpha)); 
}

void test_fk(){
    cout << "test_fk in" << endl;
    vd theta(6);
    double l1, l2, l3; l1 = l2 = l3 = 1;

    init_random();
    for(unsigned int i=0; i<theta.size(); i++) theta[i] = RAD(th(mt));
    cout << "random initialized" << endl;

    dh puma = make_puma(l1, l2, l3);
    cout << "puma initialized" << endl;

    double s[6], c[6], s12, c12;
    for(unsigned int i=0; i<theta.size(); i++){
        s[i] = sin(theta[i]);
        c[i] = cos(theta[i]);
    }
    s12 = sin(theta[1]+theta[2]); 
    c12 = cos(theta[1]+theta[2]); 
    tform puma_h;
    cout << "puma_h initialized" << endl;


    double t1, t2, t3, t4, t5, t6;
    t1 = c12*(c[3]*c[4]*c[5] - s[3]*s[5]) - s12*s[4]*c[5];
    t2 = s[3]*c[4]*c[5] + c[3]*s[5];
    puma_h(0, 0) = c[0]*t1 - s[0]*t2;
    puma_h(1, 0) = s[0]*t1 + c[0]*t2;
    puma_h(2, 0) = -s12*(c[3]*c[4]*c[5] - s[3]*s[5]) - c12*s[4]*c[5];
    
    t3 = -c12 * (c[3]*c[4]*s[5] + s[3]*c[5]) + s12*s[4]*s[5];
    t4 = -s[3]*c[4]*s[5] + c[3]*c[5];
    puma_h(0, 1) = c[0]*t3 - s[0]*t4;
    puma_h(1, 1) = s[0]*t3 + c[0]*t4;
    puma_h(2, 1) = s12*(c[3]*c[4]*s[5] + s[3]*c[5]) + c12*s[4]*s[5];
    
    t5 = c12*c[3]*s[4] + s12*c[4];
    t6 = s[3]*s[4];
    puma_h(0, 2) = c[0]*t5 - s[0]*t6;
    puma_h(1, 2) = s[0]*t5 + c[0]*t6;
    puma_h(2, 2) = -s12*c[3]*s[4] + c12*c[4];
    
    puma_h(3, 0) = 0;
    puma_h(3, 1) = 0;
    puma_h(3, 2) = 0;

    puma_h(0, 3) = l1*c[0]*c[1] + l2*c[0]*s12 + l3*puma_h(0, 2);
    puma_h(1, 3) = l1*s[0]*c[1] + l2*s[0]*s12 + l3*puma_h(1, 2);
    puma_h(2, 3) = -l1*s[1] + l2*c12 + l3*puma_h(2, 2);
    puma_h(3, 3) = 1;

    cout << fk(puma, theta) << endl;
    cout << puma_h << endl;
    return;
}

void test_ik(){
    cout << "test_ik in" << endl;
    vd theta(6);
    double l1, l2, l3; l1 = l2 = l3 = 1;

    init_random();
    for(unsigned int i=0; i<theta.size(); i++) theta[i] = RAD(th(mt));
    cout << "random initialized" << endl;

    dh puma = make_puma(l1, l2, l3);
    cout << "puma initialized" << endl;

    tform pos;
    pos << 1, 0, 0, 1,
           0, 1, 0, 0,
           0, 0, 1, 2,
           0, 0, 0, 1;
    auto joints = puma_ik(puma, pos);
    for(auto e: joints) cout << e << " ";
    cout << endl;
}

int main(){
    // dh link;
    // link.n = 2;
    // link.param = vector<vd>(2, vd({0., 1., RAD(90)}));
    // vd theta({0, 0});
    // cout << fk(link, theta) << endl;;

    // dh puma = make_puma(1, 1, 1);
    // vd theta(6, 0);
    // cout << fk(puma, theta) << endl;

    test_fk();
    test_ik();

    return 0;
}