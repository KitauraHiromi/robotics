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

// vd puma_ik(dh link, tform pos){

// }

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
    puma_h(0, 1) = s[0]*t1 + c[0]*t2;
    puma_h(0, 2) = -s12*(c[3]*c[4]*c[5] - s[3]*s[5]) + c12*s[4]*c[5];
    
    t3 = -c12 * (c[3]*c[4]*s[5] + s[3]*c[5]) + s12*s[4]*s[5];
    t4 = -s[3]*c[4]*s[5] + c[3]*c[5];
    puma_h(1, 0) = c[0]*t3 - s[0]*t4;
    puma_h(1, 1) = s[0]*t3 + c[0]*t4;
    puma_h(1, 2) = -s12*(c[3]*c[4]*s[5] + s[3]*c[5]) + c12*s[4]*s[5];
    
    t5 = c12*c[3]*s[4] + s12*c[4];
    t6 = s[3]*s[4];
    puma_h(2, 0) = c[0]*t5 - s[0]*t6;
    puma_h(2, 1) = s[0]*t5 + c[0]*t6;
    puma_h(2, 2) = -s12*c[3]*s[6] + c12*c[4];
    
    puma_h(3, 0) = 0;
    puma_h(3, 1) = 0;
    puma_h(3, 2) = 0;

    puma_h(0, 3) = l1*c[0]*c[1] + l2*c[0]*s12 + l3*puma_h(2, 0);
    puma_h(2, 3) = l1*s[0]*c[1] + l2*s[1]*s12 + l3*puma_h(2, 1);
    puma_h(1, 3) = -l1*s[1] + l2*c12 + l3*puma_h(2, 2);
    puma_h(3, 3) = 1;

    cout << fk(puma, theta) << endl;
    cout << puma_h << endl;
    return;
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

    return 0;
}