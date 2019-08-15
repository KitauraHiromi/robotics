#include "kinematics.hpp"

// void init_random(mt19937 &mt, uniform_real_distribution<double> &th, double lbound, double ubound){
//     assert(lbound < ubound);
//     std::random_device rd;
//     mt = mt19937(rd());
//     th = uniform_real_distribution<double>(lbound, ubound);
// }


tform fk(dh link, vd theta){
    assert(theta.size() == (unsigned int)link.n);
    tform end_effector_pos = tform::Identity();
    for(unsigned int i=0; i<link.n; i++){
        end_effector_pos *= link.h(link.param[i], theta[i]);
    }
    return end_effector_pos;
}

vd puma_ik(dh link, tform h){
    vd ret(6, 0);
    double x, y, z, l1, l2, l3, l;
    x = h(0, 3);
    y = h(1, 3);
    z = h(2, 3);
    l1 = link.param[1][0];
    l2 = link.param[3][2];
    l3 = link.param[5][2];
    cout << "length:" << l1 << " " << l2 << " " << l3 << endl;
    // θ1, θ2, θ3: defined from ps(xs, ys, zs)
    vd ps(3, 0);
    ps[0] = x - l3 * h(0, 2);
    ps[1] = y - l3 * h(1, 2);
    ps[2] = z - l3 * h(2, 2);
    cout << "xyz:" << x << " " << y << " " << z << endl;
    cout << "pxpypz:" << ps[0] << " " << ps[1] << " " << ps[2] << endl; 
    l = sqrt(ps[0]*ps[0] + ps[1]*ps[1]);

    ret[0] = atan2(ps[1], ps[0]);

    double alpha, beta;
    alpha = acos((l1*l1 + l2*l2 - l*l - ps[2]*ps[2]) / (2 * l1 * l2));
    // alpha = (alpha > M_PI/2) ? alpha - M_PI : alpha;
    beta = acos((l*l + ps[2]*ps[2] + l1*l1 - l2*l2) / (2 * l1 * sqrt(l*l + ps[2]*ps[2])));
    cout << "alpha, beta:" << alpha << " " << beta << endl;

    ret[1] = -atan2(ps[2], l) + beta;
    ret[2] = -M_PI / 2 + alpha;

    tform h03_t = fk(link, ret);
    h03_t.transposeInPlace();
    zyz_eular(h03_t * h, ret[3], ret[4], ret[5]);
    return ret;
}

void zyz_eular(tform h, double& alpha, double& beta, double& gamma){
    // ジンバルロックは判定しない
    alpha = atan2(h(1, 2), h(0, 2));
    beta = atan2(h(0, 2)*cos(alpha) + h(1, 2)*sin(alpha), h(2, 2));
    gamma = atan2(-h(1, 1)*sin(alpha) + h(1, 0)*cos(alpha), -h(0, 1)*sin(alpha) + h(1, 1)*cos(alpha)); 
}


void xyz_eular(tform h, double& alpha, double& beta, double& gamma){
    // ジンバルロックは判定しない
    alpha = atan2(-h(1, 2), h(2, 2));
    beta = asin(h(0, 2));
    gamma = atan2(-h(0, 1), h(0, 0)); 
}

tform rot_x(double x){
    tform ret;
    ret << 1,      0,       0, 0,
           0, sin(x), -cos(x), 0,
           0, cos(x),  sin(x), 0,
           0,      0,       0, 1;
    return ret;
}


tform rot_y(double y){
    tform ret;
    ret <<  cos(y), 0, sin(y), 0,
                 0, 1,      0, 0,
           -sin(y), 0, cos(y), 0,
                 0, 0,      0, 1;
    return ret;
}

tform rot_z(double z){
    tform ret;
    ret <<  cos(z), -sin(z), 0, 0,
           -sin(z),  cos(z), 0, 0,
                 0,       0, 1, 0,
                 0,       0, 0, 1;
    return ret;
}

tform pos_to_tform_xyz_eular(vd pos){
    assert(pos.size() == 6);
    tform h;
    h << 1, 0, 0, pos[0],
         0, 1, 0, pos[1],
         0, 0, 1, pos[2],
         0, 0, 0, 1;
    h *= rot_x(pos[3]) * rot_y(pos[4]) * rot_z(pos[5]);
    return h;
}

void test_fk(){
    cout << "test_fk in" << endl;
    vd theta(6);
    double l1, l2, l3; l1 = l2 = l3 = 1;

    // init random
    mt19937 mt;
    uniform_real_distribution<double> th;
    std::random_device rd;
    mt = mt19937(rd());
    th = uniform_real_distribution<double>(-180, 180);

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

void test_ik2(){
    cout << "test_ik2 in" << endl;
    vd theta(6);
    double l1, l2, l3; l1 = l2 = l3 = 1;

    dh puma = make_puma(l1, l2, l3);
    cout << "puma initialized" << endl;

    tform pos;
    char str[256];
    double a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p;
    ifstream ifs("ik_test_set.txt");
    // ofstream ofs("ik_test_result.txt");
    while(ifs && ifs.getline(str, 256 - 1)){
        sscanf(str,
            "%lf, %lf, %lf, %lf,"
            "%lf, %lf, %lf, %lf,"
            "%lf, %lf, %lf, %lf,"
            "%lf, %lf, %lf, %lf,"
            "%lf, %lf, %lf, %lf\n", 
            &a, &b, &c, &d,
            &e, &f, &g, &h,
            &i, &j, &k, &l,
            &m, &n, &o, &p);

        pos << a, b, c, d,
               e, f, g, h,
               i, j, k, l,
               m, n, o, p;
        auto joint = puma_ik(puma, pos);
        cout << "joint angle ";
        for(auto e: joint){
            cout << e << " ";
        }
        cout << endl;
        // ofs << endl;
        // output pass or not

        tform res = fk(puma, joint);
        for(int i=0; i<4; i++){
            for(int j=0; j<4; j++){
                if(abs(res(i, j) - pos(i, j)) < 1e-4) cout << " [pass] ";
                else cout << " [fail] ";
                cout << i << ' ' << j << " res:" << res(i, j) << " origin:" << pos(i, j) << " error: " << res(i, j) - pos(i, j) << endl;;
            }
        }
    }
}

// int main(){
//     // dh link;
//     // link.n = 2;
//     // link.param = vector<vd>(2, vd({0., 1., RAD(90)}));
//     // vd theta({0, 0});
//     // cout << fk(link, theta) << endl;;

//     // dh puma = make_puma(1, 1, 1);
//     // vd theta(6, 0);
//     // cout << fk(puma, theta) << endl;

//     test_ik2();

//     return 0;
// }