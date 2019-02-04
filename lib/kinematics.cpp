#include"dh.hpp"

tform fk(dh link, vd theta){
    if(theta.size() != (signed int)link.n) throw invalid_argument("dimension must be same");
    tform end_effector_pos = tform::Identity();
    for(int i=0; i<link.n; i++)
        end_effector_pos *= link.h(link.param[i], theta[i]);
    vd ret(6, 0);
    return end_effector_pos;
}

int main(){
    dh link;
    link.n = 2;
    link.param = vector<vd>(2, vd({0., 1., RAD(90)}));
    vd theta({0, 0});
    cout << fk(link, theta) << endl;;
    return 0;
}