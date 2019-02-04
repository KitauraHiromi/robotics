#include<iostream>
#include"Eigen/Core"
using namespace std;
using namespace Eigen;
typedef Matrix<double, 4, 4> tform;
typedef VectorXd vd;

int main(){
  cout<<"Hello world"<<endl;

  MatrixXf A=MatrixXf::Zero(2,2);
  A(0,0)=2;
  A(1,1)=5;

  cout<<A<<endl;

  return 0;
}