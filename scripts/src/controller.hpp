#ifndef CONTROLLER_H
#define CONTROLLER_H
#include <iostream>
#include <eigen3/Eigen/Dense>
//#include <odroid/odroid_node.hpp>
using namespace Eigen;
class controller
{
public:
  double kx,kv,kiX,kR,kW,kiR, eiR_sat, eiX_sat;
  double cR,cX;
  double m = 1.4;
  double g = 9.81;
  double f,fdot,nA,nLd2;
  double del_t = 0.01;
  Vector3d e3;
  Matrix3d J,R;
  Matrix<double,8,3> xc_all;
  Matrix3d Rc,Rc_dot,Rc_2dot;
  Vector3d x,v,W, xc,xc_dot,xc_2dot,xc_3dot,xc_4dot,b1d,b1d_dot,b1d_2dot,Wc,Wc_dot;
  Vector3d ex,ev,M,eW,eiR,ea,Adot,eb,Addot,Ld,Lddot,Ldddot,Ld2,Ld2dot,Ld2ddot,Rd2;
  Vector3d Rd2dot,Rd2ddot,Rd1,Rd1dot,Rd1ddot,eR,eiR_last,eiX,eiX_last,A,L,Ldot;
  Matrix<double, 4, 1> FM;

  controller(double m_in, double dt_in, double *J_in, double* gains);

  void GeometricPositionController(double* x_in, double* v_in, double* R_in, double* W_in, double* xc_in, double* M_out);

  void get_Rc(double* R){
  Map<Matrix<double,3,3, RowMajor>>(R,3,3) = Rc;
  };

  void get_Rc_dot(double* R){
  Map<Matrix<double,3,3, RowMajor>>(R,3,3) = Rc_dot;
  };

  void get_Rc_2dot(double* R){
  Map<Matrix<double,3,3, RowMajor>>(R,3,3) = Rc_2dot;
  };

  void get_b1d(double* R){
  Map<Vector3d>(R,3) = b1d;
  };

  void get_eiR(double* R){
  Map<Vector3d>(R,3) = eiR;
  };
  void get_eR(double* R){
  Map<Vector3d>(R,3) = eR;
  };

  void get_Wc(double* R){
  Map<Vector3d>(R,3) = Wc;
  };

  void get_Wc_dot(double* R){
  Map<Vector3d>(R,3) = Wc_dot;
  };

  void get_eW(double* R){
  Map<Vector3d>(R,3) = eW;
  };

  void get_e3(double* R){
  Map<Vector3d>(R,3) = e3;
  };

  void get_ex(double* R){
  Map<Vector3d>(R,3) = ex;
  };

  void get_ev(double* R){
  Map<Vector3d>(R,3) = ev;
  };
};
#endif
