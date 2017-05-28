#ifndef CONTROLLER_H
#define CONTROLLER_H
#include <iostream>
#include <eigen3/Eigen/Dense>
//#include <odroid/odroid_node.hpp>
using namespace Eigen;
class controller
{
public:
  double kx = 0.1;
  double kv = 0.1;
  double kiX = 0.1;
  double kR = 1;
  double kW = 1;
  double kiR = 0.1;

  double cR;
  double m = 1.4;
  double g = 9.81;

  double del_t = 0.01;
  Matrix3d J;
  Matrix3d Rc,Rc_dot,Rc_2dot;
  controller(double m_in, double *J_in, double* gains);
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
};
#endif
