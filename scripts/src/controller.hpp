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

  controller(double m_in, double *J_in, double* gains);
  void GeometricPositionController(double* x_in, double* v_in, double* R_in, double* W_in, double* xc_in, double* Rc_2dot, double* M_out);
};
#endif
