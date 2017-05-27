#ifndef CONTROLLER_H
#define CONTROLLER_H
#include <iostream>
#include <eigen3/Eigen/Dense>
//#include <odroid/odroid_node.hpp>
using namespace Eigen;
class controller
{
public:
  //static void GeometricPositionController( Vector3d xd, Vector3d xd_dot, Vector3d xd_ddot,Vector3d Wd, Vector3d Wddot, Vector3d x_v, Vector3d v_v, Vector3d W_in, Matrix3d R_v, Matrix3d J);
  void GeometricPositionController(double* xc);
};
#endif
