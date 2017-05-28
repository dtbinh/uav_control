#include "controller.hpp"
#include "aux_functions.h"
using namespace Eigen;
using namespace std;

controller::controller(double m_in, double *J_in, double* gains){
    m = m_in;
    J = Map<Matrix<double,3,3,RowMajor>>(J_in);
    kx = gains[0];
    kv = gains[1];
    kR = gains[3];
    kW = gains[4];
}

void controller::GeometricPositionController(double* x_in, double* v_in, double* R_in, double* W_in, double* xc_in,double* M_out){
  std::cout.precision(5);
  Vector3d eiR_last, eR;
  Vector3d x = Map<Vector3d>(x_in);
  Vector3d v = Map<Vector3d>(v_in);
  Vector3d e3(0,0,1);// commonly-used unit vector
  // convention conversion
  Matrix3d R = Map<Matrix<double,3,3,RowMajor>>(R_in);
  Vector3d W = Map<Vector3d>(W_in);
  Matrix<double,8,3> xc_all = Map<Matrix<double,8,3,RowMajor>>(xc_in);
  Vector3d xc = xc_all.block<1,3>(0,0);
  Vector3d xc_dot = xc_all.block<1,3>(1,0);
  Vector3d xc_2dot = xc_all.block<1,3>(2,0);
  Vector3d xc_3dot = xc_all.block<1,3>(3,0);
  Vector3d xc_4dot = xc_all.block<1,3>(4,0);
  Vector3d b1d = xc_all.block<1,3>(5,0);
  Vector3d b1d_dot = xc_all.block<1,3>(6,0);
  Vector3d b1d_ddot = xc_all.block<1,3>(7,0);
  Vector3d Wc, Wc_dot;
  // Translational Error Functions
  Vector3d ex = x - xc;
  Vector3d ev = v - xc_dot;
//  Vector3d eiX = eiX_last+del_t*(ex+cX*ev);
  Vector3d eiX = Vector3d::Zero();
//  // Force 'f' along negative b3-axis
  Vector3d A = -kx*ex-kv*ev-kiX*eiX-m*g*e3+m*xc_2dot;
  Vector3d L = R*e3;
  Vector3d Ldot = R*hat_eigen(W)*e3;
  double f = -A.dot(R*e3);

  // Intermediate Terms for Rotational Errors
  Vector3d ea = g*e3-f/m*L-xc_2dot;
  Vector3d Adot = -kx*ev-kv*ea+m*xc_3dot;// Lee Matlab: -ki*satdot(sigma,ei,ev+c1*ex);

  double fdot = -Adot.dot(L)-A.dot(Ldot);
  Vector3d eb = -fdot/m*L-f/m*Ldot-xc_3dot;
  Vector3d Addot = -kx*ea-kv*eb+m*xc_4dot;// Lee Matlab: -ki*satdot(sigma,ei,ea+c1*ev);

  double nA = A.norm();
  Vector3d Ld = -A/nA;
  Vector3d Lddot = -Adot/nA+A*A.dot(Adot)/pow(nA,3);
  Vector3d Ldddot = -Addot/nA+Adot/pow(nA,3)*(2*A.dot(Adot))
          +A/pow(nA,3)*(Adot.dot(Adot)+A.dot(Addot))
          -3*A/pow(nA,5)*pow(A.dot(Adot),2);

  Vector3d Ld2 = -hat_eigen(b1d)*Ld;
  Vector3d Ld2dot = -hat_eigen(b1d_dot)*Ld-hat_eigen(b1d)*Lddot;
  Vector3d Ld2ddot = -hat_eigen(b1d_ddot)*Ld-2*hat_eigen(b1d_dot)*Lddot-hat_eigen(b1d)*Ldddot;

  double nLd2 = Ld2.norm();
  Vector3d Rd2 = Ld2/nLd2;
  Vector3d Rd2dot = Ld2dot/nLd2-Ld2.dot(Ld2dot)/pow(nLd2,3)*Ld2;
  Vector3d Rd2ddot = Ld2ddot/nLd2-Ld2.dot(Ld2dot)/pow(nLd2,3)*Ld2dot
          -Ld2dot.dot(Ld2dot)/pow(nLd2,3)*Ld2-Ld2.dot(Ld2ddot)/pow(nLd2,3)*Ld2
          -Ld2.dot(Ld2dot)/pow(nLd2,3)*Ld2dot
          +3*pow(Ld2.dot(Ld2dot),2)/pow(nLd2,5)*Ld2;

  Vector3d Rd1 = hat_eigen(Rd2)*Ld;
  Vector3d Rd1dot = hat_eigen(Rd2dot)*Ld+hat_eigen(Rd2)*Lddot;
  Vector3d Rd1ddot = hat_eigen(Rd2ddot)*Ld+2*hat_eigen(Rd2dot)*Lddot+hat_eigen(Rd2)*Ldddot;

  Rc << Rd1, Rd2, Ld;
  Rc_dot << Rd1dot, Rd2dot, Lddot;
  Rc_2dot << Rd1ddot, Rd2ddot, Ldddot;
  // Vector3d Wc, Wc_dot;
  vee_eigen(Rc_dot, Wc);
  vee_eigen(Rc.transpose()*Rc_2dot-hat_eigen(Wc)*hat_eigen(Wc), Wc_dot);
  // Attitude Error 'eR'
  vee_eigen(.5*(Rc.transpose()*R-R.transpose()*Rc), eR);
  // Angular Velocity Error 'eW'
  Vector3d eW = W-R.transpose()*Rc*Wc;
  // Attitude Integral Term
  Vector3d eiR = del_t*(eR+cR*eW) + eiR_last;
  //err_sat(-eiR_sat, eiR_sat, eiR);
  eiR_last = eiR;
  // 3D Moment
  Vector3d M = -kR*eR-kW*eW-kiR*eiR+hat_eigen(R.transpose()*Rc*Wc)*J*R.transpose()*Rc*Wc+J*R.transpose()*Rc*Wc_dot;// LBFF

  Matrix<double, 4, 1> FM;
  FM[0] = f;
  FM[1] = M[0];
  FM[2] = M[1];
  FM[3] = M[2];
  Map<Matrix<double,4,1>>(M_out,4) = FM;
}
