#include "controller.hpp"
#include "aux_functions.h"
using namespace Eigen;
using namespace std;

controller::controller(double m_in,double dt_in, double *J_in, double* gains){
    m = m_in;
    g = 9.81;
    del_t = dt_in;
    J = Map<Matrix<double,3,3,RowMajor>>(J_in);
    kx = gains[0];
    kv = gains[1];
    kiX = gains[2];
    kR = gains[3];
    kW = gains[4];
    kiR = gains[5];
    e3 <<0,0,1;
    eiR_last << 0,0,0;
    eiX_last << 0,0,0;
    cR = 0.1; cX = 0.1;
    eiR_sat = 0.01;
    eiX_sat = 0.1;
}

void controller::GeometricPositionController(double* x_in, double* v_in, double* R_in, double* W_in, double* xc_in,double* M_out){
  std::cout.precision(5);
  // array to eigen
  x = Map<Vector3d>(x_in);
  v = Map<Vector3d>(v_in);
  R = Map<Matrix<double,3,3,RowMajor>>(R_in);
  W = Map<Vector3d>(W_in);
  xc_all = Map<Matrix<double,8,3,RowMajor>>(xc_in);
  xc = xc_all.block<1,3>(0,0);
  xc_dot = xc_all.block<1,3>(1,0);
  xc_2dot = xc_all.block<1,3>(2,0);
  xc_3dot = xc_all.block<1,3>(3,0);
  xc_4dot = xc_all.block<1,3>(4,0);
  b1d = xc_all.block<1,3>(5,0);
  b1d_dot = xc_all.block<1,3>(6,0);
  b1d_2dot = xc_all.block<1,3>(7,0);
  // Translational Error Functions
  ex = x - xc;
  ev = v - xc_dot;
  eiX = eiX_last+del_t*(ex+cX*ev);
  err_sat(-eiX_sat, eiX_sat, eiX);
  // Force 'f' along negative b3-axis
  A = -kx*ex-kv*ev-kiX*eiX-m*g*e3+m*xc_2dot;
  L = R*e3;
  Ldot = R*hat_eigen(W)*e3;
  f = -A.dot(R*e3);

  // Intermediate Terms for Rotational Errors
  ea = g*e3-f/m*L-xc_2dot;
  Adot = -kx*ev-kv*ea+m*xc_3dot;// Lee Matlab: -ki*satdot(sigma,ei,ev+c1*ex);

  fdot = -Adot.dot(L)-A.dot(Ldot);
  eb = -fdot/m*L-f/m*Ldot-xc_3dot;
  Addot = -kx*ea-kv*eb+m*xc_4dot;// Lee Matlab: -ki*satdot(sigma,ei,ea+c1*ev);

  nA = A.norm();
  Ld = -A/nA;
  Lddot = -Adot/nA+A*A.dot(Adot)/pow(nA,3);
  Ldddot = -Addot/nA+Adot/pow(nA,3)*(2*A.dot(Adot))
          +A/pow(nA,3)*(Adot.dot(Adot)+A.dot(Addot))
          -3*A/pow(nA,5)*pow(A.dot(Adot),2);

  Ld2 = -hat_eigen(b1d)*Ld;
  Ld2dot = -hat_eigen(b1d_dot)*Ld-hat_eigen(b1d)*Lddot;
  Ld2ddot = -hat_eigen(b1d_2dot)*Ld-2*hat_eigen(b1d_dot)*Lddot-hat_eigen(b1d)*Ldddot;

  nLd2 = Ld2.norm();
  Rd2 = Ld2/nLd2;
  Rd2dot = Ld2dot/nLd2-Ld2.dot(Ld2dot)/pow(nLd2,3)*Ld2;
  Rd2ddot = Ld2ddot/nLd2-Ld2.dot(Ld2dot)/pow(nLd2,3)*Ld2dot
          -Ld2dot.dot(Ld2dot)/pow(nLd2,3)*Ld2-Ld2.dot(Ld2ddot)/pow(nLd2,3)*Ld2
          -Ld2.dot(Ld2dot)/pow(nLd2,3)*Ld2dot
          +3*pow(Ld2.dot(Ld2dot),2)/pow(nLd2,5)*Ld2;

  Rd1 = hat_eigen(Rd2)*Ld;
  Rd1dot = hat_eigen(Rd2dot)*Ld+hat_eigen(Rd2)*Lddot;
  Rd1ddot = hat_eigen(Rd2ddot)*Ld+2*hat_eigen(Rd2dot)*Lddot+hat_eigen(Rd2)*Ldddot;

  Rc << Rd1, Rd2, Ld;
  Rc_dot << Rd1dot, Rd2dot, Lddot;
  Rc_2dot << Rd1ddot, Rd2ddot, Ldddot;
  // Vector3d Wc, Wc_dot;
  vee_eigen(Rc.transpose()*Rc_dot, Wc);
  vee_eigen(Rc.transpose()*Rc_2dot-hat_eigen(Wc)*hat_eigen(Wc), Wc_dot);
  // Attitude Error 'eR'
  vee_eigen(.5*(Rc.transpose()*R-R.transpose()*Rc), eR);
  // Angular Velocity Error 'eW'
  eW = W-R.transpose()*Rc*Wc;
  // Attitude Integral Term
  eiR = del_t*(eR+cR*eW) + eiR_last;
  err_sat(-eiR_sat, eiR_sat, eiR);
  //err_sat(-eiR_sat, eiR_sat, eiR);
  eiR_last = eiR;
  // 3D Moment
  M = -kR*eR-kW*eW-kiR*eiR+hat_eigen(R.transpose()*Rc*Wc)*J*R.transpose()*Rc*Wc+J*R.transpose()*Rc*Wc_dot;

  FM[0] = f;
  FM[1] = M[0];
  FM[2] = M[1];
  FM[3] = M[2];
  Map<Matrix<double,4,1>>(M_out,4) = FM;
}
