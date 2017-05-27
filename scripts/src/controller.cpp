#include "controller.hpp"
using namespace Eigen;
using namespace std;

void controller::GeometricPositionController(double* xc){
//void controller::GeometricPositionController(Vector3d xd, Vector3d xd_dot, Vector3d xd_ddot,Vector3d Wd, Vector3d Wddot, Vector3d x_v, Vector3d v_v, Vector3d W_in, Matrix3d R_v, Matrix3d J){
  std::cout.precision(5);
  Vector3d xd_2dot, xd_3dot, xd_4dot, b1d, b1d_dot, b1d_ddot;
  xd_2dot = xd_3dot = xd_4dot = b1d_dot = b1d_ddot = Vector3d::Zero();
//  b1d = Vector3d(1,0,0);
//  Matrix3d R_conv;
//  Vector3d e3(0,0,1);// commonly-used unit vector
//  double del_t = 0.01;
//  // convention conversion
//  Vector3d x = R_conv*x_v;
//  Vector3d v = R_conv*v_v;
//  Matrix3d R = R_conv*R_v*R_conv;
//  Vector3d W = W_in;
//
//  xd = R_conv*xd;
//  xd_dot = R_conv*xd_dot;
//  xd_2dot = R_conv*xd_2dot;
//  xd_3dot = R_conv*xd_3dot;
//  xd_4dot = R_conv*xd_4dot;
//
//  b1d = R_conv*b1d;
//  b1d_dot = R_conv*b1d_dot;
//  b1d_ddot = R_conv*b1d_ddot;
//
//  // Translational Error Functions
//  Vector3d ex = x - xd;
//  Vector3d ev = v - xd_dot;
//  Vector3d eX = ex;  Vector3d eV = ev;
//
//
//  Vector3d eiX = eiX_last+del_t*(ex+cX*ev);
//
//  // Force 'f' along negative b3-axis
//  double kx = 0.1;
//  double kv = 0.1;
//  double kiX = 0.1;
//  double kR = 1;
//  double kW = 1;
//  double kiR = 0.1;
//
//  double m = 1.4;
//  double g = 9.81;
//
//  Vector3d A = -kx*ex-kv*ev-kiX*node.eiX-m*g*e3+m*xd_2dot;
//  Vector3d L = R*e3;
//  Vector3d Ldot = R*hat_eigen(W)*e3;
//  double f = -A.dot(R*e3);
//
//  // Intermediate Terms for Rotational Errors
//  Vector3d ea = g*e3-f/m*L-xd_2dot;
//  Vector3d Adot = -kx*ev-kv*ea+m*xd_3dot;// Lee Matlab: -ki*satdot(sigma,ei,ev+c1*ex);
//
//  double fdot = -Adot.dot(L)-A.dot(Ldot);
//  Vector3d eb = -fdot/m*L-f/m*Ldot-xd_3dot;
//  Vector3d Addot = -kx*ea-kv*eb+m*xd_4dot;// Lee Matlab: -ki*satdot(sigma,ei,ea+c1*ev);
//
//  double nA = A.norm();
//  Vector3d Ld = -A/nA;
//  Vector3d Lddot = -Adot/nA+A*A.dot(Adot)/pow(nA,3);
//  Vector3d Ldddot = -Addot/nA+Adot/pow(nA,3)*(2*A.dot(Adot))
//          +A/pow(nA,3)*(Adot.dot(Adot)+A.dot(Addot))
//          -3*A/pow(nA,5)*pow(A.dot(Adot),2);
//
//  Vector3d Ld2 = -hat_eigen(b1d)*Ld;
//  Vector3d Ld2dot = -hat_eigen(b1d_dot)*Ld-hat_eigen(b1d)*Lddot;
//  Vector3d Ld2ddot = -hat_eigen(b1d_ddot)*Ld-2*hat_eigen(b1d_dot)*Lddot-hat_eigen(b1d)*Ldddot;
//
//  double nLd2 = Ld2.norm();
//  Vector3d Rd2 = Ld2/nLd2;
//  Vector3d Rd2dot = Ld2dot/nLd2-Ld2.dot(Ld2dot)/pow(nLd2,3)*Ld2;
//  Vector3d Rd2ddot = Ld2ddot/nLd2-Ld2.dot(Ld2dot)/pow(nLd2,3)*Ld2dot
//          -Ld2dot.dot(Ld2dot)/pow(nLd2,3)*Ld2-Ld2.dot(Ld2ddot)/pow(nLd2,3)*Ld2
//          -Ld2.dot(Ld2dot)/pow(nLd2,3)*Ld2dot
//          +3*pow(Ld2.dot(Ld2dot),2)/pow(nLd2,5)*Ld2;
//
//  Vector3d Rd1 = hat_eigen(Rd2)*Ld;
//  Vector3d Rd1dot = hat_eigen(Rd2dot)*Ld+hat_eigen(Rd2)*Lddot;
//  Vector3d Rd1ddot = hat_eigen(Rd2ddot)*Ld+2*hat_eigen(Rd2dot)*Lddot+hat_eigen(Rd2)*Ldddot;
//
//  Matrix3d Rd, Rddot, Rdddot;
//  Rd << Rd1, Rd2, Ld;
//  Rddot << Rd1dot, Rd2dot, Lddot;
//  Rdddot << Rd1ddot, Rd2ddot, Ldddot;
//  Matrix3d R_c = Rd;
//  Vector3d W_c = Rddot;
//  Vector3d Wdot_c = Rddot;
//  b1d = b1d;
//  // Vector3d Wd, Wddot;
//  vee_eigen(Rd.transpose()*Rddot, Wd);
//  vee_eigen(Rd.transpose()*Rdddot-hat_eigen(Wd)*hat_eigen(Wd), Wddot);
//  // Attitude Error 'eR'
//  vee_eigen(.5*(Rd.transpose()*R-R.transpose()*Rd), eR);
//  // Angular Velocity Error 'eW'
//  Vector3d eW = W-R.transpose()*Rd*Wd;
//  // Attitude Integral Term
//  double eiR = del_t*(eR+cR*eW) + eiR_last;
//  //err_sat(-eiR_sat, eiR_sat, eiR);
//  double eiR_last = eiR;
//  // 3D Moment
//  Vector3d M = -kR*eR-kW*eW-kiR*eiR+hat_eigen(R.transpose()*Rd*Wd)*J*R.transpose()*Rd*Wd+J*R.transpose()*Rd*Wddot;// LBFF
//
//  Matrix<double, 4, 1> FM;
//  FM[0] = f;
//  FM[1] = M[0];
//  FM[2] = M[1];
//  FM[3] = M[2];
//
//  Matrix<double,4,1> f_motor = Ainv*FM;
//
}
