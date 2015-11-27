/**
 * File:   ZmpPreviewController3.cpp
 * Author: arne
 *
 * Created on April 1, 2015, 5:36 PM
 */

#include "ZmpPreviewController3.h"
#include "Tools/Math/Constants.h"

#include <iostream>

using namespace std;
ZmpPreviewController3::ZmpPreviewController3() : initialized(false)
{}

Matrix4d ZmpPreviewController3::dare(const Matrix4d& A, const Vector4d& B, const Matrix4d& Q, double R) const
{
  //FIXME use same method as in dare2?!
  bool converged = false;
  Matrix4d P = Matrix4d::Identity();
  for(int i = 0; i < 10000; ++i)
  {
    const Matrix4d AX = A.transpose() * P;
    ASSERT(AX.allFinite());
    const Matrix4d AXA = AX * A;
    ASSERT(AXA.allFinite());
    const Vector4d AXB = AX * B;
    ASSERT(AXB.allFinite());
    const double M = ((B.transpose() * P * B).array() + R)(0, 0);
    ASSERT(!std::isnan(M) && !std::isinf(M));
    const Matrix4d Pnew = AXA - AXB * (1.0f / M) * AXB.transpose() + Q;
    ASSERT(Pnew.allFinite());
    const double relError = (Pnew - P).norm() / Pnew.norm();
    P = Pnew;
    if(relError < 1e-10)
    {
      converged = true;
      break;
    }
  }
  ASSERT(converged);
  return P;
}

Matrix3d ZmpPreviewController3::dare2(const Matrix3d& A, const Matrix3x2d& B, const Matrix3d& Q,
                                      const Matrix2d& R)
{
  //brute force iterative solution to the Infinite-horizon, discrete-time LQR problem
  //see: http://en.wikipedia.org/wiki/Linear-quadratic_regulator#Infinite-horizon.2C_discrete-time_LQR
  Matrix3d P = Q;
  const Matrix3d At = A.transpose();
  const Matrix2x3d Bt = B.transpose();
  bool converged = false;
  for(int i = 0; i < 10000; ++i)
  {
    Matrix3d Pnew = Q + At * P * A - At * P * B * (R + Bt * P * B).inverse() * Bt * P * A;
    const double relError = (Pnew - P).norm() / Pnew.norm();
    P = Pnew;
    if(relError < 1e-09)
    {
      converged = true;
      break;
    }
  }
  ASSERT(converged);
  return P;
}

void ZmpPreviewController3::init(const double dt, const double comHeight,
                                 const unsigned numPreviews, const double R,
                                 const double Qx, const double Qe,
                                 const Vector3d& QlDiag,
                                 const Vector2d& R0Diag)
{
  //see: writeParamNG.m in nao devils code release 2011
  const double zH = comHeight;
  const unsigned N = numPreviews;
  A0 << 1, dt, 0,
        Constants::g / zH * dt, 1, -Constants::g / zH * dt,
        0, 0, 1;

  Vector3d b0(0, 0, dt);

  RowVector3d c0(0, 0, 1);

  b << 0, 0, dt;

  Vector4d Bt;
  Bt << c0 * b0, b0;
  Vector4d It;
  It << 1, 0, 0, 0;
  Matrix4x3d Ft;
  Ft.row(0) << c0 * A0;
  Ft.row(1) << A0.row(0);
  Ft.row(2) << A0.row(1);
  Ft.row(3) << A0.row(2);
  Matrix4d Qt = Matrix4d::Zero();
  Qt(0, 0) = Qe;
  Qt.block(1, 1, 3, 3) = c0.transpose() * Qx * c0;
  Matrix4d At;
  At.col(0) = It;
  At.block(0, 1, 4, 3) = Ft;

  const Matrix4d Pt = dare(At, Bt, Qt, R);

  Gx = 1.0 / (R + Bt.transpose() * Pt * Bt) * Bt.transpose() * Pt * Ft;
  Gi = 1.0 / (R + Bt.transpose() * Pt * Bt) * Bt.transpose() * Pt * It;
  Matrix4d Ac = At - Bt * 1.0 / (R + Bt.transpose() * Pt * Bt) * Bt.transpose() * Pt * At;
  Vector4d X = -Ac.transpose() * Pt * It;
  Gd.resize(N);
  Gd(0) = -Gi;
  for(unsigned i = 1; i < N; ++i)
  {
    Gd(i) = 1.0 / (R + Bt.transpose() * Pt * Bt) * Bt.transpose() * X;
    X = Ac.transpose() * X;
  }
  ASSERT(Gd.allFinite());

  //observer parameters
  const Matrix3d Ql = QlDiag.asDiagonal();
  const Matrix2d R0 = R0Diag.asDiagonal();

  Matrix2x3d Cm;
  Cm << 1, 0, 0,
        0, 0, 1;
  const Matrix3x2d Cmt = Cm.transpose();
  const Matrix3d A0t = A0.transpose();

  //solve riccati to get P for observer
  const Matrix3d Po = dare2(A0t, Cmt, Ql, R0);
  const Matrix2x3d PG = (R0 + Cm * Po * Cmt).inverse() * (Cm * Po * A0t);
  L = PG.transpose();

  initialized = true;
  integrationError = 0.0;
  state = Vector3d::Zero();
}

void ZmpPreviewController3::reset(const double com, const double comVel, const double zmp)
{
  state << com, comVel, zmp;
  integrationError = 0.0;
}
double ZmpPreviewController3::step(const double currentZmp, const double currentCom,
                                   const std::deque<float>& zmpPreviews,
                                   const bool useFeedback, const Vector2f& feedbackFactor,
                                   const bool useFeedbackFactor)
{
  ASSERT(initialized);
  ASSERT(!std::isnan(currentZmp));
  ASSERT(!std::isinf(currentZmp));
  ASSERT(!std::isnan(currentCom));
  ASSERT(!std::isinf(currentCom));

  ASSERT(zmpPreviews.size() == (unsigned long)Gd.cols());
  double previewError = 0.0;
  for(size_t i = 0; i < zmpPreviews.size(); ++i)
  {
    const float prev = zmpPreviews[i];
    ASSERT(!std::isnan(prev) && !std::isinf(prev));
    previewError += Gd(i) * prev; //FIXME sollte hier nicht ein error berechnet werden?
  }

  //FIXME das ist komischer bullshit der observer und controller mixed?
  //FIXME maybe add a sensor control rate to diminish the effects of the diff?
  Vector2d stateDiff(currentCom - state(0), currentZmp - state(2));
  if(!useFeedback)
  {
    stateDiff = Vector2d::Zero();
  }

  const double u = (-Gi * integrationError - Gx * state - previewError);//FIXME sind die vorzeichen hier richtig?
  ASSERT(!std::isnan(u));
  ASSERT(!std::isinf(u));
  if(useFeedbackFactor)
  {
    const Vector3d feedback(stateDiff(0) * feedbackFactor(0), 0.0, stateDiff(1) * feedbackFactor(1));
    state = A0 * state + feedback + b * u;
  }
  else
  {
    state = A0 * state + L * stateDiff + b * u;
  }
  ASSERT(state.allFinite());
  integrationError += state(2) - zmpPreviews[0];
  //std::cout << "integration error " << integrationError << std::endl;
  //std::cout << "state diff " << stateDiff.transpose() << std::endl;
  //std::cout << "state " << state.transpose() << std::endl;
  //std::cout << "-----------------------------" << std::endl;
  return state(0);
}

Vector3d ZmpPreviewController3::getState() const
{
  return state;
}
