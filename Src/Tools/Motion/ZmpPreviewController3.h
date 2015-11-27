/**
 * File:   ZmpPreviewController3.h
 * Author: arne
 *
 * Created on April 1, 2015, 5:36 PM
 */
#pragma once
#include "Tools/Math/Eigen.h"
#include <deque>

class ZmpPreviewController3
{
public:
  ZmpPreviewController3();

  /**
   *
   * @param currentZmp
   * @param currentCom
   * @param zmpPreviews
   * @param useFeedback If false currentZmp and currentCom will not be used
   * @return
   */
  double step(const double currentZmp, const double currentCom,
             const std::deque<float>& zmpPreviews, const bool useFeedback,
             const Vector2f& feedbackFactor, const bool useFeedbackFactor);

  /**
   * Initializes the zmp controller and lqr observer
   * @param R Controller output penalty
   * @param Qx State penalty
   * @param Qe Zmp error penalty
   * @param QlDiag State penalty for observer
   * @param R0Diag Observer output penalty
   */
  void init(const double dt, const double comHeight, const unsigned numPreviews,
            const double R, const double Qx, const double Qe,
            const Vector3d& QlDiag, const Vector2d& R0Diag);

  void reset(const double com, const double comVel, const double zmp);


  //FIXME unify interface for the two dare methods using MatrixBase
  /**Solves discrete-time algebraic Riccati equations*/
  Matrix4d dare(const Matrix4d& A, const Vector4d& B, const Matrix4d& Q, double R) const;
  Matrix3d dare2(const Matrix3d& A, const Matrix3x2d& B, const Matrix3d& Q,
                 const Matrix2d& R);


  Vector3d getState() const;

private:
  bool initialized;
  Vector3d state; /**<com, comd, zmp */
  RowVectorXd Gd;/**< Preview gains */
  RowVector3d Gx;/**< State gain */
  double integrationError;/**< current zmp error sum */
  double Gi;/**< integration error gain */
  Matrix3d A0;/**< State transition matrix */
  Matrix3x2d L; /**< Gain matrix for observer. Depends on zH */
  Vector3d b;
};
