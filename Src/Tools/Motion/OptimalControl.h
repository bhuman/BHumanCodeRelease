/**
 * @file OptimalControl.h
 *
 * This file declares functions that are useful for controllers.
 *
 * @author Arne Hasselbring (dare is copied from somewhere else, maybe Alexis/Arne?)
 */

#include "Tools/Math/Eigen.h"

namespace OptimalControl
{
  /**
   * Solves the discrete-time algebraic Riccati equation K = A'*K*A - A'*K*B * (R+B'*K*B)^(-1) * (B'*K*A) + Q.
   * @tparam n The number of states of the system.
   * @tparam m The number of inputs of the system.
   * @param A The state transition matrix of the system.
   * @param B The input matrix of system.
   * @param Q The cost matrix for the states.
   * @param R The cost matrix for the inputs.
   * @param K The solution of the equation.
   * @return Whether the iterative method converged.
   */
  template<int n, int m>
  inline bool dare(const Eigen::Matrix<float, n, n>& A, const Eigen::Matrix<float, n, m>& B,
                   const Eigen::Matrix<float, n, n>& Q, const Eigen::Matrix<float, m, m>& R,
                   Eigen::Matrix<float, n, n>& K);

  /**
   * Solves the discrete-time linear quadratic regulator problem (minimizing the inifinte sum over x'*Q*x + u'*R*u).
   * The problem is solved by a state feedback controller where the feedback gain is the result of this function.
   * This can also be used to determine an optimal observer with dlqr(A', C', ...)'.
   * The result will most likely have to be multiplied by -1. Because of reasons.
   * @tparam n The number of states of the system.
   * @tparam m The number of inputs of the system.
   * @param A The state transition matrix of the system.
   * @param B The input matrix of system.
   * @param Q The cost matrix for the states.
   * @param R The cost matrix for the inputs.
   * @param F The optimal state feedback gain.
   * @return Whether the optimal control problem was solveable.
   */
  template<int n, int m>
  inline bool dlqr(const Eigen::Matrix<float, n, n>& A, const Eigen::Matrix<float, n, m>& B,
                   const Eigen::Matrix<float, n, n>& Q, const Eigen::Matrix<float, m, m>& R,
                   Eigen::Matrix<float, m, n>& F);
};

template<int n, int m>
bool OptimalControl::dare(const Eigen::Matrix<float, n, n>& A, const Eigen::Matrix<float, n, m>& B,
                          const Eigen::Matrix<float, n, n>& Q, const Eigen::Matrix<float, m, m>& R,
                          Eigen::Matrix<float, n, n>& K)
{
  K = Q;
  const Eigen::Matrix<float, n, n> At = A.transpose();
  const Eigen::Matrix<float, m, n> Bt = B.transpose();
  for(int i = 0; i < 50000; ++i)
  {
    const Eigen::Matrix<float, n, n> AtK = At * K;
    const Eigen::Matrix<float, m, n> BtK = Bt * K;
    const Eigen::Matrix<float, n, n> Knew = AtK * A - AtK * B * (R + BtK * B).inverse() * BtK * A + Q;
    const float relError = (Knew - K).norm() / Knew.norm();
    K = Knew;
    if(!std::isfinite(relError))
      return false;
    else if(relError < 1e-08f)
      return true;
  }
  return false;
}

template<int n, int m>
bool OptimalControl::dlqr(const Eigen::Matrix<float, n, n>& A, const Eigen::Matrix<float, n, m>& B,
                          const Eigen::Matrix<float, n, n>& Q, const Eigen::Matrix<float, m, m>& R,
                          Eigen::Matrix<float, m, n>& F)
{
  Eigen::Matrix<float, n, n> K;
  if(!dare(A, B, Q, R, K))
    return false;

  F = (R + B.transpose() * K * B).inverse() * B.transpose() * K * A;
  return true;
}
