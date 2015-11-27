/*
 * File:   DynamicMotionPrimitive.h
 * Author: arne
 *
 * Created on January 28, 2015, 4:42 PM
 */

#pragma once
#include "Tools/Math/FifthOrderPolynomial.h"
#include "Tools/Math/Eigen.h"
#include "CanonicalSystem.h"
#include "Tools/Math/RadialBasisFunctionApproximator.h"

/**A n-dimensional dynamic motion primitive with settable end velocity.
 *
 * Read the following papers before trying to understand this code:
 *
 *  Ijspeert et al. - Dynamical Movement Primitives: Learning Attractor Models for Motor Behaviors
 *    This is a basic introduction to dynamic motion primitives. The described dmp
 *    is not the one implemented in this code but this paper helps to understand
 *    what dmps are in general.
 *
 *  Mülling et al. - Learning to Select and Generalize Striking Movements in Robot Table Tennis
 *    This paper describes the dmp implemented in this class.
 *    The variable names in the code correspond to the variable names in this paper.
 *    Note: the scaling term used in this code is different from the one in the paper.
 *
 * @param DIM number of dimensions
 */
template <int DIM>
class DynamicMotionPrimitive
{
public:
  using Vectorf = Eigen::Matrix<float, DIM, 1>;
  using Vectord = Eigen::Matrix<double, DIM, 1>;
  using Arrayf = Eigen::Array<float, DIM, 1>;
  using Matrixf = Eigen::Matrix<float, DIM, Eigen::Dynamic>;

private:
  CanonicalSystem cs;
  FifthOrderPolynomial fop[DIM]; /**<One poly for each dimension*/
  static const float alpha; /**< Dampening constant */
  static const float beta; /**< Dampening constant. Needs to be alpha/4 for critical dampening. */
  float T; /**<Time scaling parameter. FIXME should be execution time?! */
  unsigned numWeights;
  Vectorf v; /**< The current state*/
  Vectorf vStart; /**<The initial v. Used to reset the dmp. */
  Vectorf currentPos; /**<The current position */
  Vectorf currentVel; /**<The current velocity */
  Vectorf startPos;
  Vectorf startVel;
  Vectorf startAcc;
  Vectorf endPos;
  Vectorf endVel;
  Vectorf endAcc;
  RadialBasisFunctionApproximator rbf[DIM]; /**<The rbf governs the trajectory in each dimension */
  unsigned integrationSteps;
  Vectorf originalVelocityAmplitude; /**<Velocity amplitude of the original movement. Used for scaling the forcing term */
  bool initialized = false;

public:
  /**Creates an uninitialized dmp*/
  DynamicMotionPrimitive() = default;

  /**initializes the dmp.
   * After initialization the dmp will move from start to end on a straight line as fast as possible.
   * Call setWeights() to change the shape of the trajectory
   * @param overlap of the radial basis functions
   * @param integrationSteps number of integration steps to do each time step() is called.
   *                         E.g. if integrationSteps is 2 and dt is 0.01 the dmp will do
   *                         two steps with a dt of 0.005 each.
   */
  void initialize(const float executionTime, const Vectorf& startPos,
                  const Vectorf& startVel, const Vectorf& startAcc,
                  const Vectorf& endPos, const Vectorf& endVel,
                  const Vectorf& endAcc, const unsigned numWeights,
                  const float overlap = 0.3f, const float finalPhaseValue = 0.01f,
                  const unsigned integrationSteps = 3);

  /**Run one dmp iteration
   * @return the next position on the trajectory */
  Vectorf step(const float dt);
  /**Resets the dmp to the beginning of the motion*/
  void reset(const Vectorf& newStart, const Vectorf& newStartVel,
             const Vectorf& newStartAcc);

  /**Changes the goal location and velocity of the currently running dmp.
   * Reinitializes the fifth order polynomial. This takes some time. Do NOT call this method every frame.*/
  void changeGoal(const Vectorf& newGoal, const Vectorf& newGoalVel);
  /**Returns true if another step is possible.*/
  bool stepPossible() const;

  /**Imitates the given trajectory. This will update the weights.
   * data needs to be sampled at a constant rate.
   *
   * @see eq. (9) and (12) in [Mülling et al.]
   * @note This will reset the dmp.
   * @return the new weights  */
  Matrixf imitate(const Matrixf& positions, const Matrixf& velocities,
                  const Matrixf& accelerations);

  /** Changes the chape of the trajectory according to the weights. */
  void setWeights(const Matrixf& weights);

  const Vectorf& getCurrentVelocity() const;
  const Vectorf& getCurrentPosition() const;
  const Vectorf& getEndPosition() const;
  const Vectorf& getEndVelocity() const;

private:

  void stepInternal(const float dt);
  /**Places the centers evenly in time domain and converts them to phase domain afterwards.
     This way we get a more even rbf activation over time.
   */
  VectorXf calculateCenters(const float executionTime, const unsigned noCenters) const;
  /**Calculate forces for the given dimension. The forces are used to imitate a trajectory*/
  VectorXf calculateForces(const int dim, const Matrixf& positions, const Matrixf& velocities,
                           const Matrixf& accelerations) const;
  /**Calculates a diagonal matrix of psis over time.
   * I.e. evaluates a single radial basis function over the whole phase.
   */
  VectorXf calculatePsi(const int dim, const int weightIdx, const VectorXf& phases) const;
};

template <int DIM>
const float DynamicMotionPrimitive<DIM>::alpha = 25.0f;
template <int DIM>
const float DynamicMotionPrimitive<DIM>::beta = 6.25f;

template <int DIM>
void DynamicMotionPrimitive<DIM>::initialize(const float executionTime, const Vectorf& startPos, const Vectorf& startVel, const Vectorf& startAcc,
                                             const Vectorf& endPos, const Vectorf& endVel, const Vectorf& endAcc, const unsigned numWeights,
                                             const float overlap, const float finalPhaseValue, const unsigned integrationSteps)
{
  cs = CanonicalSystem(executionTime, 1.0f, finalPhaseValue);
  T = executionTime;
  this->numWeights = numWeights;
  v = startVel * T;
  vStart = v;
  currentPos = startPos;
  this->startPos = startPos;
  this->startVel = startVel;
  this->startAcc = startAcc;
  this->endPos = endPos;
  this->endVel = endVel;
  this->endAcc = endAcc;
  this->currentVel = startVel;
  this->integrationSteps = integrationSteps;
  this->originalVelocityAmplitude = endVel - startVel;

  VectorXf centers = calculateCenters(executionTime, numWeights);
  for(int i = 0; i < DIM; ++i)
  {
    fop[i] = FifthOrderPolynomial(0, startPos(i), startVel(i), startAcc(i), executionTime, endPos(i), endVel(i), endAcc(i));
    rbf[i] = RadialBasisFunctionApproximator(centers, overlap);
  }

  //zero weights => the dmp will move to the target on a straight line as fast as possible
  Matrixf weights(DIM, numWeights);
  weights.setZero();
  setWeights(weights);

  initialized = true;
}

template <int DIM>
Eigen::Matrix<float, DIM, 1> DynamicMotionPrimitive<DIM>::step(const float dt)
{
  ASSERT(initialized);
  const float realDt = dt / integrationSteps;
  for(unsigned i = 0; i < integrationSteps; ++i)
  {
    stepInternal(realDt);
  }
  currentVel = v / T;
  return currentPos;
}

template <int DIM>
void DynamicMotionPrimitive<DIM>::changeGoal(const Vectorf& newGoal, const Vectorf& newGoalVel)
{
  endPos = newGoal;
  endVel = newGoalVel;
  for(int i = 0; i < DIM; ++i)
  {
    fop[i] = FifthOrderPolynomial(0, startPos(i), startVel(i), startAcc(i),
                                  T, endPos(i), endVel(i), endAcc(i));
  }
}

template <int DIM>
void DynamicMotionPrimitive<DIM>::stepInternal(const float dt)
{
  const float t = cs.currentTime;
  //evaluate the polynomial at the next point in time to get the next
  //location of the moving target
  Vectorf targetPos;
  Vectorf targetVel;
  Vectorf targetAcc;
  for(int i = 0; i < DIM; ++i)
  {
    const Vectorf fopResult = fop[i].evaluate(t);
    targetPos(i) = fopResult(0);
    targetVel(i) = fopResult(1);
    targetAcc(i) = fopResult(2);
  }

  //calculate forcing term
  Vectorf f;
  for(int i = 0; i < DIM; ++i)
  {
    f[i] = rbf[i].evaluate(cs.z);
  }
  ASSERT(f.allFinite());

  //calculate scaling term
  Arrayf eta = (endVel - startVel).array() / originalVelocityAmplitude.array();
  //sanitize eta
  for(int i = 0; i < eta.size(); ++i)
  {
    if(eta(i) == 0.0 || std::isnan(eta(i)) || std::isinf(eta(i)))
    {//this happens when start and goal velocity are identical.
      eta(i) = 1.0;
    }
  }
  f.array() *= eta;
  cs.step(dt);

  //iterate the dynamical system
  const Vectorf vd = (alpha * (beta * (targetPos - currentPos) + targetVel * T - v) +
                     targetAcc * T * T + f) / T;
  const Vectorf yd = v / T;
  v += vd * dt;
  currentPos += yd * dt;
}

template <int DIM>
void DynamicMotionPrimitive<DIM>::reset(const Vectorf& newStart, const Vectorf& newStartVel, const Vectorf& newStartAcc)
{
  vStart = newStart;
  startPos = newStart;
  startVel = newStartVel;
  startAcc = newStartAcc;
  cs.reset();
  v = vStart;
  currentPos = startPos;
  currentVel = startVel;

  for(int i = 0; i < DIM; ++i)
  {
    fop[i] = FifthOrderPolynomial(0, startPos(i), startVel(i), startAcc(i),
                                  T, endPos(i), endVel(i), endAcc(i));
  }
}

template <int DIM>
bool DynamicMotionPrimitive<DIM>::stepPossible() const
{
  //add 0.0001 to avoid accumulation errors
  return (cs.currentTime + 0.0001) < T;
}

template <int DIM>
VectorXf DynamicMotionPrimitive<DIM>::calculateCenters(const float executionTime,
                                                       const unsigned noCenters) const
{
  VectorXf centers(noCenters);
  VectorXf times = VectorXf::LinSpaced(noCenters, 0.0f, executionTime);
  for(unsigned i = 0; i < noCenters; ++i)
  {
    centers[i] = cs.getPhaseAt(times[i]);
  }
  return centers;
}

template <int DIM>
Eigen::Matrix<float, DIM, Eigen::Dynamic> DynamicMotionPrimitive<DIM>::imitate(const Matrixf& positions, const Matrixf& velocities, const Matrixf& accelerations)
{
  ASSERT(accelerations.col(accelerations.cols() - 1).isZero());
  const float dt = T / (positions.cols() - 1);
  //pre calculate phase vector
  VectorXf z(positions.cols());
  float t = 0.0f;
  for(int c = 0; c < positions.cols(); ++c)
  {
    z(c) = cs.getPhaseAt(t);
    t += dt;
  }
  Matrixf weights(DIM, numWeights);
  for(int d = 0; d < DIM; ++d)//for each dimension (each row)
  {
    const VectorXf forces = calculateForces(d, positions, velocities, accelerations);
    //calculate weights
    for(unsigned w = 0; w < numWeights; ++w)
    {
      const VectorXf psi = calculatePsi(d, w, z);
      weights(d, w) = (z.transpose() * psi.asDiagonal() * z).inverse() *
              z.transpose() * psi.asDiagonal() * forces;
    }
  }

  setWeights(weights);
  reset(positions.col(0), velocities.col(0), accelerations.col(0));
  changeGoal(positions.col(positions.cols() - 1), velocities.col(velocities.cols() - 1));
  originalVelocityAmplitude = velocities.col(velocities.cols() - 1) - velocities.col(0);

  return weights;
}
template <int DIM>
void DynamicMotionPrimitive<DIM>::setWeights(const Matrixf& weights)
{
  ASSERT(weights.cols() == static_cast<int>(numWeights));

  for(int i = 0; i < DIM; ++i)
  {
    rbf[i].setWeights(weights.row(i));
  }
}

template <int DIM>
VectorXf DynamicMotionPrimitive<DIM>::calculateForces(const int dim, const Matrixf& positions, const Matrixf& velocities, const Matrixf& accelerations) const
{
  //note: eta doesn't have any effect as long as the the motion target does not change.
  //      During imitation learning the motion target
  //      obviously doesn't change, thus eta is ignored in the following calculations.
  ASSERT(positions.cols() == velocities.cols());
  ASSERT(positions.cols() == accelerations.cols());
  ASSERT(positions.cols() > 0);
  ASSERT(dim >= 0);
  ASSERT(dim < DIM);
  VectorXf forces(positions.cols());
  const float T2 = T * T;
  float t = 0.0f;
  const float dt = T / (positions.cols() - 1);
  for(int c = 0; c < positions.cols(); ++c) //for each column
  {
    const Vector3f fopVal = fop[dim].evaluate(t);
    const float g = fopVal(0);
    const float gd = fopVal(1);
    const float gdd = fopVal(2);
    const float theta = positions(dim, c);
    const float thetad = velocities(dim, c);
    const float thetadd = accelerations(dim, c);
    t += dt;
    forces(c) = T2 * thetadd - alpha * (beta * (g - theta) + T *  gd -
                T * thetad) - gdd * T2;
  }
  return forces;
}

template <int DIM>
VectorXf DynamicMotionPrimitive<DIM>::calculatePsi(const int dim, const int weightIdx, const VectorXf& phases) const
{
  VectorXf psi(phases.rows());//a vector is used to store the diagonal values
  const float c = rbf[dim].getCenters()[weightIdx];
  const float pn = rbf[dim].getWidths()[weightIdx];
  for(int p = 0; p < phases.rows(); ++p) //for each phase
  {
    psi(p) = expf(-pn * powf(phases[p] - c, 2.0f));
  }
  return psi;
}

template <int DIM>
const Eigen::Matrix<float, DIM, 1>& DynamicMotionPrimitive<DIM>::getCurrentVelocity() const
{
  return currentVel;
}

template <int DIM>
const Eigen::Matrix<float, DIM, 1>& DynamicMotionPrimitive<DIM>::getCurrentPosition() const
{
  return currentPos;
}

template <int DIM>
const Eigen::Matrix<float, DIM, 1>& DynamicMotionPrimitive<DIM>::getEndPosition() const
{
  return endPos;
}

template <int DIM>
const Eigen::Matrix<float, DIM, 1>& DynamicMotionPrimitive<DIM>::getEndVelocity() const
{
  return endVel;
}