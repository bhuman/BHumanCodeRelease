/**
 * @file ODETools.h
 * Utility functions for using the Open Dynamics Engine
 * @author <A href="mailto:tlaue@uni-bremen.de">Tim Laue</A>
 * @author <A href="mailto:kspiess@informatik.uni-bremen.de">Kai Spiess</A>
 */

#pragma once

#include <ode/ode.h>

#include "Tools/Math/Eigen.h"

class ODETools
{
public:
  /**
   * Converts a matrix from the SimRobot format to the ODE format
   * @param m The original matrix
   * @param mODE The ODE matrix
   */
  static void convertMatrix(const Matrix3f& m, dMatrix3& mODE)
  {
    mODE[0] = static_cast<dReal>(m(0, 0));
    mODE[1] = static_cast<dReal>(m(0, 1));
    mODE[2] = static_cast<dReal>(m(0, 2));
    mODE[3] = 0;
    mODE[4] = static_cast<dReal>(m(1, 0));
    mODE[5] = static_cast<dReal>(m(1, 1));
    mODE[6] = static_cast<dReal>(m(1, 2));
    mODE[7] = 0;
    mODE[8] = static_cast<dReal>(m(2, 0));
    mODE[9] = static_cast<dReal>(m(2, 1));
    mODE[10] = static_cast<dReal>(m(2, 2));
    mODE[11] = 0;
  }

  /**
   * Converts a matrix from the ODE format to the SimRobot format
   * @param mODE The original ODE matrix as pointer to the first value
   * @param m The SimRobot matrix
   */
  template<typename V>
  static void convertMatrix(const dReal* mODE, Eigen::Matrix<V, 3, 3>& mSim)
  {
    mSim(0, 0) = static_cast<V>(mODE[0]);
    mSim(0, 1) = static_cast<V>(mODE[1]);
    mSim(0, 2) = static_cast<V>(mODE[2]);
    mSim(1, 0) = static_cast<V>(mODE[4]);
    mSim(1, 1) = static_cast<V>(mODE[5]);
    mSim(1, 2) = static_cast<V>(mODE[6]);
    mSim(2, 0) = static_cast<V>(mODE[8]);
    mSim(2, 1) = static_cast<V>(mODE[9]);
    mSim(2, 2) = static_cast<V>(mODE[10]);
  }

  /**
   * Converts a 3-dimensional vector from the ODE format to the SimRobot format
   * @param vODE The original ODE vector
   * @param vSim The SimRobot vector
   */
  template<typename V>
  static void convertVector(const dReal* vODE, Eigen::Matrix<V, 3, 1>& vSim)
  {
    vSim.x() = static_cast<V>(vODE[0]);
    vSim.y() = static_cast<V>(vODE[1]);
    vSim.z() = static_cast<V>(vODE[2]);
  }

  /**
   * Converts a 3-dimensional vector from the ODE format to an array of any type
   * @param vODE The original ODE vector
   * @param farray The float array
   */
  template<typename V>
  static void convertVector(const dVector3& vODE, V* farray)
  {
    farray[0] = static_cast<V>(vODE[0]);
    farray[1] = static_cast<V>(vODE[1]);
    farray[2] = static_cast<V>(vODE[2]);
  }
};
