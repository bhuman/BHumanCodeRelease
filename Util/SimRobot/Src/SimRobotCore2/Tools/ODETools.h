/**
* @file ODETools.h
* Utility functions for using the Open Dynamics Engine
* @author <A href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</A>
* @author <A href="mailto:kspiess@informatik.uni-bremen.de">Kai Spiess</A>
*/

#pragma once

#include <ode/ode.h>

#include "Matrix3x3.h"

class ODETools
{
public:
  /** Converts a matrix from the SimRobot format to the ODE format
  * @param m The original matrix
  * @param mODE The ODE matrix
  */
  static void convertMatrix(const Matrix3x3<>& m, dMatrix3& mODE)
  {
    mODE[0] = dReal(m.c0.x);
    mODE[1] = dReal(m.c1.x);
    mODE[2] = dReal(m.c2.x);
    mODE[3] = 0;
    mODE[4] = dReal(m.c0.y);
    mODE[5] = dReal(m.c1.y);
    mODE[6] = dReal(m.c2.y);
    mODE[7] = 0;
    mODE[8] = dReal(m.c0.z);
    mODE[9] = dReal(m.c1.z);
    mODE[10] = dReal(m.c2.z);
    mODE[11] = 0;
  }

  /**
  * Converts a matrix from the ODE format to the SimRobot format
  * @param mODE The original ODE matrix as pointer to the first value
  * @param m The SimRobot matrix
  */
  template <class V> static void convertMatrix(const dReal* mODE, Matrix3x3<V>& mSim)
  {
    mSim.c0.x = (V)mODE[0];
    mSim.c1.x = (V)mODE[1];
    mSim.c2.x = (V)mODE[2];
    mSim.c0.y = (V)mODE[4];
    mSim.c1.y = (V)mODE[5];
    mSim.c2.y = (V)mODE[6];
    mSim.c0.z = (V)mODE[8];
    mSim.c1.z = (V)mODE[9];
    mSim.c2.z = (V)mODE[10];
  }

  /**
  * Converts a 3-dimensional vector from the ODE format to the SimRobot format
  * @param vODE The original ODE vector
  * @param vSim The SimRobot vector
  */
  template <class V> static void convertVector(const dReal* vODE, Vector3<V>& vSim)
  {
    vSim.x = (V)vODE[0];
    vSim.y = (V)vODE[1];
    vSim.z = (V)vODE[2];
  }

  /**
  * Converts a 3-dimensional vector from the ODE format to a array of any type
  * @param vODE The original ODE vector
  * @param farray The float array
  */
  template <class V> static void convertVector(const dVector3& vODE, V* farray)
  {
    farray[0] = (V)vODE[0];
    farray[1] = (V)vODE[1];
    farray[2] = (V)vODE[2];
  }
};
