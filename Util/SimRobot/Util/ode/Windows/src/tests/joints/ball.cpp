/*************************************************************************
  *                                                                       *
  * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
  * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
  *                                                                       *
  * This library is free software; you can redistribute it and/or         *
  * modify it under the terms of EITHER:                                  *
  *   (1) The GNU Lesser General Public License as published by the Free  *
  *       Software Foundation; either version 2.1 of the License, or (at  *
  *       your option) any later version. The text of the GNU Lesser      *
  *       General Public License is included with this library in the     *
  *       file LICENSE.TXT.                                               *
  *   (2) The BSD-style license that is included with this library in     *
  *       the file LICENSE-BSD.TXT.                                       *
  *                                                                       *
  * This library is distributed in the hope that it will be useful,       *
  * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
  * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
  *                                                                       *
  *************************************************************************/
//234567890123456789012345678901234567890123456789012345678901234567890123456789
//        1         2         3         4         5         6         7

////////////////////////////////////////////////////////////////////////////////
// This file create unit test for some of the functions found in:
// ode/src/joinst/ball.cpp
//
//
////////////////////////////////////////////////////////////////////////////////

#include <UnitTest++.h>
#include <ode/ode.h>

#include "../../ode/src/config.h"
#include "../../ode/src/joints/ball.h"


using namespace std;

SUITE (TestdxJointBall)
{
  // The 2 bodies are positionned at (-1, -2, -3),  and (11, 22, 33)
  // The bodis have rotation of 27deg around some axis.
  // The joint is a Ball Joint
  // Axis is along the X axis
  // Anchor at (0, 0, 0)
  struct dxJointBall_Fixture_B1_and_B2_At_Zero_Axis_Along_X {
    dxJointBall_Fixture_B1_and_B2_At_Zero_Axis_Along_X()
    {
      wId = dWorldCreate();

      for (int j=0; j<2; ++j) {
        bId[j][0] = dBodyCreate (wId);
        dBodySetPosition (bId[j][0], -1, -2, -3);

        bId[j][1] = dBodyCreate (wId);
        dBodySetPosition (bId[j][1], 11, 22, 33);


        dMatrix3 R;
        dVector3 axis; // Random axis

        axis[0] =  REAL(0.53);
        axis[1] = -REAL(0.71);
        axis[2] =  REAL(0.43);
        dNormalize3(axis);
        dRFromAxisAndAngle (R, axis[0], axis[1], axis[2],
                            REAL(0.47123)); // 27deg
        dBodySetRotation (bId[j][0], R);


        axis[0] =  REAL(1.2);
        axis[1] =  REAL(0.87);
        axis[2] = -REAL(0.33);
        dNormalize3(axis);
        dRFromAxisAndAngle (R, axis[0], axis[1], axis[2],
                            REAL(0.47123)); // 27deg
        dBodySetRotation (bId[j][1], R);

        jId[j]   = dJointCreateBall (wId, 0);
        dJointAttach (jId[j], bId[j][0], bId[j][1]);
      }
    }

    ~dxJointBall_Fixture_B1_and_B2_At_Zero_Axis_Along_X()
    {
      dWorldDestroy (wId);
    }

    dWorldID wId;

    dBodyID bId[2][2];


    dJointID jId[2];
  };

  // Rotate 2nd body 90deg around X then back to original position
  //
  //   ^  ^       ^
  //   |  |  =>   |  <---
  //   |  |       |
  //  B1  B2     B1   B2
  //
  // Start with a Delta of 90deg
  //   ^           ^   ^
  //   | <---  =>  |   |
  //   |           |   |
  //  B1  B2      B1   B2
  TEST_FIXTURE (dxJointBall_Fixture_B1_and_B2_At_Zero_Axis_Along_X,
                test_dJointSetBallAxisOffset_B2_90deg) {

   dVector3 anchor;
    dJointGetBallAnchor(jId[1], anchor);
    dJointSetBallAnchor(jId[1], anchor[0], anchor[1], anchor[2]);


    for (int b=0; b<2; ++b) {
      // Compare body b of the first joint with its equivalent on the
      // second joint
      const dReal *qA = dBodyGetQuaternion(bId[0][b]);
      const dReal *qB = dBodyGetQuaternion(bId[1][b]);
      CHECK_CLOSE (qA[0], qB[0], 1e-4);
      CHECK_CLOSE (qA[1], qB[1], 1e-4);
      CHECK_CLOSE (qA[2], qB[2], 1e-4);
      CHECK_CLOSE (qA[3], qB[3], 1e-4);
    }

    dWorldStep (wId,0.5);
    dWorldStep (wId,0.5);
    dWorldStep (wId,0.5);
    dWorldStep (wId,0.5);

    for (int b=0; b<2; ++b) {
      // Compare body b of the first joint with its equivalent on the
      // second joint
      const dReal *qA = dBodyGetQuaternion(bId[0][b]);
      const dReal *qB = dBodyGetQuaternion(bId[1][b]);
      CHECK_CLOSE (qA[0], qB[0], 1e-4);
      CHECK_CLOSE (qA[1], qB[1], 1e-4);
      CHECK_CLOSE (qA[2], qB[2], 1e-4);
      CHECK_CLOSE (qA[3], qB[3], 1e-4);


      const dReal *posA = dBodyGetPosition(bId[0][b]);
      const dReal *posB = dBodyGetPosition(bId[1][b]);
      CHECK_CLOSE (posA[0], posB[0], 1e-4);
      CHECK_CLOSE (posA[1], posB[1], 1e-4);
      CHECK_CLOSE (posA[2], posB[2], 1e-4);
      CHECK_CLOSE (posA[3], posB[3], 1e-4);
    }
  }




} // End of SUITE TestdxJointBall


