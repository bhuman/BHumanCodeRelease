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
// ode/src/joinst/fixed.cpp
//
//
////////////////////////////////////////////////////////////////////////////////

#include <UnitTest++.h>
#include <ode/ode.h>

#include "../../ode/src/config.h"
#include "../../ode/src/joints/fixed.h"

SUITE (TestdxJointFixed)
{
    struct dxJointFixed_Fixture_1
    {
        dxJointFixed_Fixture_1()
        {
            wId = dWorldCreate();

            bId1 = dBodyCreate (wId);
            dBodySetPosition (bId1, 0, -1, 0);

            bId2 = dBodyCreate (wId);
            dBodySetPosition (bId2, 0, 1, 0);

            jId   = dJointCreateFixed (wId, 0);
            joint = (dxJointFixed*) jId;


            dJointAttach (jId, bId1, bId2);
        }

        ~dxJointFixed_Fixture_1()
        {
            dWorldDestroy (wId);
        }

        dWorldID wId;

        dBodyID bId1;
        dBodyID bId2;


        dJointID jId;
        dxJointFixed* joint;
    };

    TEST_FIXTURE (dxJointFixed_Fixture_1, test_dJointSetFixed)
    {
        // the 2 bodies are align
        dJointSetFixed (jId);
        CHECK_CLOSE (joint->qrel[0], 1.0, 1e-4);
        CHECK_CLOSE (joint->qrel[1], 0.0, 1e-4);
        CHECK_CLOSE (joint->qrel[2], 0.0, 1e-4);
        CHECK_CLOSE (joint->qrel[3], 0.0, 1e-4);

        dMatrix3 R;
        // Rotate 2nd body 90deg around X
        dBodySetPosition (bId2, 0, 0, 1);
        dRFromAxisAndAngle (R, 1, 0, 0, M_PI/2.0);
        dBodySetRotation (bId2, R);

        dJointSetFixed (jId);
        CHECK_CLOSE (joint->qrel[0], 0.70710678118654757, 1e-4);
        CHECK_CLOSE (joint->qrel[1], 0.70710678118654757, 1e-4);
        CHECK_CLOSE (joint->qrel[2], 0.0, 1e-4);
        CHECK_CLOSE (joint->qrel[3], 0.0, 1e-4);


        // Rotate 2nd body -90deg around X
        dBodySetPosition (bId2, 0, 0, -1);
        dRFromAxisAndAngle (R, 1, 0, 0, -M_PI/2.0);
        dBodySetRotation (bId2, R);

        dJointSetFixed (jId);
        CHECK_CLOSE (joint->qrel[0], 0.70710678118654757, 1e-4);
        CHECK_CLOSE (joint->qrel[1], -0.70710678118654757, 1e-4);
        CHECK_CLOSE (joint->qrel[2], 0.0, 1e-4);
        CHECK_CLOSE (joint->qrel[3], 0.0, 1e-4);


        // Rotate 2nd body 90deg around Z
        dBodySetPosition (bId2, 0, 1, 0);
        dRFromAxisAndAngle (R, 0, 0, 1, M_PI/2.0);
        dBodySetRotation (bId2, R);

        dJointSetFixed (jId);
        CHECK_CLOSE (joint->qrel[0], 0.70710678118654757, 1e-4);
        CHECK_CLOSE (joint->qrel[1], 0.0, 1e-4);
        CHECK_CLOSE (joint->qrel[2], 0.0, 1e-4);
        CHECK_CLOSE (joint->qrel[3], 0.70710678118654757, 1e-4);


        // Rotate 2nd body 45deg around Y
        dBodySetPosition (bId2, 0, 1, 0);
        dRFromAxisAndAngle (R, 0, 1, 0, M_PI/4.0);
        dBodySetRotation (bId2, R);

        dJointSetFixed (jId);
        CHECK_CLOSE (joint->qrel[0], 0.92387953251128674, 1e-4);
        CHECK_CLOSE (joint->qrel[1], 0.0, 1e-4);
        CHECK_CLOSE (joint->qrel[2], 0.38268343236508984, 1e-4);
        CHECK_CLOSE (joint->qrel[3], 0.0, 1e-4);

        // Rotate in a strange manner
        // Both bodies at origin
        dRFromEulerAngles (R, REAL(0.23), REAL(3.1), REAL(-0.73));
        dBodySetPosition (bId1, 0, 0, 0);
        dBodySetRotation (bId1, R);

        dRFromEulerAngles (R, REAL(-0.57), REAL(1.49), REAL(0.81));
        dBodySetPosition (bId2, 0, 0, 0);
        dBodySetRotation (bId2, R);

        dJointSetFixed (jId);
        CHECK_CLOSE (joint->qrel[0], -0.25526036263124319, 1e-4);
        CHECK_CLOSE (joint->qrel[1],  0.28434861188441968, 1e-4);
        CHECK_CLOSE (joint->qrel[2], -0.65308047160141625, 1e-4);
        CHECK_CLOSE (joint->qrel[3],  0.65381489108282143, 1e-4);
    }


} // End of SUITE TestdxJointFixed
