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
// ode/src/joinst/piston.cpp
//
//
////////////////////////////////////////////////////////////////////////////////

#include <UnitTest++.h>
#include <ode/ode.h>

#include "../../ode/src/config.h"
#include "../../ode/src/joints/piston.h"

SUITE (TestdxJointPiston)
{
    // The 2 bodies are positionned at (0, 0, 0), with no rotation
    // The joint is a Piston Joint
    // Axis is along the X axis
    // Anchor at (0, 0, 0)
    struct Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Along_X
    {
        Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Along_X()
        {
            wId = dWorldCreate();

            bId1 = dBodyCreate (wId);
            dBodySetPosition (bId1, 0, 0, 0);

            bId2 = dBodyCreate (wId);
            dBodySetPosition (bId2, 0, 0, 0);

            jId   = dJointCreatePiston (wId, 0);
            joint = (dxJointPiston*) jId;


            dJointAttach (jId, bId1, bId2);

            dJointSetPistonAxis (jId, axis[0], axis[1], axis[2]);
        }

        ~Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Along_X()
        {
            dWorldDestroy (wId);
        }

        dWorldID wId;

        dBodyID bId1;
        dBodyID bId2;


        dJointID jId;
        dxJointPiston* joint;

        static const dVector3 axis;

        static const dReal offset;
    };
    const dVector3 Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Along_X::axis =
    {
        1, 0, 0
    };
    const dReal    Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Along_X::offset = REAL (3.1);

    // Move 1st body offset unit in the X direction
    //
    //  X------->       X---------> Axis -->
    //  B1          =>     B1
    //  B2              B2
    //
    // Start with a Offset of offset unit
    //
    //  X------->       X---------> Axis -->
    //     B1       =>  B1
    //  B2              B2
    TEST_FIXTURE (Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Along_X,
                  test_dJointSetPistonAxisOffset_B1_3Unit)
    {
        dJointSetPistonAnchor (jId, 0, 0, 0);

        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId1, offset, 0, 0);

        CHECK_CLOSE (offset, dJointGetPistonPosition (jId), 1e-4);

        dJointSetPistonAnchorOffset (jId, 0, 0, 0,
                                     offset*axis[0],offset*axis[1],offset*axis[2]);
        CHECK_CLOSE (offset, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId1, 0, 0, 0);
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

        // Only here to test a deprecated warning
        #if 0 // the deprecated warning is not a functional part of the API, no need to test it.
        dJointSetPistonAxisDelta (jId, 1, 0, 0, 0, 0, 0);
        #endif
    }

    // Move 1st body offset unit in the opposite X direction
    //
    //  X------->          X---------> Axis -->
    //  B1          =>  B1
    //  B2                 B2
    //
    // Start with a Offset of -offset unit
    //
    //      X------->      X---------> Axis -->
    //  B1            =>   B1
    //      B2             B2
    TEST_FIXTURE (Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Along_X,
                  test_dJointSetPistonAxisOffset_B1_Minus_3Unit)
    {
        dJointSetPistonAnchor (jId, 0, 0, 0);

        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId1, -offset, 0, 0);

        CHECK_CLOSE (-offset, dJointGetPistonPosition (jId), 1e-4);

        dJointSetPistonAnchorOffset (jId, 0, 0, 0,
                                     -offset*axis[0],-offset*axis[1],-offset*axis[2]);
        CHECK_CLOSE (-offset, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId1, 0, 0, 0);
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
    }

    // Move 2nd body offset unit in the X direction
    //
    //  X------->       X---------> Axis -->
    //  B1          =>  B1
    //  B2                 B2
    //
    // Start with a Offset of offset unit
    //
    //  X------->       X---------> Axis -->
    //  B1          =>  B1
    //     B2           B2
    TEST_FIXTURE (Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Along_X,
                  test_dJointSetPistonAxisOffset_B2_3Unit)
    {
        dJointSetPistonAnchor (jId, 0, 0, 0);

        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId2, offset, 0, 0);

        CHECK_CLOSE (-offset, dJointGetPistonPosition (jId), 1e-4);

        dJointSetPistonAnchorOffset (jId, 0, 0, 0,
                                     -offset*axis[0],-offset*axis[1],-offset*axis[2]);
        CHECK_CLOSE (-offset, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId2, 0, 0, 0);
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
    }

    // Move 2nd body offset unit in the opposite X direction
    //
    //  X------->          X---------> Axis -->
    //  B1          =>     B1
    //  B2              B2
    //
    // Start with a Offset of -offset unit
    //
    //     X------->    X---------> Axis -->
    //     B1       =>  B1
    //  B2              B2
    TEST_FIXTURE (Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Along_X,
                  test_dJointSetPistonAxisOffset_B2_Minus_3Unit)
    {
        dJointSetPistonAnchor (jId, 0, 0, 0);

        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId2, -offset, 0, 0);

        CHECK_CLOSE (offset, dJointGetPistonPosition (jId), 1e-4);

        dJointSetPistonAnchorOffset (jId, 0, 0, 0,
                                     offset*axis[0],offset*axis[1],offset*axis[2]);
        CHECK_CLOSE (offset, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId2, 0, 0, 0);
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
    }



    // The 2 bodies are positionned at (0, 0, 0), with no rotation
    // The joint is a Piston Joint
    // Axis is the opposite of the X axis
    // Anchor at (0, 0, 0)
    struct Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Inverse_of_X
    {
        Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Inverse_of_X()
        {
            wId = dWorldCreate();

            bId1 = dBodyCreate (wId);
            dBodySetPosition (bId1, 0, 0, 0);

            bId2 = dBodyCreate (wId);
            dBodySetPosition (bId2, 0, 0, 0);

            jId   = dJointCreatePiston (wId, 0);
            joint = (dxJointPiston*) jId;


            dJointAttach (jId, bId1, bId2);


            dJointSetPistonAxis (jId, axis[0], axis[1], axis[2]);
        }

        ~Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Inverse_of_X()
        {
            dWorldDestroy (wId);
        }

        dWorldID wId;

        dBodyID bId1;
        dBodyID bId2;


        dJointID jId;
        dxJointPiston* joint;

        static const dVector3 axis;
        static const dReal offset;
    };
    const dVector3 Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Inverse_of_X::axis =
    {
        -1, 0, 0
    };
    const dReal    Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Inverse_of_X::offset = REAL (3.1);

    // Move 1st body offset unit in the X direction
    //
    //  X------->       X--------->   <-- Axis
    //  B1          =>     B1
    //  B2              B2
    //
    // Start with a Offset of offset unit
    //
    //  X------->       X--------->  <-- Axis
    //     B1       =>  B1
    //  B2              B2
    TEST_FIXTURE (Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Inverse_of_X,
                  test_dJointSetPistonAxisOffset_B1_3Unit)
    {
        dJointSetPistonAnchor (jId, 0, 0, 0);

        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId1, offset, 0, 0);

        CHECK_CLOSE (-offset, dJointGetPistonPosition (jId), 1e-4);

        dJointSetPistonAnchorOffset (jId, 0, 0, 0,
                                     -offset*axis[0],-offset*axis[1],-offset*axis[2]);
        CHECK_CLOSE (-offset, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId1, 0, 0, 0);
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
    }

    // Move 1st body offset unit in the opposite X direction
    //
    //  X------->          X--------->  <-- Axis
    //  B1          =>  B1
    //  B2                 B2
    //
    // Start with a Offset of offset unit
    //
    //     X------->  X--------->      <-- Axis
    //  B1       =>   B1
    //     B2         B2
    TEST_FIXTURE (Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Inverse_of_X,
                  test_dJointSetPistonAxisOffset_B1_Minus_3Unit)
    {
        dJointSetPistonAnchor (jId, 0, 0, 0);

        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId1, -offset, 0, 0);

        CHECK_CLOSE (offset, dJointGetPistonPosition (jId), 1e-4);

        dJointSetPistonAnchorOffset (jId, 0, 0, 0,
                                     offset*axis[0],offset*axis[1],offset*axis[2]);
        CHECK_CLOSE (offset, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId1, 0, 0, 0);
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
    }

    // Move 2nd body offset unit in the X direction
    //
    //  X------->       X--------->  <-- Axis
    //  B1          =>  B1
    //  B2                 B2
    //
    // Start with a Offset of offset unit
    //
    //  X------->       X--------->  <-- Axis
    //  B1          =>  B1
    //     B2           B2
    TEST_FIXTURE (Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Inverse_of_X,
                  test_dJointSetPistonAxisOffset_B2_3Unit)
    {
        dJointSetPistonAnchor (jId, 0, 0, 0);

        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId2, offset, 0, 0);

        CHECK_CLOSE (offset, dJointGetPistonPosition (jId), 1e-4);

        dJointSetPistonAnchorOffset (jId, 0, 0, 0,
                                     offset*axis[0],offset*axis[1],offset*axis[2]);
        CHECK_CLOSE (offset, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId2, 0, 0, 0);
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
    }

    // Move 2nd body offset unit in the opposite X direction
    //
    //  X------->          X--------->  <-- Axis
    //  B1          =>     B1
    //  B2              B2
    //
    // Start with a Offset of -offset unit
    //
    //     X------->    X--------->     <-- Axis
    //     B1       =>  B1
    //  B2              B2
    TEST_FIXTURE (Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Inverse_of_X,
                  test_dJointSetPistonAxisOffset_B2_Minus_3Unit)
    {
        dJointSetPistonAnchor (jId, 0, 0, 0);

        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId2, -offset, 0, 0);

        CHECK_CLOSE (-offset, dJointGetPistonPosition (jId), 1e-4);

        dJointSetPistonAnchorOffset (jId, 0, 0, 0,
                                     -offset*axis[0],-offset*axis[1],-offset*axis[2]);
        CHECK_CLOSE (-offset, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId2, 0, 0, 0);
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
    }


    // Only body 1
    // The body are positionned at (0, 0, 0), with no rotation
    // The joint is a Piston Joint
    // Axis is along the X axis
    // Anchor at (0, 0, 0)
    struct Fixture_dxJointPiston_B1_At_Zero_Axis_Along_X
    {
        Fixture_dxJointPiston_B1_At_Zero_Axis_Along_X()
        {
            wId = dWorldCreate();

            bId1 = dBodyCreate (wId);
            dBodySetPosition (bId1, 0, 0, 0);

            jId   = dJointCreatePiston (wId, 0);
            joint = (dxJointPiston*) jId;


            dJointAttach (jId, bId1, NULL);

            dJointSetPistonAxis (jId, axis[0], axis[1], axis[2]);
        }

        ~Fixture_dxJointPiston_B1_At_Zero_Axis_Along_X()
        {
            dWorldDestroy (wId);
        }

        dWorldID wId;

        dBodyID bId1;

        dJointID jId;
        dxJointPiston* joint;

        static const dVector3 axis;

        static const dReal offset;
    };
    const dVector3 Fixture_dxJointPiston_B1_At_Zero_Axis_Along_X::axis =
    {
        1, 0, 0
    };
    const dReal    Fixture_dxJointPiston_B1_At_Zero_Axis_Along_X::offset = REAL (3.1);

    // Move 1st body offset unit in the X direction
    //
    //  X------->       X---------> Axis -->
    //  B1          =>     B1
    //
    // Start with a Offset of offset unit
    //
    //  X------->       X---------> Axis -->
    //     B1       =>  B1
    TEST_FIXTURE (Fixture_dxJointPiston_B1_At_Zero_Axis_Along_X,
                  test_dJointSetPistonAxisOffset_B1_OffsetUnit)
    {
        dJointSetPistonAnchor (jId, 0, 0, 0);

        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId1, offset, 0, 0);

        CHECK_CLOSE (offset, dJointGetPistonPosition (jId), 1e-4);

        dJointSetPistonAnchorOffset (jId, 0, 0, 0,
                                     offset*axis[0],offset*axis[1],offset*axis[2]);
        CHECK_CLOSE (offset, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId1, 0, 0, 0);
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
    }

    // Move 1st body offset unit in the opposite X direction
    //
    //  X------->          X---------> Axis -->
    //  B1          =>  B1
    //
    // Start with a Offset of -offset unit
    //
    //      X------->      X---------> Axis -->
    //  B1            =>   B1
    TEST_FIXTURE (Fixture_dxJointPiston_B1_At_Zero_Axis_Along_X,
                  test_dJointSetPistonAxisOffset_B1_Minus_OffsetUnit)
    {
        dJointSetPistonAnchor (jId, 0, 0, 0);

        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId1, -offset, 0, 0);

        CHECK_CLOSE (-offset, dJointGetPistonPosition (jId), 1e-4);

        dJointSetPistonAnchorOffset (jId, 0, 0, 0,
                                     -offset*axis[0],-offset*axis[1],-offset*axis[2]);
        CHECK_CLOSE (-offset, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId1, 0, 0, 0);
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
    }

    // Only body 1
    // The body are positionned at (0, 0, 0), with no rotation
    // The joint is a Piston Joint
    // Axis is in the oppsite X axis
    // Anchor at (0, 0, 0)
    struct Fixture_dxJointPiston_B1_At_Zero_Axis_Inverse_of_X
    {
        Fixture_dxJointPiston_B1_At_Zero_Axis_Inverse_of_X()
        {
            wId = dWorldCreate();

            bId1 = dBodyCreate (wId);
            dBodySetPosition (bId1, 0, 0, 0);

            jId   = dJointCreatePiston (wId, 0);
            joint = (dxJointPiston*) jId;


            dJointAttach (jId, bId1, NULL);

            dJointSetPistonAxis (jId, axis[0], axis[1], axis[2]);
        }

        ~Fixture_dxJointPiston_B1_At_Zero_Axis_Inverse_of_X()
        {
            dWorldDestroy (wId);
        }

        dWorldID wId;

        dBodyID bId1;

        dJointID jId;
        dxJointPiston* joint;

        static const dVector3 axis;

        static const dReal offset;
    };
    const dVector3 Fixture_dxJointPiston_B1_At_Zero_Axis_Inverse_of_X::axis =
    {
        -1, 0, 0
    };
    const dReal    Fixture_dxJointPiston_B1_At_Zero_Axis_Inverse_of_X::offset = REAL (3.1);

    // Move 1st body offset unit in the X direction
    //
    //  X------->       X--------->  <--- Axis
    //  B1          =>     B1
    //
    // Start with a Offset of offset unit
    //
    //  X------->       X--------->  <--- Axis
    //     B1       =>  B1
    TEST_FIXTURE (Fixture_dxJointPiston_B1_At_Zero_Axis_Inverse_of_X,
                  test_dJointSetPistonAxisOffset_B1_OffsetUnit)
    {
        dJointSetPistonAnchor (jId, 0, 0, 0);

        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId1, offset, 0, 0);

        CHECK_CLOSE (-offset, dJointGetPistonPosition (jId), 1e-4);

        dJointSetPistonAnchorOffset (jId, 0, 0, 0,
                                     -offset*axis[0],-offset*axis[1],-offset*axis[2]);
        CHECK_CLOSE (-offset, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId1, 0, 0, 0);
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
    }

    // Move 1st body offset unit in the opposite X direction
    //
    //  X------->          X--------->   <--- Axis
    //  B1          =>  B1
    //
    // Start with a Offset of -offset unit
    //
    //      X------->      X--------->   <--- Axis
    //  B1            =>   B1
    TEST_FIXTURE (Fixture_dxJointPiston_B1_At_Zero_Axis_Inverse_of_X,
                  test_dJointSetPistonAxisOffset_B1_Minus_OffsetUnit)
    {
        dJointSetPistonAnchor (jId, 0, 0, 0);

        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId1, -offset, 0, 0);

        CHECK_CLOSE (offset, dJointGetPistonPosition (jId), 1e-4);

        dJointSetPistonAnchorOffset (jId, 0, 0, 0,
                                     offset*axis[0],offset*axis[1],offset*axis[2]);
        CHECK_CLOSE (offset, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId1, 0, 0, 0);
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
    }









    // Only body 2
    // The body are positionned at (0, 0, 0), with no rotation
    // The joint is a Piston Joint
    // Axis is along the X axis
    // Anchor at (0, 0, 0)
    struct Fixture_dxJointPiston_B2_At_Zero_Axis_Along_X
    {
        Fixture_dxJointPiston_B2_At_Zero_Axis_Along_X()
        {
            wId = dWorldCreate();

            bId2 = dBodyCreate (wId);
            dBodySetPosition (bId2, 0, 0, 0);

            jId   = dJointCreatePiston (wId, 0);
            joint = (dxJointPiston*) jId;


            dJointAttach (jId, NULL, bId2);

            dJointSetPistonAxis (jId, axis[0], axis[1], axis[2]);
        }

        ~Fixture_dxJointPiston_B2_At_Zero_Axis_Along_X()
        {
            dWorldDestroy (wId);
        }

        dWorldID wId;

        dBodyID bId2;

        dJointID jId;
        dxJointPiston* joint;

        static const dVector3 axis;

        static const dReal offset;
    };
    const dVector3 Fixture_dxJointPiston_B2_At_Zero_Axis_Along_X::axis =
    {
        1, 0, 0
    };
    const dReal    Fixture_dxJointPiston_B2_At_Zero_Axis_Along_X::offset = REAL (3.1);

    // Move 2nd body offset unit in the X direction
    //
    //  X------->       X---------> Axis -->
    //  B2          =>     B2
    //
    // Start with a Offset of offset unit
    //
    //  X------->       X---------> Axis -->
    //     B2       =>  B2
    TEST_FIXTURE (Fixture_dxJointPiston_B2_At_Zero_Axis_Along_X,
                  test_dJointSetPistonAxisOffset_B2_OffsetUnit)
    {
        dJointSetPistonAnchor (jId, 0, 0, 0);

        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId2, offset, 0, 0);

        CHECK_CLOSE (-offset, dJointGetPistonPosition (jId), 1e-4);

        dJointSetPistonAnchorOffset (jId, 0, 0, 0,
                                     -offset*axis[0],-offset*axis[1],-offset*axis[2]);
        CHECK_CLOSE (-offset, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId2, 0, 0, 0);
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
    }

    // Move 2nd body offset unit in the opposite X direction
    //
    //  X------->          X---------> Axis -->
    //  B2          =>  B2
    //
    // Start with a Offset of -offset unit
    //
    //      X------->      X---------> Axis -->
    //  B2            =>   B2
    TEST_FIXTURE (Fixture_dxJointPiston_B2_At_Zero_Axis_Along_X,
                  test_dJointSetPistonAxisOffset_B2_Minus_OffsetUnit)
    {
        dJointSetPistonAnchor (jId, 0, 0, 0);

        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId2, -offset, 0, 0);

        CHECK_CLOSE (offset, dJointGetPistonPosition (jId), 1e-4);

        dJointSetPistonAnchorOffset (jId, 0, 0, 0,
                                     offset*axis[0],offset*axis[1],offset*axis[2]);
        CHECK_CLOSE (offset, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId2, 0, 0, 0);
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
    }

    // Only body 2
    // The body are positionned at (0, 0, 0), with no rotation
    // The joint is a Piston Joint
    // Axis is in the opposite X axis
    // Anchor at (0, 0, 0)
    struct Fixture_dxJointPiston_B2_At_Zero_Axis_Inverse_of_X
    {
        Fixture_dxJointPiston_B2_At_Zero_Axis_Inverse_of_X()
        {
            wId = dWorldCreate();

            bId2 = dBodyCreate (wId);
            dBodySetPosition (bId2, 0, 0, 0);

            jId   = dJointCreatePiston (wId, 0);
            joint = (dxJointPiston*) jId;


            dJointAttach (jId, NULL, bId2);

            dJointSetPistonAxis (jId, axis[0], axis[1], axis[2]);
        }

        ~Fixture_dxJointPiston_B2_At_Zero_Axis_Inverse_of_X()
        {
            dWorldDestroy (wId);
        }

        dWorldID wId;

        dBodyID bId2;

        dJointID jId;
        dxJointPiston* joint;

        static const dVector3 axis;

        static const dReal offset;
    };
    const dVector3 Fixture_dxJointPiston_B2_At_Zero_Axis_Inverse_of_X::axis =
    {
        -1, 0, 0
    };
    const dReal    Fixture_dxJointPiston_B2_At_Zero_Axis_Inverse_of_X::offset = REAL (3.1);

    // Move 2nd body offset unit in the X direction
    //
    //  X------->       X--------->  <--- Axis
    //  B2          =>     B2
    //
    // Start with a Offset of offset unit
    //
    //  X------->       X--------->  <--- Axis
    //     B2       =>  B2
    TEST_FIXTURE (Fixture_dxJointPiston_B2_At_Zero_Axis_Inverse_of_X,
                  test_dJointSetPistonAxisOffset_B2_OffsetUnit)
    {
        dJointSetPistonAnchor (jId, 0, 0, 0);

        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId2, offset, 0, 0);

        CHECK_CLOSE (offset, dJointGetPistonPosition (jId), 1e-4);

        dJointSetPistonAnchorOffset (jId, 0, 0, 0,
                                     offset*axis[0],offset*axis[1],offset*axis[2]);
        CHECK_CLOSE (offset, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId2, 0, 0, 0);
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
        #if 0 // another deprecated warning test?
        dJointSetPistonAxisDelta (jId, 1, 0, 0, 0, 0, 0);
        #endif
    }

    // Move 1st body offset unit in the opposite X direction
    //
    //  X------->          X--------->   <--- Axis
    //  B2          =>  B2
    //
    // Start with a Offset of -offset unit
    //
    //      X------->      X--------->   <--- Axis
    //  B2            =>   B2
    TEST_FIXTURE (Fixture_dxJointPiston_B2_At_Zero_Axis_Inverse_of_X,
                  test_dJointSetPistonAxisOffset_B2_Minus_OffsetUnit)
    {
        dJointSetPistonAnchor (jId, 0, 0, 0);

        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId2, -offset, 0, 0);

        CHECK_CLOSE (-offset, dJointGetPistonPosition (jId), 1e-4);

        dJointSetPistonAnchorOffset (jId, 0, 0, 0,
                                     -offset*axis[0],-offset*axis[1],-offset*axis[2]);
        CHECK_CLOSE (-offset, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId2, 0, 0, 0);
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
    }

    // ==========================================================================
    // Test Position Rate
    // ==========================================================================

    // Apply force on 1st body in the X direction also the Axis direction
    //
    //  X------->       X---------> Axis -->
    //  B1  F->      =>     B1
    //  B2              B2
    TEST_FIXTURE (Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Along_X,
                  test_dJointSetPistonPositionRate_Force_Along_Axis_on_B1)
    {
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
        CHECK_CLOSE (0.0, dJointGetPistonPositionRate (jId), 1e-4);

        dBodyAddForce (bId1, 1.0, 0, 0);
        dWorldQuickStep (wId, 1.0);

        CHECK_CLOSE (1, dJointGetPistonPositionRate (jId), 1e-4);
    }

    // Apply force on 1st body in the inverse X direction
    //
    //  X------->           X---------> Axis -->
    //  B1  <-F      => B1
    //  B2                  B2
    TEST_FIXTURE (Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Along_X,
                  test_dJointSetPistonPositionRate_Force_Inverse_of_Axis_on_B1)
    {
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
        CHECK_CLOSE (0.0, dJointGetPistonPositionRate (jId), 1e-4);

        dBodyAddForce (bId1, -1.0, 0, 0);
        dWorldQuickStep (wId, 1.0);

        CHECK_CLOSE (-1, dJointGetPistonPositionRate (jId), 1e-4);
    }


    // Apply force on 1st body in the X direction also the Axis direction
    //
    //  X------->       X---------> <-- Axis
    //  B1  F->      =>     B1
    //  B2              B2
    TEST_FIXTURE (Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Inverse_of_X,
                  test_dJointSetPistonPositionRate_Force_Inverse_Axis_on_B1)
    {
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
        CHECK_CLOSE (0.0, dJointGetPistonPositionRate (jId), 1e-4);

        dBodyAddForce (bId1, 1.0, 0, 0);
        dWorldQuickStep (wId, 1.0);

        CHECK_CLOSE (-1, dJointGetPistonPositionRate (jId), 1e-4);
    }

    // Apply force on 1st body in the inverse X direction
    //
    //  X------->           X---------> <-- Axis
    //  B1  <-F      => B1
    //  B2                  B2
    TEST_FIXTURE (Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Inverse_of_X,
                  test_dJointSetPistonPositionRate_Force_Along_of_Axis_on_B1)
    {
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
        CHECK_CLOSE (0.0, dJointGetPistonPositionRate (jId), 1e-4);

        dBodyAddForce (bId1, -1.0, 0, 0);
        dWorldQuickStep (wId, 1.0);

        CHECK_CLOSE (1, dJointGetPistonPositionRate (jId), 1e-4);
    }

    // Apply force on 1st body in the X direction also the Axis direction
    //
    //  X------->       X---------> Axis -->
    //  B1          =>  B1
    //  B2 F->             B2
    TEST_FIXTURE (Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Along_X,
                  test_dJointSetPistonPositionRate_Force_Along_Axis_on_B2)
    {
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
        CHECK_CLOSE (0.0, dJointGetPistonPositionRate (jId), 1e-4);

        dBodyAddForce (bId2, 1.0, 0, 0);
        dWorldQuickStep (wId, 1.0);

        CHECK_CLOSE (-1, dJointGetPistonPositionRate (jId), 1e-4);
    }

    // Apply force on 1st body in the inverse X direction
    //
    //  X------->           X---------> Axis -->
    //  B1           =>     B1
    //  B2  <-F          B2
    TEST_FIXTURE (Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Along_X,
                  test_dJointSetPistonPositionRate_Force_Inverse_of_Axis_on_B2)
    {
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
        CHECK_CLOSE (0.0, dJointGetPistonPositionRate (jId), 1e-4);

        dBodyAddForce (bId2, -1.0, 0, 0);
        dWorldQuickStep (wId, 1.0);

        CHECK_CLOSE (1, dJointGetPistonPositionRate (jId), 1e-4);
    }


    // Apply force on 1st body in the X direction also the Axis direction
    //
    //  X------->       X---------> <-- Axis
    //  B1          =>  B1
    //  B2 F->             B2
    TEST_FIXTURE (Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Inverse_of_X,
                  test_dJointSetPistonPositionRate_Force_Inverse_Axis_on_B2)
    {
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
        CHECK_CLOSE (0.0, dJointGetPistonPositionRate (jId), 1e-4);

        dBodyAddForce (bId2, 1.0, 0, 0);
        dWorldQuickStep (wId, 1.0);

        CHECK_CLOSE (1, dJointGetPistonPositionRate (jId), 1e-4);
    }

    // Apply force on 1st body in the inverse X direction
    //
    //  X------->           X---------> <-- Axis
    //  B1          =>      B1
    //  B2 <-F           B2
    TEST_FIXTURE (Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Inverse_of_X,
                  test_dJointSetPistonPositionRate_Force_Along_of_Axis_on_B2)
    {
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
        CHECK_CLOSE (0.0, dJointGetPistonPositionRate (jId), 1e-4);

        dBodyAddForce (bId2, -1.0, 0, 0);
        dWorldQuickStep (wId, 1.0);

        CHECK_CLOSE (-1, dJointGetPistonPositionRate (jId), 1e-4);
    }



    // Apply force on 1st body in the X direction also the Axis direction
    //
    //  X------->       X---------> Axis -->
    //  B1  F->      =>     B1
    TEST_FIXTURE (Fixture_dxJointPiston_B1_At_Zero_Axis_Along_X,
                  test_dJointSetPistonPositionRate_Force_Along_Axis_on_B1)
    {
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
        CHECK_CLOSE (0.0, dJointGetPistonPositionRate (jId), 1e-4);

        dBodyAddForce (bId1, 1.0, 0, 0);
        dWorldQuickStep (wId, 1.0);

        CHECK_CLOSE (1, dJointGetPistonPositionRate (jId), 1e-4);
    }

    // Apply force on 1st body in the inverse X direction
    //
    //  X------->           X---------> Axis -->
    //  B1  <-F      => B1
    TEST_FIXTURE (Fixture_dxJointPiston_B1_At_Zero_Axis_Along_X,
                  test_dJointSetPistonPositionRate_Force_Inverse_of_Axis_on_B1)
    {
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
        CHECK_CLOSE (0.0, dJointGetPistonPositionRate (jId), 1e-4);

        dBodyAddForce (bId1, -1.0, 0, 0);
        dWorldQuickStep (wId, 1.0);

        CHECK_CLOSE (-1, dJointGetPistonPositionRate (jId), 1e-4);
    }


    // Apply force on 1st body in the X direction also the Axis direction
    //
    //  X------->       X---------> <-- Axis
    //  B1  F->      =>     B1
    TEST_FIXTURE (Fixture_dxJointPiston_B1_At_Zero_Axis_Inverse_of_X,
                  test_dJointSetPistonPositionRate_Force_Inverse_Axis_on_B1)
    {
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
        CHECK_CLOSE (0.0, dJointGetPistonPositionRate (jId), 1e-4);

        dBodyAddForce (bId1, 1.0, 0, 0);
        dWorldQuickStep (wId, 1.0);

        CHECK_CLOSE (-1, dJointGetPistonPositionRate (jId), 1e-4);
    }

    // Apply force on 1st body in the inverse X direction
    //
    //  X------->           X---------> <-- Axis
    //  B1  <-F      => B1
    TEST_FIXTURE (Fixture_dxJointPiston_B1_At_Zero_Axis_Inverse_of_X,
                  test_dJointSetPistonPositionRate_Force_Along_of_Axis_on_B1)
    {
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
        CHECK_CLOSE (0.0, dJointGetPistonPositionRate (jId), 1e-4);

        dBodyAddForce (bId1, -1.0, 0, 0);
        dWorldQuickStep (wId, 1.0);

        CHECK_CLOSE (1, dJointGetPistonPositionRate (jId), 1e-4);
    }


    // Apply force on body 2 in the X direction also the Axis direction
    //
    //  X------->       X---------> Axis -->
    //  B2 F->             B2
    TEST_FIXTURE (Fixture_dxJointPiston_B2_At_Zero_Axis_Along_X,
                  test_dJointSetPistonPositionRate_Force_Along_Axis_on_B2)
    {
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
        CHECK_CLOSE (0.0, dJointGetPistonPositionRate (jId), 1e-4);

        dBodyAddForce (bId2, 1.0, 0, 0);
        dWorldQuickStep (wId, 1.0);

        CHECK_CLOSE (-1, dJointGetPistonPositionRate (jId), 1e-4);
    }

    // Apply force on body 2 in the inverse X direction
    //
    //  X------->           X---------> Axis -->
    //  B2  <-F          B2
    TEST_FIXTURE (Fixture_dxJointPiston_B2_At_Zero_Axis_Along_X,
                  test_dJointSetPistonPositionRate_Force_Inverse_of_Axis_on_B2)
    {
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
        CHECK_CLOSE (0.0, dJointGetPistonPositionRate (jId), 1e-4);

        dBodyAddForce (bId2, -1.0, 0, 0);
        dWorldQuickStep (wId, 1.0);

        CHECK_CLOSE (1, dJointGetPistonPositionRate (jId), 1e-4);
    }


    // Apply force on body 2 in the X direction also the Axis direction
    //
    //  X------->       X---------> <-- Axis
    //  B2 F->             B2
    TEST_FIXTURE (Fixture_dxJointPiston_B2_At_Zero_Axis_Inverse_of_X,
                  test_dJointSetPistonPositionRate_Force_Inverse_Axis_on_B2)
    {
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
        CHECK_CLOSE (0.0, dJointGetPistonPositionRate (jId), 1e-4);

        dBodyAddForce (bId2, 1.0, 0, 0);
        dWorldQuickStep (wId, 1.0);

        CHECK_CLOSE (1, dJointGetPistonPositionRate (jId), 1e-4);
    }

    // Apply force on body 2 in the inverse X direction
    //
    //  X------->           X---------> <-- Axis
    //  B2 <-F           B2
    TEST_FIXTURE (Fixture_dxJointPiston_B2_At_Zero_Axis_Inverse_of_X,
                  test_dJointSetPistonPositionRate_Force_Along_of_Axis_on_B2)
    {
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
        CHECK_CLOSE (0.0, dJointGetPistonPositionRate (jId), 1e-4);

        dBodyAddForce (bId2, -1.0, 0, 0);
        dWorldQuickStep (wId, 1.0);

        CHECK_CLOSE (-1, dJointGetPistonPositionRate (jId), 1e-4);
    }




// Create 2 bodies attached by a Piston joint
    // Axis is along the X axis (Default value
    // Anchor at (0, 0, 0)      (Default value)
    //
    //       ^Y
    //       |
    //       * Body2
    //       |
    //       |
    // Body1 |
    // *     Z-------->
    struct dxJointPiston_Test_Initialization
    {
        dxJointPiston_Test_Initialization()
        {
            wId = dWorldCreate();

            // Remove gravity to have the only force be the force of the joint
            dWorldSetGravity(wId, 0,0,0);

            for (int j=0; j<2; ++j)
            {
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

                jId[j] = dJointCreatePiston (wId, 0);
                dJointAttach (jId[j], bId[j][0], bId[j][1]);
            }
        }

        ~dxJointPiston_Test_Initialization()
        {
            dWorldDestroy (wId);
        }

        dWorldID wId;

        dBodyID bId[2][2];


        dJointID jId[2];

    };


    // Test if setting a Piston with its default values
    // will behave the same as a default Piston joint
    TEST_FIXTURE (dxJointPiston_Test_Initialization,
                  test_Piston_Initialization)
    {
        using namespace std;

        dVector3 axis;
        dJointGetPistonAxis(jId[1], axis);
        dJointSetPistonAxis(jId[1], axis[0], axis[1], axis[2]);


        dVector3 anchor;
        dJointGetPistonAnchor(jId[1], anchor);
        dJointSetPistonAnchor(jId[1], anchor[0], anchor[1], anchor[2]);


        for (int b=0; b<2; ++b)
        {
            // Compare body b of the first joint with its equivalent on the
            // second joint
            const dReal *qA = dBodyGetQuaternion(bId[0][b]);
            const dReal *qB = dBodyGetQuaternion(bId[1][b]);
            CHECK_CLOSE (qA[0], qB[0], 1e-6);
            CHECK_CLOSE (qA[1], qB[1], 1e-6);
            CHECK_CLOSE (qA[2], qB[2], 1e-6);
            CHECK_CLOSE (qA[3], qB[3], 1e-6);
        }

        dWorldStep (wId,0.5);
        dWorldStep (wId,0.5);
        dWorldStep (wId,0.5);
        dWorldStep (wId,0.5);

        for (int b=0; b<2; ++b)
        {
            // Compare body b of the first joint with its equivalent on the
            // second joint
            const dReal *qA = dBodyGetQuaternion(bId[0][b]);
            const dReal *qB = dBodyGetQuaternion(bId[1][b]);
            CHECK_CLOSE (qA[0], qB[0], 1e-6);
            CHECK_CLOSE (qA[1], qB[1], 1e-6);
            CHECK_CLOSE (qA[2], qB[2], 1e-6);
            CHECK_CLOSE (qA[3], qB[3], 1e-6);


            const dReal *posA = dBodyGetPosition(bId[0][b]);
            const dReal *posB = dBodyGetPosition(bId[1][b]);
            CHECK_CLOSE (posA[0], posB[0], 1e-6);
            CHECK_CLOSE (posA[1], posB[1], 1e-6);
            CHECK_CLOSE (posA[2], posB[2], 1e-6);
            CHECK_CLOSE (posA[3], posB[3], 1e-6);
        }


    }





    // Compare only one body to 2 bodies with one fixed.
    //
    // The body are positionned at (0, 0, 0), with no rotation
    // The joint is a Piston Joint
    // Axis is along the X axis
    // Anchor at (0, 0, 0)
    struct Fixture_dxJointPiston_Compare_Body_At_Zero_Axis_Along_X
    {
        Fixture_dxJointPiston_Compare_Body_At_Zero_Axis_Along_X()
        {
            wId = dWorldCreate();

            bId1_12 = dBodyCreate (wId);
            dBodySetPosition (bId1_12, 0, 0, 0);

            bId2_12 = dBodyCreate (wId);
            dBodySetPosition (bId2_12, 0, 0, 0);
            // The force will be added in the function since it is not
            // always on the same body

            jId_12 = dJointCreatePiston (wId, 0);
            dJointAttach(jId_12, bId1_12, bId2_12);

            fixed = dJointCreateFixed (wId, 0);



            bId = dBodyCreate (wId);
            dBodySetPosition (bId, 0, 0, 0);

            dBodyAddForce (bId, 4, 0, 0);

            jId = dJointCreatePiston (wId, 0);
        }

        ~Fixture_dxJointPiston_Compare_Body_At_Zero_Axis_Along_X()
        {
            dWorldDestroy (wId);
        }

        dWorldID wId;

        dBodyID bId1_12;
        dBodyID bId2_12;

        dJointID jId_12; // Joint with 2 bodies

        dJointID fixed;



        dBodyID  bId;
        dJointID jId;    // Joint with one body
    };

    // This test compare the result of a slider with 2 bodies where body body 2 is
    // fixed to the world to a slider with only one body at position 1.
    //
    // Test the limits [-1, 0.25] when only one body at is attached to the joint
    // using dJointAttache(jId, bId, 0);
    //
    TEST_FIXTURE(Fixture_dxJointPiston_Compare_Body_At_Zero_Axis_Along_X,
                 test_Limit_minus1_025_One_Body_on_left)
    {
        dBodyAddForce (bId1_12, 4, 0, 0);

        dJointAttach(jId_12, bId1_12, bId2_12);
        dJointSetPistonParam(jId_12, dParamLoStop, -1);
        dJointSetPistonParam(jId_12, dParamHiStop, 0.25);

        dJointAttach(fixed, 0, bId2_12);
        dJointSetFixed(fixed);

        dJointAttach(jId, bId, 0);
        dJointSetPistonParam(jId, dParamLoStop, -1);
        dJointSetPistonParam(jId, dParamHiStop, 0.25);


        for (int i=0; i<50; ++i)
            dWorldStep(wId, 1.0);


        const dReal *pos1_12 = dBodyGetPosition(bId1_12);
        const dReal *pos = dBodyGetPosition(bId);

        CHECK_CLOSE (pos1_12[0], pos[0], 1e-2);
        CHECK_CLOSE (pos1_12[1], pos[1], 1e-2);
        CHECK_CLOSE (pos1_12[2], pos[2], 1e-2);

        const dReal *q1_12 = dBodyGetQuaternion(bId1_12);
        const dReal *q = dBodyGetQuaternion(bId);

        CHECK_CLOSE (q1_12[0], q[0], 1e-4);
        CHECK_CLOSE (q1_12[1], q[1], 1e-4);
        CHECK_CLOSE (q1_12[2], q[2], 1e-4);
        CHECK_CLOSE (q1_12[3], q[3], 1e-4);
    }



    // This test compare the result of a slider with 2 bodies where body body 1 is
    // fixed to the world to a slider with only one body at position 2.
    //
    // Test the limits [-1, 0.25] when only one body at is attached to the joint
    // using dJointAttache(jId, 0, bId);
    //
    TEST_FIXTURE(Fixture_dxJointPiston_Compare_Body_At_Zero_Axis_Along_X,
                 test_Limit_minus1_025_One_Body_on_right)
    {
        dBodyAddForce (bId2_12, 4, 0, 0);

        dJointAttach(jId_12, bId1_12, bId2_12);
        dJointSetPistonParam(jId_12, dParamLoStop, -1);
        dJointSetPistonParam(jId_12, dParamHiStop, 0.25);

        dJointAttach(fixed, bId1_12, 0);
        dJointSetFixed(fixed);


        dJointAttach(jId, 0, bId);
        dJointSetPistonParam(jId, dParamLoStop, -1);
        dJointSetPistonParam(jId, dParamHiStop, 0.25);

        for (int i=0; i<50; ++i)
            dWorldStep(wId, 1.0);


        const dReal *pos2_12 = dBodyGetPosition(bId2_12);
        const dReal *pos = dBodyGetPosition(bId);

        CHECK_CLOSE (pos2_12[0], pos[0], 1e-2);
        CHECK_CLOSE (pos2_12[1], pos[1], 1e-2);
        CHECK_CLOSE (pos2_12[2], pos[2], 1e-2);


        const dReal *q2_12 = dBodyGetQuaternion(bId2_12);
        const dReal *q = dBodyGetQuaternion(bId);

        CHECK_CLOSE (q2_12[0], q[0], 1e-4);
        CHECK_CLOSE (q2_12[1], q[1], 1e-4);
        CHECK_CLOSE (q2_12[2], q[2], 1e-4);
        CHECK_CLOSE (q2_12[3], q[3], 1e-4);
    }



    // This test compare the result of a slider with 2 bodies where body body 2 is
    // fixed to the world to a slider with only one body at position 1.
    //
    // Test the limits [0, 0] when only one body at is attached to the joint
    // using dJointAttache(jId, bId, 0);
    //
    // The body should not move since their is no room between the two limits
    //
    TEST_FIXTURE(Fixture_dxJointPiston_Compare_Body_At_Zero_Axis_Along_X,
                 test_Limit_0_0_One_Body_on_left)
    {
        dBodyAddForce (bId1_12, 4, 0, 0);

        dJointAttach(jId_12, bId1_12, bId2_12);
        dJointSetPistonParam(jId_12, dParamLoStop, 0);
        dJointSetPistonParam(jId_12, dParamHiStop, 0);

        dJointAttach(fixed, 0, bId2_12);
        dJointSetFixed(fixed);


        dJointAttach(jId, bId, 0);
        dJointSetPistonParam(jId, dParamLoStop, 0);
        dJointSetPistonParam(jId, dParamHiStop, 0);

        for (int i=0; i<500; ++i)
            dWorldStep(wId, 1.0);


        const dReal *pos1_12 = dBodyGetPosition(bId1_12);
        const dReal *pos = dBodyGetPosition(bId);

        CHECK_CLOSE (pos1_12[0], pos[0], 1e-4);
        CHECK_CLOSE (pos1_12[1], pos[1], 1e-4);
        CHECK_CLOSE (pos1_12[2], pos[2], 1e-4);

        CHECK_CLOSE (0, pos[0], 1e-4);
        CHECK_CLOSE (0, pos[1], 1e-4);
        CHECK_CLOSE (0, pos[2], 1e-4);


        const dReal *q1_12 = dBodyGetQuaternion(bId1_12);
        const dReal *q = dBodyGetQuaternion(bId);

        CHECK_CLOSE (q1_12[0], q[0], 1e-4);
        CHECK_CLOSE (q1_12[1], q[1], 1e-4);
        CHECK_CLOSE (q1_12[2], q[2], 1e-4);
        CHECK_CLOSE (q1_12[3], q[3], 1e-4);
    }


    // This test compare the result of a slider with 2 bodies where body body 1 is
    // fixed to the world to a slider with only one body at position 2.
    //
    // Test the limits [0, 0] when only one body at is attached to the joint
    // using dJointAttache(jId, 0, bId);
    //
    // The body should not move since their is no room between the two limits
    //
    TEST_FIXTURE(Fixture_dxJointPiston_Compare_Body_At_Zero_Axis_Along_X,
                 test_Limit_0_0_One_Body_on_right)
    {
        dBodyAddForce (bId2_12, 4, 0, 0);

        dJointAttach(jId_12, bId1_12, bId2_12);
        dJointSetPistonParam(jId_12, dParamLoStop, 0);
        dJointSetPistonParam(jId_12, dParamHiStop, 0);

        dJointAttach(fixed, bId1_12, 0);
        dJointSetFixed(fixed);


        dJointAttach(jId, 0, bId);
        dJointSetPistonParam(jId, dParamLoStop, 0);
        dJointSetPistonParam(jId, dParamHiStop, 0);

        for (int i=0; i<500; ++i)
            dWorldStep(wId, 1.0);

        const dReal *pos2_12 = dBodyGetPosition(bId2_12);
        const dReal *pos = dBodyGetPosition(bId);

        CHECK_CLOSE (pos2_12[0], pos[0], 1e-4);
        CHECK_CLOSE (pos2_12[1], pos[1], 1e-4);
        CHECK_CLOSE (pos2_12[2], pos[2], 1e-4);

        CHECK_CLOSE (0, pos[0], 1e-4);
        CHECK_CLOSE (0, pos[1], 1e-4);
        CHECK_CLOSE (0, pos[2], 1e-4);


        const dReal *q2_12 = dBodyGetQuaternion(bId2_12);
        const dReal *q = dBodyGetQuaternion(bId);

        CHECK_CLOSE (q2_12[0], q[0], 1e-4);
        CHECK_CLOSE (q2_12[1], q[1], 1e-4);
        CHECK_CLOSE (q2_12[2], q[2], 1e-4);
        CHECK_CLOSE (q2_12[3], q[3], 1e-4);
    }


} // End of SUITE TestdxJointPiston
