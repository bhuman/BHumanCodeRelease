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
// ode/src/joinst/pu.cpp
//
//
////////////////////////////////////////////////////////////////////////////////

#include <UnitTest++.h>
#include <ode/ode.h>

#include "../../ode/src/config.h"
#include "../../ode/src/joints/pu.h"

SUITE (TestdxJointPU)
{
    // The 2 bodies are positionned at (0, 0, 0),  and (0, 0, 0)
    // The second body has a rotation of 27deg around X axis.
    // The joint is a PU Joint
    // Axis is along the X axis
    // Anchor at (0, 0, 0)
    struct Fixture_dxJointPU_B1_and_B2_At_Zero_Axis_Along_X
    {
        Fixture_dxJointPU_B1_and_B2_At_Zero_Axis_Along_X()
        {
            wId = dWorldCreate();

            bId1 = dBodyCreate (wId);
            dBodySetPosition (bId1, 0, 0, 0);

            bId2 = dBodyCreate (wId);
            dBodySetPosition (bId2, 0, 0, 0);

            dMatrix3 R;

            dRFromAxisAndAngle (R, 1, 0, 0, REAL(0.47123)); // 27deg
            dBodySetRotation (bId2, R);

            jId   = dJointCreatePU (wId, 0);
            joint = (dxJointPU*) jId;


            dJointAttach (jId, bId1, bId2);
        }

        ~Fixture_dxJointPU_B1_and_B2_At_Zero_Axis_Along_X()
        {
            dWorldDestroy (wId);
        }

        dWorldID wId;

        dBodyID bId1;
        dBodyID bId2;


        dJointID jId;
        dxJointPU* joint;
    };

    // Test is dJointSetPUAxis and dJointGetPUAxis return same value
    TEST_FIXTURE (Fixture_dxJointPU_B1_and_B2_At_Zero_Axis_Along_X,
                  test_dJointSetGetPUAxis)
    {
        dVector3 axisOrig, axis;


        dJointGetPUAxis1 (jId, axisOrig);
        dJointGetPUAxis1 (jId, axis);
        dJointSetPUAxis1 (jId, axis[0], axis[1], axis[2]);
        dJointGetPUAxis1 (jId, axis);
        CHECK_CLOSE (axis[0], axisOrig[0] , 1e-4);
        CHECK_CLOSE (axis[1], axisOrig[1] , 1e-4);
        CHECK_CLOSE (axis[2], axisOrig[2] , 1e-4);


        dJointGetPUAxis2 (jId, axisOrig);
        dJointGetPUAxis2(jId, axis);
        dJointSetPUAxis2 (jId, axis[0], axis[1], axis[2]);
        dJointGetPUAxis2 (jId, axis);
        CHECK_CLOSE (axis[0], axisOrig[0] , 1e-4);
        CHECK_CLOSE (axis[1], axisOrig[1] , 1e-4);
        CHECK_CLOSE (axis[2], axisOrig[2] , 1e-4);


        dJointGetPUAxis3 (jId, axisOrig);
        dJointGetPUAxis3(jId, axis);
        dJointSetPUAxis3 (jId, axis[0], axis[1], axis[2]);
        dJointGetPUAxis3 (jId, axis);
        CHECK_CLOSE (axis[0], axisOrig[0] , 1e-4);
        CHECK_CLOSE (axis[1], axisOrig[1] , 1e-4);
        CHECK_CLOSE (axis[2], axisOrig[2] , 1e-4);
    }














    // The joint is a PU Joint
    // Default joint value
    // The two bodies at at (0, 0, 0)
    struct Fixture_dxJointPU_B1_and_B2_At_Zero
    {
        Fixture_dxJointPU_B1_and_B2_At_Zero()
        {
            wId = dWorldCreate();

            bId1 = dBodyCreate (wId);
            dBodySetPosition (bId1, 0, 0, 0);

            bId2 = dBodyCreate (wId);
            dBodySetPosition (bId2, 0, 0, 0);

            jId   = dJointCreatePU (wId, 0);
            joint = (dxJointPU*) jId;


            dJointAttach (jId, bId1, bId2);
        }

        ~Fixture_dxJointPU_B1_and_B2_At_Zero()
        {
            dWorldDestroy (wId);
        }

        dWorldID wId;

        dBodyID bId1;
        dBodyID bId2;


        dJointID jId;
        dxJointPU* joint;

        static const dReal offset;
    };
    const dReal    Fixture_dxJointPU_B1_and_B2_At_Zero::offset = REAL (3.1);





    // Move 1st body offset unit in the X direction
    //
    //  X------->       X---------> Axis -->
    //  B1          =>     B1
    //  B2              B2
    //
    // Start with a Offset of offset unit
    //
    //  X------->       X---------> Axis -->
    //     B1       =>     B1
    //  B2                 B2
    TEST_FIXTURE (Fixture_dxJointPU_B1_and_B2_At_Zero,
                  test_dJointSetPUAxisOffset_B1_3Unit)
    {
        CHECK_CLOSE (0.0, dJointGetPUPosition (jId), 1e-4);

        dBodySetPosition (bId1, offset, 0, 0);

        CHECK_CLOSE (offset, dJointGetPUPosition (jId), 1e-4);

        dVector3 axis;
        dJointGetPUAxisP (jId, axis);

        dJointSetPUAnchorOffset (jId, 0, 0, 0,
                                 offset*axis[0],offset*axis[1],offset*axis[2]);
        CHECK_CLOSE (offset, dJointGetPUPosition (jId), 1e-4);

        dBodySetPosition (bId1, 0, 0, 0);
        CHECK_CLOSE (0.0, dJointGetPUPosition (jId), 1e-4);
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
    TEST_FIXTURE (Fixture_dxJointPU_B1_and_B2_At_Zero,
                  test_dJointSetPUAxisOffset_B1_Minus_3Unit)
    {
        CHECK_CLOSE (0.0, dJointGetPUPosition (jId), 1e-4);

        dBodySetPosition (bId1, -offset, 0, 0);

        CHECK_CLOSE (-offset, dJointGetPUPosition (jId), 1e-4);

        dVector3 axis;
        dJointGetPUAxisP (jId, axis);
        dJointSetPUAnchorOffset (jId, 0, 0, 0,
                                 -offset*axis[0],-offset*axis[1],-offset*axis[2]);
        CHECK_CLOSE (-offset, dJointGetPUPosition (jId), 1e-4);

        dBodySetPosition (bId1, 0, 0, 0);
        CHECK_CLOSE (0.0, dJointGetPUPosition (jId), 1e-4);
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
    TEST_FIXTURE (Fixture_dxJointPU_B1_and_B2_At_Zero,
                  test_dJointSetPUAxisOffset_B2_3Unit)
    {
        CHECK_CLOSE (0.0, dJointGetPUPosition (jId), 1e-4);

        dBodySetPosition (bId2, offset, 0, 0);

        CHECK_CLOSE (-offset, dJointGetPUPosition (jId), 1e-4);

        dVector3 axis;
        dJointGetPUAxisP (jId, axis);
        dJointSetPUAnchorOffset (jId, 0, 0, 0,
                                 -offset*axis[0],-offset*axis[1],-offset*axis[2]);
        CHECK_CLOSE (-offset, dJointGetPUPosition (jId), 1e-4);

        dBodySetPosition (bId2, 0, 0, 0);
        CHECK_CLOSE (0.0, dJointGetPUPosition (jId), 1e-4);
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
    TEST_FIXTURE (Fixture_dxJointPU_B1_and_B2_At_Zero,
                  test_dJointSetPUAxisOffset_B2_Minus_3Unit)
    {
        CHECK_CLOSE (0.0, dJointGetPUPosition (jId), 1e-4);

        dBodySetPosition (bId2, -offset, 0, 0);

        CHECK_CLOSE (offset, dJointGetPUPosition (jId), 1e-4);

        dVector3 axis;
        dJointGetPUAxisP (jId, axis);
        dJointSetPUAnchorOffset (jId, 0, 0, 0,
                                 offset*axis[0],offset*axis[1],offset*axis[2]);
        CHECK_CLOSE (offset, dJointGetPUPosition (jId), 1e-4);

        dBodySetPosition (bId2, 0, 0, 0);
        CHECK_CLOSE (0.0, dJointGetPUPosition (jId), 1e-4);
    }



    // Attach only one body at position 1 to the joint dJointAttach (jId, bId, 0)
    // Move 1st body offset unit in the X direction
    //
    //  X------->       X---------> Axis -->
    //  B1          =>     B1
    //
    // Start with a Offset of offset unit
    //
    //  X------->       X---------> Axis -->
    //     B1       =>  B1
    TEST_FIXTURE (Fixture_dxJointPU_B1_and_B2_At_Zero,
                  test_dJointSetPUAxisOffset_B1_OffsetUnit)
    {
        dJointAttach (jId, bId1, 0);

        CHECK_CLOSE (0.0, dJointGetPUPosition (jId), 1e-4);

        dBodySetPosition (bId1, offset, 0, 0);

        CHECK_CLOSE (offset, dJointGetPUPosition (jId), 1e-4);

        dVector3 axis;
        dJointGetPUAxisP (jId, axis);
        dJointSetPUAnchorOffset (jId, 0, 0, 0,
                                 offset*axis[0],offset*axis[1],offset*axis[2]);
        CHECK_CLOSE (offset, dJointGetPUPosition (jId), 1e-4);

        dBodySetPosition (bId1, 0, 0, 0);
        CHECK_CLOSE (0.0, dJointGetPUPosition (jId), 1e-4);
    }

    // Attache only one body at position 1 to the joint dJointAttach (jId, bId, 0)
    // Move 1st body offset unit in the opposite X direction
    //
    //  X------->          X---------> Axis -->
    //  B1          =>  B1
    //
    // Start with a Offset of -offset unit
    //
    //      X------->      X---------> Axis -->
    //  B1            =>   B1
    TEST_FIXTURE (Fixture_dxJointPU_B1_and_B2_At_Zero,
                  test_dJointSetPUAxisOffset_B1_Minus_OffsetUnit)
    {
        dJointAttach (jId, bId1, 0);

        CHECK_CLOSE (0.0, dJointGetPUPosition (jId), 1e-4);

        dBodySetPosition (bId1, -offset, 0, 0);

        CHECK_CLOSE (-offset, dJointGetPUPosition (jId), 1e-4);

        dVector3 axis;
        dJointGetPUAxisP (jId, axis);
        dJointSetPUAnchorOffset (jId, 0, 0, 0,
                                 -offset*axis[0],-offset*axis[1],-offset*axis[2]);
        CHECK_CLOSE (-offset, dJointGetPUPosition (jId), 1e-4);

        dBodySetPosition (bId1, 0, 0, 0);
        CHECK_CLOSE (0.0, dJointGetPUPosition (jId), 1e-4);
    }



    // Attache only one body at position 2 to the joint dJointAttach (jId, 0, bId)
    // Move 1st body offset unit in the X direction
    //
    //  X------->       X---------> Axis -->
    //  B2          =>     B2
    //
    // Start with a Offset of offset unit
    //
    //  X------->       X---------> Axis -->
    //     B2       =>  B2
    TEST_FIXTURE (Fixture_dxJointPU_B1_and_B2_At_Zero,
                  test_dJointSetPUAxisOffset_B2_OffsetUnit)
    {
        dJointAttach (jId, 0, bId2);

        CHECK_CLOSE (0.0, dJointGetPUPosition (jId), 1e-4);

        dBodySetPosition (bId2, offset, 0, 0);

        CHECK_CLOSE (-offset, dJointGetPUPosition (jId), 1e-4);

        dVector3 axis;
        dJointGetPUAxisP (jId, axis);
        dJointSetPUAnchorOffset (jId, 0, 0, 0,
                                 -offset*axis[0], -offset*axis[1], -offset*axis[2]);
        CHECK_CLOSE (-offset, dJointGetPUPosition (jId), 1e-4);

        dBodySetPosition (bId2, 0, 0, 0);
        CHECK_CLOSE (0.0, dJointGetPUPosition (jId), 1e-4);
    }

    // Attache only one body at position 2 to the joint dJointAttach (jId, 0, bId)
    // Move 1st body offset unit in the opposite X direction
    //
    //  X------->          X---------> Axis -->
    //  B2          =>  B2
    //
    // Start with a Offset of -offset unit
    //
    //      X------->      X---------> Axis -->
    //  B2            =>   B2
    TEST_FIXTURE (Fixture_dxJointPU_B1_and_B2_At_Zero,
                  test_dJointSetPUAxisOffset_B2_Minus_OffsetUnit)
    {
        dJointAttach (jId, 0, bId2);

        CHECK_CLOSE (0.0, dJointGetPUPosition (jId), 1e-4);

        dBodySetPosition (bId2, -offset, 0, 0);

        CHECK_CLOSE (offset, dJointGetPUPosition (jId), 1e-4);

        dVector3 axis;
        dJointGetPUAxisP (jId, axis);
        dJointSetPUAnchorOffset (jId, 0, 0, 0,
                                 offset*axis[0], offset*axis[1], offset*axis[2]);
        CHECK_CLOSE (offset, dJointGetPUPosition (jId), 1e-4);

        dBodySetPosition (bId2, 0, 0, 0);
        CHECK_CLOSE (0.0, dJointGetPUPosition (jId), 1e-4);
    }




    // Only one body
    // The body are positionned at (0, 0, 0), with no rotation
    // The joint is a PU Joint
    // Axis is in the oppsite X axis
    // Anchor at (0, 0, 0)
    // N.B. By default the body is attached at position 1 on the joint
    //      dJointAttach (jId, bId, 0);
    struct Fixture_dxJointPU_One_Body_At_Zero_Axis_Inverse_of_X
    {
        Fixture_dxJointPU_One_Body_At_Zero_Axis_Inverse_of_X()
        {
            wId = dWorldCreate();

            bId = dBodyCreate (wId);
            dBodySetPosition (bId, 0, 0, 0);

            jId   = dJointCreatePU (wId, 0);
            joint = (dxJointPU*) jId;


            dJointAttach (jId, bId, NULL);

            dJointSetPUAxisP (jId, axis[0], axis[1], axis[2]);
        }

        ~Fixture_dxJointPU_One_Body_At_Zero_Axis_Inverse_of_X()
        {
            dWorldDestroy (wId);
        }

        dWorldID wId;

        dBodyID bId;

        dJointID jId;
        dxJointPU* joint;

        static const dVector3 axis;

        static const dReal offset;
    };
    const dVector3 Fixture_dxJointPU_One_Body_At_Zero_Axis_Inverse_of_X::axis =
    {
        -1, 0, 0
    };
    const dReal    Fixture_dxJointPU_One_Body_At_Zero_Axis_Inverse_of_X::offset = REAL (3.1);


    // Move 1st body offset unit in the X direction
    //
    //  X------->       X--------->  <--- Axis
    //  B1          =>     B1
    //
    // Start with a Offset of offset unit
    //
    //  X------->       X--------->  <--- Axis
    //     B1       =>  B1
    TEST_FIXTURE (Fixture_dxJointPU_One_Body_At_Zero_Axis_Inverse_of_X,
                  test_dJointSetPUAxisOffset_B1_At_Position_1_OffsetUnit)
    {
        CHECK_CLOSE (0.0, dJointGetPUPosition (jId), 1e-4);

        dBodySetPosition (bId, offset, 0, 0);

        CHECK_CLOSE (-offset, dJointGetPUPosition (jId), 1e-4);

        dJointSetPUAnchorOffset (jId, 0, 0, 0,
                                 -offset*axis[0],-offset*axis[1],-offset*axis[2]);
        CHECK_CLOSE (-offset, dJointGetPUPosition (jId), 1e-4);

        dBodySetPosition (bId, 0, 0, 0);
        CHECK_CLOSE (0.0, dJointGetPUPosition (jId), 1e-4);
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
    TEST_FIXTURE (Fixture_dxJointPU_One_Body_At_Zero_Axis_Inverse_of_X,
                  test_dJointSetPUAxisOffset_B1_Minus_OffsetUnit)
    {
        CHECK_CLOSE (0.0, dJointGetPUPosition (jId), 1e-4);

        dBodySetPosition (bId, -offset, 0, 0);

        CHECK_CLOSE (offset, dJointGetPUPosition (jId), 1e-4);

        dJointSetPUAnchorOffset (jId, 0, 0, 0,
                                 offset*axis[0],offset*axis[1],offset*axis[2]);
        CHECK_CLOSE (offset, dJointGetPUPosition (jId), 1e-4);

        dBodySetPosition (bId, 0, 0, 0);
        CHECK_CLOSE (0.0, dJointGetPUPosition (jId), 1e-4);
    }


    // Move 1st body offset unit in the X direction
    //
    //  X------->       X--------->  <--- Axis
    //  B2          =>     B2
    //
    // Start with a Offset of offset unit
    //
    //  X------->       X--------->  <--- Axis
    //     B2       =>  B2
    TEST_FIXTURE (Fixture_dxJointPU_One_Body_At_Zero_Axis_Inverse_of_X,
                  test_dJointSetPUAxisOffset_B2_OffsetUnit)
    {
        // By default it is attached to position 1
        // Now attach the body at positiojn 2
        dJointAttach(jId, 0, bId);

        CHECK_CLOSE (0.0, dJointGetPUPosition (jId), 1e-4);

        dBodySetPosition (bId, offset, 0, 0);

        CHECK_CLOSE (offset, dJointGetPUPosition (jId), 1e-4);

        dJointSetPUAnchorOffset (jId, 0, 0, 0,
                                 offset*axis[0], offset*axis[1], offset*axis[2]);
        CHECK_CLOSE (offset, dJointGetPUPosition (jId), 1e-4);

        dBodySetPosition (bId, 0, 0, 0);
        CHECK_CLOSE (0.0, dJointGetPUPosition (jId), 1e-4);
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
    TEST_FIXTURE (Fixture_dxJointPU_One_Body_At_Zero_Axis_Inverse_of_X,
                  test_dJointSetPUAxisOffset_B2_Minus_OffsetUnit)
    {
        // By default it is attached to position 1
        // Now attach the body at positiojn 2
        dJointAttach(jId, 0, bId);

        CHECK_CLOSE (0.0, dJointGetPUPosition (jId), 1e-4);

        dBodySetPosition (bId, -offset, 0, 0);

        CHECK_CLOSE (-offset, dJointGetPUPosition (jId), 1e-4);

        dJointSetPUAnchorOffset (jId, 0, 0, 0,
                                 -offset*axis[0], -offset*axis[1], -offset*axis[2]);
        CHECK_CLOSE (-offset, dJointGetPUPosition (jId), 1e-4);

        dBodySetPosition (bId, 0, 0, 0);
        CHECK_CLOSE (0.0, dJointGetPUPosition (jId), 1e-4);
    }









    // Compare only one body to 2 bodies with one fixed.
    //
    // The body are positionned at (0, 0, 0), with no rotation
    // The joint is a PU Joint with default values
    struct Fixture_dxJointPU_Compare_One_Body_To_Two_Bodies_At_Zero
    {
        Fixture_dxJointPU_Compare_One_Body_To_Two_Bodies_At_Zero()
        {
            wId = dWorldCreate();

            bId1_12 = dBodyCreate (wId);
            dBodySetPosition (bId1_12, 0, 0, 0);

            bId2_12 = dBodyCreate (wId);
            dBodySetPosition (bId2_12, 0, 0, 0);
            // The force will be added in the function since it is not
            // always on the same body

            jId_12 = dJointCreatePU (wId, 0);
            dJointAttach(jId_12, bId1_12, bId2_12);

            fixed = dJointCreateFixed (wId, 0);



            jId = dJointCreatePU (wId, 0);

            bId = dBodyCreate (wId);
            dBodySetPosition (bId, 0, 0, 0);

            // Linear velocity along the prismatic axis;
            dVector3 axis;
            dJointGetPUAxisP(jId_12, axis);
            dJointSetPUAxisP(jId, axis[0], axis[1], axis[2]);
            dBodySetLinearVel (bId, magnitude*axis[0], magnitude*axis[1], magnitude*axis[2]);
        }

        ~Fixture_dxJointPU_Compare_One_Body_To_Two_Bodies_At_Zero()
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

        static const dReal magnitude;
    };
    const dReal Fixture_dxJointPU_Compare_One_Body_To_Two_Bodies_At_Zero::magnitude = REAL (4.27);


    TEST_FIXTURE (Fixture_dxJointPU_Compare_One_Body_To_Two_Bodies_At_Zero,
                  test_dJointSetPUPositionRate_Only_B1)
    {
        // Linear velocity along the prismatic axis;
        dVector3 axis;
        dJointGetPUAxisP(jId_12, axis);
        dBodySetLinearVel (bId1_12, magnitude*axis[0], magnitude*axis[1], magnitude*axis[2]);

        dJointAttach(jId_12, bId1_12, bId2_12);

        dJointAttach(fixed, 0, bId2_12);
        dJointSetFixed(fixed);

        dJointAttach(jId, bId, 0);

        CHECK_CLOSE(dJointGetPUPositionRate(jId_12), dJointGetPUPositionRate(jId), 1e-2);
    }


    TEST_FIXTURE (Fixture_dxJointPU_Compare_One_Body_To_Two_Bodies_At_Zero,
                  test_dJointSetPUPositionRate_Only_B2)
    {
        // Linear velocity along the prismatic axis;
        dVector3 axis;
        dJointGetPUAxisP(jId_12, axis);
        dBodySetLinearVel (bId2_12, magnitude*axis[0], magnitude*axis[1], magnitude*axis[2]);

        dJointAttach(jId_12, bId1_12, bId2_12);

        dJointAttach(fixed, bId1_12, 0);
        dJointSetFixed(fixed);

        dJointAttach(jId, 0, bId);

        CHECK_CLOSE(dJointGetPUPositionRate(jId_12), dJointGetPUPositionRate(jId), 1e-2);
    }








    // This test compare the result of a pu joint with 2 bodies where body body 2 is
    // fixed to the world to a pu joint with only one body at position 1.
    //
    // Test the limits [-1, 0.25] when only one body at is attached to the joint
    // using dJointAttache(jId, bId, 0);
    //
    TEST_FIXTURE(Fixture_dxJointPU_Compare_One_Body_To_Two_Bodies_At_Zero,
                 test_Limit_minus1_025_One_Body_on_left)
    {
        dVector3 axis;
        dJointGetPUAxisP(jId_12, axis);
        dJointSetPUAxisP(jId, axis[0], axis[1], axis[2]);
        dBodySetLinearVel (bId1_12, magnitude*axis[0], magnitude*axis[1], magnitude*axis[2]);

        dJointAttach(jId_12, bId1_12, bId2_12);
        dJointSetPUParam(jId_12, dParamLoStop3, -1);
        dJointSetPUParam(jId_12, dParamHiStop3, 0.25);

        dJointAttach(fixed, 0, bId2_12);
        dJointSetFixed(fixed);

        dJointAttach(jId, bId, 0);
        dJointSetPUParam(jId, dParamLoStop3, -1);
        dJointSetPUParam(jId, dParamHiStop3, 0.25);


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

        // Should be different than zero
        CHECK( dJointGetPUPosition(jId_12) );
        CHECK( dJointGetPUPosition(jId) );

        CHECK( dJointGetPUPositionRate(jId_12) );
        CHECK( dJointGetPUPositionRate(jId) );
    }



    // This test compare the result of a pu joint with 2 bodies where body body 1 is
    // fixed to the world to a pu joint with only one body at position 2.
    //
    // Test the limits [-1, 0.25] when only one body at is attached to the joint
    // using dJointAttache(jId, 0, bId);
    //
    TEST_FIXTURE(Fixture_dxJointPU_Compare_One_Body_To_Two_Bodies_At_Zero,
                 test_Limit_minus1_025_One_Body_on_right)
    {
        dVector3 axis;
        dJointGetPUAxisP(jId_12, axis);
        dJointSetPUAxisP(jId, axis[0], axis[1], axis[2]);
        dBodySetLinearVel (bId2_12, magnitude*axis[0], magnitude*axis[1], magnitude*axis[2]);

        dJointAttach(jId_12, bId1_12, bId2_12);
        dJointSetPUParam(jId_12, dParamLoStop3, -1);
        dJointSetPUParam(jId_12, dParamHiStop3, 0.25);

        dJointAttach(fixed, bId1_12, 0);
        dJointSetFixed(fixed);


        dJointAttach(jId, 0, bId);
        dJointSetPUParam(jId, dParamLoStop3, -1);
        dJointSetPUParam(jId, dParamHiStop3, 0.25);

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

        // Should be different than zero
        CHECK( dJointGetPUPosition(jId_12) );
        CHECK( dJointGetPUPosition(jId) );

        CHECK( dJointGetPUPositionRate(jId_12) );
        CHECK( dJointGetPUPositionRate(jId) );
    }



    // This test compare the result of a pu joint with 2 bodies where body 2 is
    // fixed to the world to a pu joint with only one body at position 1.
    //
    // Test the limits [0, 0] when only one body at is attached to the joint
    // using dJointAttache(jId, bId, 0);
    //
    // The body should not move since their is no room between the two limits
    //
    TEST_FIXTURE(Fixture_dxJointPU_Compare_One_Body_To_Two_Bodies_At_Zero,
                 test_Limit_0_0_One_Body_on_left)
    {
        dVector3 axis;
        dJointGetPUAxisP(jId_12, axis);
        dJointSetPUAxisP(jId, axis[0], axis[1], axis[2]);
        dBodySetLinearVel (bId1_12, magnitude*axis[0], magnitude*axis[1], magnitude*axis[2]);

        dJointAttach(jId_12, bId1_12, bId2_12);
        dJointSetPUParam(jId_12, dParamLoStop3, 0);
        dJointSetPUParam(jId_12, dParamHiStop3, 0);

        dJointAttach(fixed, 0, bId2_12);
        dJointSetFixed(fixed);


        dJointAttach(jId, bId, 0);
        dJointSetPUParam(jId, dParamLoStop3, 0);
        dJointSetPUParam(jId, dParamHiStop3, 0);

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


    // This test compare the result of a pu joint with 2 bodies where body body 1 is
    // fixed to the world to a pu joint with only one body at position 2.
    //
    // Test the limits [0, 0] when only one body at is attached to the joint
    // using dJointAttache(jId, 0, bId);
    //
    // The body should not move since their is no room between the two limits
    //
    TEST_FIXTURE(Fixture_dxJointPU_Compare_One_Body_To_Two_Bodies_At_Zero,
                 test_Limit_0_0_One_Body_on_right)
    {
        dVector3 axis;
        dJointGetPUAxisP(jId_12, axis);
        dJointSetPUAxisP(jId, axis[0], axis[1], axis[2]);
        dBodySetLinearVel (bId2_12, magnitude*axis[0], magnitude*axis[1], magnitude*axis[2]);

        dJointAttach(jId_12, bId1_12, bId2_12);
        dJointSetPUParam(jId_12, dParamLoStop3, 0);
        dJointSetPUParam(jId_12, dParamHiStop3, 0);

        dJointAttach(fixed, bId1_12, 0);
        dJointSetFixed(fixed);


        dJointAttach(jId, 0, bId);
        dJointSetPUParam(jId, dParamLoStop3, 0);
        dJointSetPUParam(jId, dParamHiStop3, 0);

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


} // End of SUITE TestdxJointPU

