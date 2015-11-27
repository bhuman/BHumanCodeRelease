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

////////////////////////////////////////////////////////////////////////////////
// This file create unit test for some of the functions found in:
// ode/src/joinst/universal.cpp
//
//
////////////////////////////////////////////////////////////////////////////////

#include <iostream>

#include <UnitTest++.h>
#include <ode/ode.h>

#include "../../ode/src/config.h"
#include "../../ode/src/joints/universal.h"

dReal d2r(dReal degree)
{
    return degree * (dReal)(M_PI / 180.0);
}
dReal r2d(dReal degree)
{
    return degree * (dReal)(180.0/M_PI);
}

SUITE (TestdxJointUniversal)
{
    // The 2 bodies are positionned at (0, 0, 0)
    // The bodis have no rotation.
    // The joint is a Universal Joint
    // Axis1 is along the X axis
    // Axis2 is along the Y axis
    // Anchor at (0, 0, 0)
    struct Fixture_dxJointUniversal_B1_and_B2_At_Zero_Axis1_Along_X_Axis2_Along_Y
    {
        Fixture_dxJointUniversal_B1_and_B2_At_Zero_Axis1_Along_X_Axis2_Along_Y()
        {

            wId = dWorldCreate();

            bId1 = dBodyCreate (wId);
            dBodySetPosition (bId1, 0, 0, 0);

            bId2 = dBodyCreate (wId);
            dBodySetPosition (bId2, 0, 0, 0);


            jId   = dJointCreateUniversal (wId, 0);
            joint = (dxJointUniversal*) jId;


            dJointAttach (jId, bId1, bId2);
        }

        ~Fixture_dxJointUniversal_B1_and_B2_At_Zero_Axis1_Along_X_Axis2_Along_Y()
        {
            dWorldDestroy (wId);
        }

        dWorldID wId;

        dBodyID bId1;
        dBodyID bId2;


        dJointID jId;
        dxJointUniversal* joint;
    };


    // The 2 bodies are positionned at (-1, -2, -3),  and (11, 22, 33)
    // The bodis have rotation of 27deg around some axis.
    // The joint is a Universal Joint
    // Axis is along the X axis
    // Anchor at (0, 0, 0)
    struct Fixture_dxJointUniversal_B1_and_B2_At_Random_Axis_Along_X
    {
        Fixture_dxJointUniversal_B1_and_B2_At_Random_Axis_Along_X()
        {
            wId = dWorldCreate();

            bId1 = dBodyCreate (wId);
            dBodySetPosition (bId1, -1, -2, -3);

            bId2 = dBodyCreate (wId);
            dBodySetPosition (bId2, 11, 22, 33);

            dMatrix3 R;

            dVector3 axis;

            axis[0] =  REAL(0.53);
            axis[1] = -REAL(0.71);
            axis[2] =  REAL(0.43);
            dNormalize3(axis);
            dRFromAxisAndAngle (R, axis[0], axis[1], axis[2],
                                REAL(0.47123)); // 27deg
            dBodySetRotation (bId1, R);


            axis[0] =  REAL(1.2);
            axis[1] =  REAL(0.87);
            axis[2] = -REAL(0.33);
            dNormalize3(axis);
            dRFromAxisAndAngle (R, axis[0], axis[1], axis[2],
                                REAL(0.47123)); // 27deg
            dBodySetRotation (bId2, R);

            jId   = dJointCreateUniversal (wId, 0);
            joint = (dxJointUniversal*) jId;


            dJointAttach (jId, bId1, bId2);
        }

        ~Fixture_dxJointUniversal_B1_and_B2_At_Random_Axis_Along_X()
        {
            dWorldDestroy (wId);
        }

        dWorldID wId;

        dBodyID bId1;
        dBodyID bId2;


        dJointID jId;
        dxJointUniversal* joint;
    };


    // Only one body body1 at (0,0,0)
    // The joint is an Universal Joint.
    // Axis1 is along the X axis
    // Axis2 is along the Y axis
    // Anchor at (0, 0, 0)
    //
    //       ^Y
    //       |
    //       |
    //       |
    //       |
    //       |
    // Z <-- X
    struct Fixture_dxJointUniversal_B1_At_Zero_Default_Axes
    {
        Fixture_dxJointUniversal_B1_At_Zero_Default_Axes()
        {
            wId = dWorldCreate();

            bId1 = dBodyCreate (wId);
            dBodySetPosition (bId1, 0, 0, 0);

            jId   = dJointCreateUniversal (wId, 0);


            dJointAttach (jId, bId1, NULL);
            dJointSetUniversalAnchor (jId, 0, 0, 0);
        }

        ~Fixture_dxJointUniversal_B1_At_Zero_Default_Axes()
        {
            dWorldDestroy (wId);
        }

        dWorldID wId;

        dBodyID bId1;


        dJointID jId;
    };



    // Only one body body2 at (0,0,0)
    // The joint is an Universal Joint.
    // Axis1 is along the X axis.
    // Axis2 is along the Y axis.
    // Anchor at (0, 0, 0)
    //
    //       ^Y
    //       |
    //       |
    //       |
    //       |
    //       |
    // Z <-- X
    struct Fixture_dxJointUniversal_B2_At_Zero_Default_Axes
    {
        Fixture_dxJointUniversal_B2_At_Zero_Default_Axes()
        {
            wId = dWorldCreate();

            bId2 = dBodyCreate (wId);
            dBodySetPosition (bId2, 0, 0, 0);

            jId   = dJointCreateUniversal (wId, 0);


            dJointAttach (jId, NULL, bId2);
            dJointSetUniversalAnchor (jId, 0, 0, 0);
        }

        ~Fixture_dxJointUniversal_B2_At_Zero_Default_Axes()
        {
            dWorldDestroy (wId);
        }

        dWorldID wId;

        dBodyID bId2;

        dJointID jId;
    };


    // Test is dJointGetUniversalAngles versus
    // dJointGetUniversalAngle1 and dJointGetUniversalAngle2 dJointGetUniversalAxis
    TEST_FIXTURE (Fixture_dxJointUniversal_B1_and_B2_At_Zero_Axis1_Along_X_Axis2_Along_Y,
                  test_dJointSetGetUniversalAngles_Versus_Angle1_and_Angle2)
    {
        CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

        dReal angle1, angle2;
        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (0, angle1, 1e-4);
        CHECK_CLOSE (0, angle2, 1e-4);

        dMatrix3 R;
        dReal ang1, ang2;


        dVector3 axis1;
        dJointGetUniversalAxis1 (jId, axis1);

        dVector3 axis2;
        dJointGetUniversalAxis2 (jId, axis2);

        ang1 = d2r(REAL(23.0));
        dRFromAxisAndAngle (R, axis1[0], axis1[1], axis1[2], ang1);
        dBodySetRotation (bId1, R);

        ang2 = d2r(REAL(17.0));
        dRFromAxisAndAngle (R, axis2[0], axis2[1], axis2[2], ang2);
        dBodySetRotation (bId2, R);

        CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (ang1, angle1, 1e-4);
        CHECK_CLOSE (-ang2, angle2, 1e-4);






        // ax1 and ax2 are pseudo-random axis. N.B. They are NOT the axis of the joints.
        dVector3 ax1;
        ax1[0] =  REAL(0.2);
        ax1[1] = -REAL(0.67);
        ax1[2] = -REAL(0.81);
        dNormalize3(ax1);

        dVector3 ax2;
        ax2[0] = REAL(0.62);
        ax2[1] = REAL(0.31);
        ax2[2] = REAL(0.43);
        dNormalize3(ax2);


        ang1 = d2r(REAL(23.0));
        dRFromAxisAndAngle (R, ax1[0], ax1[1], ax1[2], ang1);
        dBodySetRotation (bId1, R);

        ang2 = d2r(REAL(0.0));

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (angle1, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (angle2, dJointGetUniversalAngle2 (jId), 1e-4);



        ang1 = d2r(REAL(0.0));

        ang2 = d2r(REAL(23.0));
        dRFromAxisAndAngle (R, ax2[0], ax2[1], ax2[2], ang2);
        dBodySetRotation (bId1, R);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (angle1, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (angle2, dJointGetUniversalAngle2 (jId), 1e-4);


        ang1 = d2r(REAL(38.0));
        dRFromAxisAndAngle (R, ax1[0], ax1[1], ax1[2], ang2);
        dBodySetRotation (bId1, R);

        ang2 = d2r(REAL(-43.0));
        dRFromAxisAndAngle (R, ax2[0], ax2[1], ax2[2], ang2);
        dBodySetRotation (bId1, R);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (angle1, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (angle2, dJointGetUniversalAngle2 (jId), 1e-4);


        // Try with random axis for the axis of the joints
        dRSetIdentity(R);
        dBodySetRotation (bId1, R);
        dBodySetRotation (bId1, R);

        axis1[0] =  REAL(0.32);
        axis1[1] = -REAL(0.57);
        axis1[2] =  REAL(0.71);
        dNormalize3(axis1);

        axis2[0] = -REAL(0.26);
        axis2[1] = -REAL(0.31);
        axis2[2] =  REAL(0.69);
        dNormalize3(axis2);

        dVector3 cross;
        dCalcVectorCross3(cross, axis1, axis2);
        dJointSetUniversalAxis1(jId, axis1[0], axis1[1], axis1[2]);
        dJointSetUniversalAxis2(jId, cross[0], cross[1], cross[2]);


        ang1 = d2r(REAL(23.0));
        dRFromAxisAndAngle (R, ax1[0], ax1[1], ax1[2], ang1);
        dBodySetRotation (bId1, R);

        ang2 = d2r(REAL(0.0));

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (angle1, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (angle2, dJointGetUniversalAngle2 (jId), 1e-4);



        ang1 = d2r(REAL(0.0));

        ang2 = d2r(REAL(23.0));
        dRFromAxisAndAngle (R, ax2[0], ax2[1], ax2[2], ang2);
        dBodySetRotation (bId1, R);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (angle1, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (angle2, dJointGetUniversalAngle2 (jId), 1e-4);


        ang1 = d2r(REAL(38.0));
        dRFromAxisAndAngle (R, ax1[0], ax1[1], ax1[2], ang2);
        dBodySetRotation (bId1, R);

        ang2 = d2r(REAL(-43.0));
        dRFromAxisAndAngle (R, ax2[0], ax2[1], ax2[2], ang2);
        dBodySetRotation (bId1, R);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (angle1, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (angle2, dJointGetUniversalAngle2 (jId), 1e-4);
    }


    // =========================================================================
    // Test ONE BODY behavior
    // =========================================================================


    // Test when there is only one body at position one on the joint
    TEST_FIXTURE (Fixture_dxJointUniversal_B1_At_Zero_Default_Axes,
                  test_dJointGetUniversalAngle1_1Body_B1)
    {
        CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

        dReal angle1, angle2;
        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (0, angle1, 1e-4);
        CHECK_CLOSE (0, angle2, 1e-4);

        dVector3 axis1;
        dJointGetUniversalAxis1 (jId, axis1);
        dVector3 axis2;
        dJointGetUniversalAxis2 (jId, axis2);

        dMatrix3 R;

        dReal ang1 = REAL(0.23);
        dRFromAxisAndAngle (R, axis1[0], axis1[1], axis1[2], ang1);
        dBodySetRotation (bId1, R);

        dReal ang2 = REAL(0.0);


        CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (ang1, angle1, 1e-4);
        CHECK_CLOSE (-ang2, angle2, 1e-4);



        dMatrix3 I;
        dRSetIdentity(I); // Set the rotation of the body to be the Identity (i.e. zero)
        dBodySetRotation (bId1, I);

        CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);


        // Test the same rotation, when axis1 is inverted
        dJointSetUniversalAxis1 (jId, -axis1[0], -axis1[1], -axis1[2]);

        dBodySetRotation (bId1, R);

        CHECK_CLOSE (-ang1, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (-ang1, angle1, 1e-4);
        CHECK_CLOSE (-ang2, angle2, 1e-4);


        // Test the same rotation, when axis1 is default and axis2 is inverted
        dBodySetRotation (bId1, I);

        dJointSetUniversalAxis1 (jId, axis1[0], axis1[1], axis1[2]);
        dJointSetUniversalAxis2 (jId, -axis2[0], -axis2[1], -axis2[2]);


        dBodySetRotation (bId1, R);

        CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (ang2, dJointGetUniversalAngle2 (jId), 1e-4);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (ang1, angle1, 1e-4);
        CHECK_CLOSE (ang2, angle2, 1e-4);
    }




    // Test when there is only one body at position two on the joint
    TEST_FIXTURE (Fixture_dxJointUniversal_B2_At_Zero_Default_Axes,
                  test_dJointGetUniversalAngle1_1Body_B2)
    {
        CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

        dReal angle1, angle2;
        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (0, angle1, 1e-4);
        CHECK_CLOSE (0, angle2, 1e-4);

        dVector3 axis1;
        dJointGetUniversalAxis1 (jId, axis1);

        dVector3 axis2;
        dJointGetUniversalAxis2 (jId, axis2);

        dMatrix3 R;

        dReal ang1 = REAL(0.0);

        dReal ang2 = REAL(0.23);
        dRFromAxisAndAngle (R, axis2[0], axis2[1], axis2[2], ang2);
        dBodySetRotation (bId2, R);



        CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (ang1, angle1, 1e-4);
        CHECK_CLOSE (-ang2, angle2, 1e-4);



        dMatrix3 I;
        dRSetIdentity(I); // Set the rotation of the body to be the Identity (i.e. zero)
        dBodySetRotation (bId2, I);

        CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);


        dJointSetUniversalAxis2 (jId, -axis2[0], -axis2[1], -axis2[2]);

        dBodySetRotation (bId2, R);

        CHECK_CLOSE (-ang1, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (ang2, dJointGetUniversalAngle2 (jId), 1e-4);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (-ang1, angle1, 1e-4);
        CHECK_CLOSE (ang2, angle2, 1e-4);

        // Test the same rotation, when axis1 is inverted and axis2 is default
        dBodySetRotation (bId2, I);

        dJointSetUniversalAxis1 (jId, -axis1[0], -axis1[1], -axis1[2]);
        dJointSetUniversalAxis2 (jId, axis2[0], axis2[1], axis2[2]);


        dBodySetRotation (bId2, R);

        CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (ang1, angle1, 1e-4);
        CHECK_CLOSE (-ang2, angle2, 1e-4);
    }






    // =========================================================================
    //
    // =========================================================================


    // Test is dJointSetUniversalAxis and dJointGetUniversalAxis return same value
    TEST_FIXTURE (Fixture_dxJointUniversal_B1_and_B2_At_Random_Axis_Along_X,
                  test_dJointSetGetUniversalAxis)
    {
        dVector3 axisOrig, axis;


        dJointGetUniversalAxis1 (jId, axisOrig);
        dJointGetUniversalAxis1 (jId, axis);
        dJointSetUniversalAxis1 (jId, axis[0], axis[1], axis[2]);
        dJointGetUniversalAxis1 (jId, axis);
        CHECK_CLOSE (axis[0], axisOrig[0] , 1e-4);
        CHECK_CLOSE (axis[1], axisOrig[1] , 1e-4);
        CHECK_CLOSE (axis[2], axisOrig[2] , 1e-4);


        dJointGetUniversalAxis2 (jId, axisOrig);
        dJointGetUniversalAxis2(jId, axis);
        dJointSetUniversalAxis2 (jId, axis[0], axis[1], axis[2]);
        dJointGetUniversalAxis2 (jId, axis);
        CHECK_CLOSE (axis[0], axisOrig[0] , 1e-4);
        CHECK_CLOSE (axis[1], axisOrig[1] , 1e-4);
        CHECK_CLOSE (axis[2], axisOrig[2] , 1e-4);


        dVector3 anchor1, anchor2, anchorOrig1, anchorOrig2;
        dJointGetUniversalAnchor (jId, anchorOrig1);
        dJointGetUniversalAnchor (jId, anchor1);
        dJointGetUniversalAnchor2 (jId, anchorOrig2);
        dJointGetUniversalAnchor2 (jId, anchor2);

        dJointSetUniversalAnchor (jId, anchor1[0], anchor1[1], anchor1[2]);
        dJointGetUniversalAnchor (jId, anchor1);
        dJointGetUniversalAnchor2 (jId, anchor2);
        CHECK_CLOSE (anchor1[0], anchorOrig1[0] , 1e-4);
        CHECK_CLOSE (anchor1[1], anchorOrig1[1] , 1e-4);
        CHECK_CLOSE (anchor1[2], anchorOrig1[2] , 1e-4);

        CHECK_CLOSE (anchor2[0], anchorOrig2[0] , 1e-4);
        CHECK_CLOSE (anchor2[1], anchorOrig2[1] , 1e-4);
        CHECK_CLOSE (anchor2[2], anchorOrig2[2] , 1e-4);
    }



    // Create 2 bodies attached by a Universal joint
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
    struct dxJointUniversal_Test_Initialization
    {
        dxJointUniversal_Test_Initialization()
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

                jId[j]   = dJointCreateUniversal (wId, 0);
                dJointAttach (jId[j], bId[j][0], bId[j][1]);
                dJointSetUniversalParam(jId[j], dParamLoStop, 1);
                dJointSetUniversalParam(jId[j], dParamHiStop, 2);
                dJointSetUniversalParam(jId[j], dParamFMax, 200);
            }
        }

        ~dxJointUniversal_Test_Initialization()
        {
            dWorldDestroy (wId);
        }

        dWorldID wId;

        dBodyID bId[2][2];


        dJointID jId[2];

    };


    // Test if setting a Universal with its default values
    // will behave the same as a default Universal joint
    TEST_FIXTURE (dxJointUniversal_Test_Initialization,
                  test_Universal_Initialization)
    {
        using namespace std;

        dVector3 axis;
        dJointGetUniversalAxis1(jId[1], axis);
        dJointSetUniversalAxis1(jId[1], axis[0], axis[1], axis[2]);

        dJointGetUniversalAxis2(jId[1], axis);
        dJointSetUniversalAxis2(jId[1], axis[0], axis[1], axis[2]);

        dVector3 anchor;
        dJointGetUniversalAnchor(jId[1], anchor);
        dJointSetUniversalAnchor(jId[1], anchor[0], anchor[1], anchor[2]);


        for (int b=0; b<2; ++b)
        {
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

        for (int b=0; b<2; ++b)
        {
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













    // ==========================================================================
    // Testing the offset
    // TODO:
    // - Test Axis1Offset(...., 0, ang2);
    // ==========================================================================


    //  Rotate first body 90deg around X (Axis1) then back to original position
    //
    //    ^  ^           ^           Z ^
    //    |  |  => <---  |             |
    //    |  |           |             |
    //   B1  B2     B1   B2            .----->Y
    //                                /
    //                               /
    //                              v X    (N.B. X is going out of the screen)
    //
    //  Set Axis1 with an Offset of 90deg
    //       ^        ^   ^
    //  <--- |  =>    |   |
    //       |        |   |
    //   B1  B2      B1   B2
    TEST_FIXTURE (Fixture_dxJointUniversal_B1_and_B2_At_Zero_Axis1_Along_X_Axis2_Along_Y,
                  test_dJointSetUniversalAxis1Offset_B1_90deg)
    {
        dMatrix3 R;

        CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

        dReal angle1, angle2;
        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (0, angle1, 1e-4);
        CHECK_CLOSE (0, angle2, 1e-4);


        dVector3 axis;
        dJointGetUniversalAxis1 (jId, axis);

        dReal ang1 = d2r(REAL(90.0));
        dReal ang2 = d2r(REAL(0.0));
        dRFromAxisAndAngle (R, axis[0], axis[1], axis[2], ang1);
        dBodySetRotation (bId1, R);

        CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (ang2, dJointGetUniversalAngle2 (jId), 1e-4);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (ang1, angle1, 1e-4);
        CHECK_CLOSE (ang2, angle2, 1e-4);



        dJointSetUniversalAxis1Offset (jId, axis[0], axis[1], axis[2],
                                       ang1, ang2);
        CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (ang2, dJointGetUniversalAngle2 (jId), 1e-4);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (ang1, angle1, 1e-4);
        CHECK_CLOSE (ang2, angle2, 1e-4);


        dRSetIdentity(R); // Set the rotation of the body to be zero
        dBodySetRotation (bId1, R);

        CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (0, angle1, 1e-4);
        CHECK_CLOSE (0, angle2, 1e-4);
    }

    //  Rotate 2nd body 90deg around (Axis2) then back to original position
    //  Offset when setting axis1
    //
    //    ^  ^           ^           Z ^
    //    |  |  => <---  |             |
    //    |  |           |             |
    //   B1  B2     B1   B2            .----->Y
    //                                /
    //                               /
    //                              v X    (N.B. X is going out of the screen)
    //
    //  Set Axis1 with an Offset of 90deg
    //       ^        ^   ^
    //  <--- |  =>    |   |
    //       |        |   |
    //   B1  B2      B1   B2
    TEST_FIXTURE (Fixture_dxJointUniversal_B1_and_B2_At_Zero_Axis1_Along_X_Axis2_Along_Y,
                  test_dJointSetUniversalAxis1Offset_B2_90deg)
    {
        dMatrix3 R;

        CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

        dReal angle1, angle2;
        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (0, angle1, 1e-4);
        CHECK_CLOSE (0, angle2, 1e-4);


        dVector3 ax1, ax2;
        dJointGetUniversalAxis1 (jId, ax1);
        dJointGetUniversalAxis2 (jId, ax2);

        dReal ang1 = d2r(REAL(0.0));
        dReal ang2 = d2r(REAL(90.0));
        dRFromAxisAndAngle (R, ax2[0], ax2[1], ax2[2], ang2);
        dBodySetRotation (bId2, R);

        CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (ang1, angle1, 1e-4);
        CHECK_CLOSE (-ang2, angle2, 1e-4);



        dJointSetUniversalAxis1Offset (jId, ax1[0], ax1[1], ax1[2],
                                       ang1, -ang2);
        CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (ang1, angle1, 1e-4);
        CHECK_CLOSE (-ang2, angle2, 1e-4);


        dRSetIdentity(R); // Set the rotation of the body to be zero
        dBodySetRotation (bId1, R);
        dBodySetRotation (bId2, R);

        CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (0, angle1, 1e-4);
        CHECK_CLOSE (0, angle2, 1e-4);
    }








    //  Rotate second body 90deg around Y (Axis2) then back to original position
    //
    //    ^  ^       ^           Z ^
    //    |  |  =>   |   .         |
    //    |  |       |             |
    //   B1  B2     B1   B2        .----->Y
    //                            /
    //                           /
    //                          v X    (N.B. X is going out of the screen)
    //
    //  Set Axis2 with an Offset of 90deg
    //   ^           ^   ^
    //   |   .  =>   |   |
    //   |           |   |
    //   B1  B2     B1   B2
    TEST_FIXTURE (Fixture_dxJointUniversal_B1_and_B2_At_Zero_Axis1_Along_X_Axis2_Along_Y,
                  test_dJointSetUniversalAxisOffset_B2_90deg)
    {
        CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

        dReal angle1, angle2;
        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (0, angle1, 1e-4);
        CHECK_CLOSE (0, angle2, 1e-4);

        dVector3 axis;
        dJointGetUniversalAxis2 (jId, axis);

        dReal ang1 = d2r(REAL(0.0));
        dReal ang2 = d2r(REAL(90.0));

        dMatrix3 R;
        dRFromAxisAndAngle (R, axis[0], axis[1], axis[2], ang2);
        dBodySetRotation (bId2, R);

        CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (ang1, angle1, 1e-4);
        CHECK_CLOSE (-ang2, angle2, 1e-4);


        dJointSetUniversalAxis2Offset (jId, axis[0], axis[1], axis[2],
                                       ang1, -ang2);
        CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (ang1, angle1, 1e-4);
        CHECK_CLOSE (-ang2, angle2, 1e-4);


        dRSetIdentity(R); // Set the rotation of the body to be zero
        dBodySetRotation (bId2, R);

        CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (0, angle1, 1e-4);
        CHECK_CLOSE (0, angle2, 1e-4);
    }



    //  Rotate 2nd body -90deg around Y (Axis2) then back to original position
    //
    //    ^  ^       ^               Z ^
    //    |  |  =>   |   x             |
    //    |  |       |                 |
    //   B1  B2     B1   B2          X .----> Y
    //                               N.B. X is going out of the screen
    //  Start with a Delta of 90deg
    //    ^           ^   ^
    //    |  x  =>    |   |
    //    |           |   |
    //   B1  B2      B1   B2
    TEST_FIXTURE (Fixture_dxJointUniversal_B1_and_B2_At_Zero_Axis1_Along_X_Axis2_Along_Y,
                  test_dJointSetUniversalAxisOffset_B2_Minus90deg)
    {
        CHECK_CLOSE (dJointGetUniversalAngle1 (jId), 0, 1e-4);
        CHECK_CLOSE (dJointGetUniversalAngle2 (jId), 0, 1e-4);

        dReal angle1, angle2;
        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (0, angle1, 1e-4);
        CHECK_CLOSE (0, angle2, 1e-4);

        dVector3 axis;
        dJointGetUniversalAxis2 (jId, axis);

        dReal ang1 = d2r(REAL(0.0));
        dReal ang2 = d2r(REAL(90.0));

        dMatrix3 R;
        dRFromAxisAndAngle (R, axis[0], axis[1], axis[2], -ang2);
        dBodySetRotation (bId2, R);

        CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (ang2, dJointGetUniversalAngle2 (jId), 1e-4);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (ang1, angle1, 1e-4);
        CHECK_CLOSE (ang2, angle2, 1e-4);



        dJointSetUniversalAxis2Offset (jId, axis[0], axis[1], axis[2],
                                       ang1, ang2);
        CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (ang2, dJointGetUniversalAngle2 (jId), 1e-4);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (ang1, angle1, 1e-4);
        CHECK_CLOSE (ang2, angle2, 1e-4);


        dRFromAxisAndAngle (R, axis[0], axis[1], axis[2], 0);
        dBodySetRotation (bId2, R);

        CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (0, angle1, 1e-4);
        CHECK_CLOSE (0, angle2, 1e-4);
    }


    //  Rotate 1st body 0.23rad around X (Axis1) then back to original position
    //
    //    ^  ^     ^      ^           Z ^
    //    |  |  =>  \     |             |
    //    |  |       \    |             |
    //   B1  B2     B1   B2             .-------> Y
    //                                 /
    //                                /
    //                               v X  (N.B. X is going out of the screen)
    //
    //  Start with a Delta of 0.23rad
    //  ^    ^        ^   ^
    //   \   | =>     |   |
    //    \  |        |   |
    //   B1  B2      B1   B2
    TEST_FIXTURE (Fixture_dxJointUniversal_B1_and_B2_At_Zero_Axis1_Along_X_Axis2_Along_Y,
                  test_dJointSetUniversalAxis1Offset_B1_0_23rad)
    {
        CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

        dReal angle1, angle2;
        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (0, angle1, 1e-4);
        CHECK_CLOSE (0, angle2, 1e-4);

        dVector3 axis;
        dJointGetUniversalAxis1 (jId, axis);

        dReal ang1 = REAL(0.23);
        dReal ang2 = REAL(0.0);

        dMatrix3 R;
        dRFromAxisAndAngle (R, axis[0], axis[1], axis[2], ang1);
        dBodySetRotation (bId1, R);

        CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (ang2, dJointGetUniversalAngle2 (jId), 1e-4);


        dJointSetUniversalAxis1Offset (jId, axis[0], axis[1], axis[2],
                                       ang1, ang2);
        CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (ang2, dJointGetUniversalAngle2 (jId), 1e-4);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (ang1, angle1, 1e-4);
        CHECK_CLOSE (ang2, angle2, 1e-4);


        dRSetIdentity(R); // Set the rotation of the body to be zero
        dBodySetRotation (bId1, R);

        CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (0, angle1, 1e-4);
        CHECK_CLOSE (0, angle2, 1e-4);
    }

    //  Rotate 2nd body 0.23rad around Y (Axis2) then back to original position
    //
    //    ^  ^      ^     ^           Z ^   ^ Y (N.B. Y is going in the screen)
    //    |  |  =>  |    /              |  /
    //    |  |      |   /               | /
    //   B1  B2     B1  B2              .-------> X
    //
    //  Start with a Delta of 0.23rad
    //   ^     ^    ^   ^
    //   |    /  => |   |
    //   |   /      |   |
    //   B1  B2     B1  B2
    TEST_FIXTURE (Fixture_dxJointUniversal_B1_and_B2_At_Zero_Axis1_Along_X_Axis2_Along_Y,
                  test_dJointSetUniversalAxisOffset_B2_0_23rad)
    {
        CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

        dReal angle1, angle2;
        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (0, angle1, 1e-4);
        CHECK_CLOSE (0, angle2, 1e-4);

        dVector3 axis;
        dJointGetUniversalAxis2 (jId, axis);

        dReal ang1 = REAL(0.0);
        dReal ang2 = REAL(0.23);

        dMatrix3 R;
        dRFromAxisAndAngle (R, axis[0], axis[1], axis[2], ang2);
        dBodySetRotation (bId2, R);

        CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);


        dJointSetUniversalAxis2Offset (jId, axis[0], axis[1], axis[2],
                                       ang1, -ang2);
        CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (ang1, angle1, 1e-4);
        CHECK_CLOSE (-ang2, angle2, 1e-4);


        dRSetIdentity(R); // Set the rotation of the body to be zero
        dBodySetRotation (bId2, R);

        CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (0, angle1, 1e-4);
        CHECK_CLOSE (0, angle2, 1e-4);
    }


    // Rotate 1st body 0.23rad around X axis and 2nd body 0.37rad around Y (Axis2)
    // then back to their original position.
    // The Axis offset are set one at a time
    //
    //    ^  ^    ^         ^          Z ^   ^ Y (N.B. Y is going in the screen)
    //    |  |  => \      /             |  /
    //    |  |      \   /               | /
    //   B1  B2     B1  B2              .-------> X
    //
    //  Start with a Delta of 0.23rad
    // ^         ^  ^   ^
    //  \      / => |   |
    //   \   /      |   |
    //   B1  B2     B1  B2
    TEST_FIXTURE (Fixture_dxJointUniversal_B1_and_B2_At_Zero_Axis1_Along_X_Axis2_Along_Y,
                  test_dJointSetUniversalAxisOffset_B1_0_23rad_B2_0_37rad_One_by_One)
    {
        CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

        dReal angle1, angle2;
        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (0, angle1, 1e-4);
        CHECK_CLOSE (0, angle2, 1e-4);

        dVector3 axis1;
        dJointGetUniversalAxis1 (jId, axis1);
        dVector3 axis2;
        dJointGetUniversalAxis2 (jId, axis2);

        dMatrix3 R;

        dReal ang1 = REAL(0.23);
        dRFromAxisAndAngle (R, axis1[0], axis1[1], axis1[2], ang1);
        dBodySetRotation (bId1, R);

        dReal ang2 = REAL(0.37);
        dRFromAxisAndAngle (R, axis2[0], axis2[1], axis2[2], ang2);
        dBodySetRotation (bId2, R);


        CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (ang1, angle1, 1e-4);
        CHECK_CLOSE (-ang2, angle2, 1e-4);


        dJointSetUniversalAxis1Offset (jId, axis1[0], axis1[1], axis1[2],
                                       ang1, -ang2 );
        CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (ang1, angle1, 1e-4);
        CHECK_CLOSE (-ang2, angle2, 1e-4);

        dJointGetUniversalAxis1 (jId, axis1);
        dJointGetUniversalAxis2 (jId, axis2);

        dRFromAxisAndAngle (R, axis2[0], axis2[1], axis2[2], ang2);
        dBodySetRotation (bId2, R);

        dJointSetUniversalAxis2Offset (jId, axis2[0], axis2[1], axis2[2],
                                       ang1, -ang2);
        CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (ang1, angle1, 1e-4);
        CHECK_CLOSE (-ang2, angle2, 1e-4);


        dRSetIdentity(R); // Set the rotation of the body to be zero
        dBodySetRotation (bId1, R);
        dBodySetRotation (bId2, R);

        CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (0, angle1, 1e-4);
        CHECK_CLOSE (0, angle2, 1e-4);
    }



//  The 2 bodies are positionned at (0, 0, 0), with no rotation
//  The joint is an Universal Joint.
//  Axis in the inverse direction of the X axis
//  Anchor at (0, 0, 0)
//          ^Y
//          |
//          |
//          |
//          |
//          |
//  Z <---- x (X going out of the page)
    struct Fixture_dxJointUniversal_B1_and_B2_At_Zero_Axis1_Inverse_of_X
    {
        Fixture_dxJointUniversal_B1_and_B2_At_Zero_Axis1_Inverse_of_X()
        {
            wId = dWorldCreate();

            bId1 = dBodyCreate (wId);
            dBodySetPosition (bId1, 0, 0, 0);

            bId2 = dBodyCreate (wId);
            dBodySetPosition (bId2, 0, 0, 0);

            jId   = dJointCreateUniversal (wId, 0);
            joint = (dxJointUniversal*) jId;


            dJointAttach (jId, bId1, bId2);
            dJointSetUniversalAnchor (jId, 0, 0, 0);

            axis[0] = -1;
            axis[1] = 0;
            axis[2] = 0;
            dJointSetUniversalAxis1(jId, axis[0], axis[1], axis[2]);
        }

        ~Fixture_dxJointUniversal_B1_and_B2_At_Zero_Axis1_Inverse_of_X()
        {
            dWorldDestroy (wId);
        }

        dWorldID wId;

        dBodyID bId1;
        dBodyID bId2;


        dJointID jId;
        dxJointUniversal* joint;

        dVector3 axis;
    };


    // No offset when setting the Axis1 offset
    // x is a Symbol for lines pointing into the screen
    // . is a Symbol for lines pointing out of the screen
    //
    //    In 2D                   In 3D
    //    ^  ^      ^    ^        Z ^   ^ Y
    //    |  |  =>  |    |          |  /
    //    |  |      |    |          | /
    //   B1  B2     B1   B2         .-------> X     <-- Axis1
    //
    //  Start with a Delta of 90deg
    //    ^  ^         ^   ^
    //    |  |    =>  |   |
    //    |  |        |   |
    //   B1  B2      B1   B2
    TEST_FIXTURE (Fixture_dxJointUniversal_B1_and_B2_At_Zero_Axis1_Inverse_of_X,
                  test_dJointSetUniversalAxis1Offset_No_Offset_Axis1_Inverse_of_X)
    {
        CHECK_CLOSE (dJointGetUniversalAngle1 (jId), 0, 1e-4);
        CHECK_CLOSE (dJointGetUniversalAngle2 (jId), 0, 1e-4);

        dReal angle1, angle2;
        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (0, angle1, 1e-4);
        CHECK_CLOSE (0, angle2, 1e-4);

        dVector3 axis;
        dJointGetUniversalAxis1 (jId, axis);

        dReal ang1 = REAL(0.0);
        dReal ang2 = REAL(0.0);

        dMatrix3 R;
        dRFromAxisAndAngle (R, axis[0], axis[1], axis[2], ang1);
        dBodySetRotation (bId1, R);

        CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (ang2, dJointGetUniversalAngle2 (jId), 1e-4);


        dJointSetUniversalAxis1Offset (jId, axis[0], axis[1], axis[2],
                                       ang1, ang2);
        CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (ang2, dJointGetUniversalAngle2 (jId), 1e-4);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (ang1, angle1, 1e-4);
        CHECK_CLOSE (ang2, angle2, 1e-4);

        dRSetIdentity(R); // Set the rotation of the body to be zero
        dBodySetRotation (bId1, R);

        CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (0, angle1, 1e-4);
        CHECK_CLOSE (0, angle2, 1e-4);
    }



    //  Rotate 1st body 90deg around axis1 then back to original position
    //  x is a Symbol for lines pointing into the screen
    //  . is a Symbol for lines pointing out of the screen
    //
    //    In 2D                   In 3D
    //    ^  ^           ^        Z ^   ^ Y
    //    |  |  =>   x   |          |  /
    //    |  |           |          | /
    //   B1  B2     B1   B2         .-------> X     <-- Axis1
    //
    //  Start with a Delta of 90deg
    //       ^         ^   ^
    //    x  |    =>  |   |
    //       |        |   |
    //   B1  B2      B1   B2
    TEST_FIXTURE (Fixture_dxJointUniversal_B1_and_B2_At_Zero_Axis1_Inverse_of_X,
                  test_dJointSetUniversalAxis1Offset_B1_90Deg_Axis1_Inverse_of_X)
    {
        CHECK_CLOSE (dJointGetUniversalAngle1 (jId), 0, 1e-4);
        CHECK_CLOSE (dJointGetUniversalAngle2 (jId), 0, 1e-4);

        dReal angle1, angle2;
        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (0, angle1, 1e-4);
        CHECK_CLOSE (0, angle2, 1e-4);

        dVector3 axis;
        dJointGetUniversalAxis1 (jId, axis);

        dReal ang1 = d2r(90);
        dReal ang2 = REAL(0.0);

        dMatrix3 R;
        dRFromAxisAndAngle (R, axis[0], axis[1], axis[2], ang1);
        dBodySetRotation (bId1, R);

        CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (ang2, dJointGetUniversalAngle2 (jId), 1e-4);


        dJointSetUniversalAxis1Offset (jId, axis[0], axis[1], axis[2],
                                       ang1, ang2);
        CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (ang2, dJointGetUniversalAngle2 (jId), 1e-4);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (ang1, angle1, 1e-4);
        CHECK_CLOSE (ang2, angle2, 1e-4);

        dRSetIdentity(R); // Set the rotation of the body to be zero
        dBodySetRotation (bId1, R);

        CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (0, angle1, 1e-4);
        CHECK_CLOSE (0, angle2, 1e-4);
    }



    // No offset when setting the Axis 2 offset
    // x is a Symbol for lines pointing into the screen
    // . is a Symbol for lines pointing out of the screen
    //
    //    In 2D                   In 3D
    //    ^  ^       ^   ^        Z ^   ^ Y             ^ Axis2
    //    |  |  =>   |   |          |  /               /
    //    |  |       |   |          | /               /
    //   B1  B2     B1   B2         . ------->    <-- Axis1
    //
    //  Start with a Delta of 90deg
    //    ^  ^        ^   ^
    //    |  |    =>  |   |
    //    |  |        |   |
    //   B1  B2      B1   B2
    TEST_FIXTURE (Fixture_dxJointUniversal_B1_and_B2_At_Zero_Axis1_Inverse_of_X,
                  test_dJointSetUniversalAxis2Offset_No_Offset_Axis2_Inverse_of_X)
    {
        CHECK_CLOSE (dJointGetUniversalAngle1 (jId), 0, 1e-4);
        CHECK_CLOSE (dJointGetUniversalAngle2 (jId), 0, 1e-4);

        dReal angle1, angle2;
        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (0, angle1, 1e-4);
        CHECK_CLOSE (0, angle2, 1e-4);

        dVector3 axis;
        dJointGetUniversalAxis2 (jId, axis);

        dReal ang1 = d2r(REAL(0.0));
        dReal ang2 = d2r(REAL(0.0));

        dMatrix3 R;
        dRFromAxisAndAngle (R, axis[0], axis[1], axis[2], ang2);
        dBodySetRotation (bId2, R);

        CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);


        dJointSetUniversalAxis2Offset (jId, axis[0], axis[1], axis[2],
                                       ang1, -ang2);
        CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (ang1, angle1, 1e-4);
        CHECK_CLOSE (-ang2, angle2, 1e-4);

        dRSetIdentity(R); // Set the rotation of the body to be zero
        dBodySetRotation (bId2, R);

        CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (0, angle1, 1e-4);
        CHECK_CLOSE (0, angle2, 1e-4);
    }

    //  Rotate 2nd body 90deg around axis2 then back to original position
    //
    //    In 2D                   In 3D
    //    ^  ^       ^            Z ^   ^ Y             ^ Axis2
    //    |  |  =>   |   -->        |  /               /
    //    |  |       |              | /               /
    //   B1  B2     B1   B2         . ------->    <-- Axis1
    //
    //  Start with a Delta of 90deg
    //    ^           ^   ^
    //    | <---  =>  |   |
    //    |           |   |
    //   B1  B2      B1   B2
    TEST_FIXTURE (Fixture_dxJointUniversal_B1_and_B2_At_Zero_Axis1_Inverse_of_X,
                  test_dJointSetUniversalAxisOffset_B2_90Deg)
    {
        CHECK_CLOSE (dJointGetUniversalAngle1 (jId), 0, 1e-4);
        CHECK_CLOSE (dJointGetUniversalAngle2 (jId), 0, 1e-4);

        dReal angle1, angle2;
        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (0, angle1, 1e-4);
        CHECK_CLOSE (0, angle2, 1e-4);

        dVector3 axis;
        dJointGetUniversalAxis2 (jId, axis);

        dReal ang1 = d2r(REAL(0.0));
        dReal ang2 = d2r(REAL(90.0));

        dMatrix3 R;
        dRFromAxisAndAngle (R, axis[0], axis[1], axis[2], ang2);
        dBodySetRotation (bId2, R);

        CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);


        dJointSetUniversalAxis2Offset (jId, axis[0], axis[1], axis[2],
                                       ang1, -ang2);
        CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (ang1, angle1, 1e-4);
        CHECK_CLOSE (-ang2, angle2, 1e-4);

        dRSetIdentity(R); // Set the rotation of the body to be zero
        dBodySetRotation (bId2, R);

        CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (0, angle1, 1e-4);
        CHECK_CLOSE (0, angle2, 1e-4);
    }


    //  Rotate 2nd body -90deg around axis2 then back to original position
    //
    //   ^  ^       ^
    //   |  |  =>   |  --->
    //   |  |       |
    //  B1  B2     B1   B2
    //
    // Start with a Delta of 90deg
    //   ^           ^   ^
    //   | --->  =>  |   |
    //   |           |   |
    //  B1  B2      B1   B2
    TEST_FIXTURE (Fixture_dxJointUniversal_B1_and_B2_At_Zero_Axis1_Inverse_of_X,
                  test_dJointSetUniversalAxis1Offset_B2_Minus90Deg)
    {
        CHECK_CLOSE (dJointGetUniversalAngle1 (jId), 0, 1e-4);
        CHECK_CLOSE (dJointGetUniversalAngle2 (jId), 0, 1e-4);

        dReal angle1, angle2;
        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (0, angle1, 1e-4);
        CHECK_CLOSE (0, angle2, 1e-4);

        dVector3 axis;
        dJointGetUniversalAxis2 (jId, axis);

        dReal ang1 = d2r(0.0);
        dReal ang2 = d2r(REAL(-90.0));


        dMatrix3 R;
        dRFromAxisAndAngle (R, axis[0], axis[1], axis[2], ang2);
        dBodySetRotation (bId2, R);

        CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (ang1, angle1, 1e-4);
        CHECK_CLOSE (-ang2, angle2, 1e-4);


        dJointGetUniversalAxis1 (jId, axis);
        dJointSetUniversalAxis1Offset (jId, axis[0], axis[1], axis[2],
                                       ang1, -ang2);
        CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (ang1, angle1, 1e-4);
        CHECK_CLOSE (-ang2, angle2, 1e-4);


        dRSetIdentity(R); // Set the rotation of the body to be zero
        dBodySetRotation (bId2, R);


        CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (0, angle1, 1e-4);
        CHECK_CLOSE (0, angle2, 1e-4);
    }


    // Rotate 1st body 0.23rad around X then back to original position
    //
    //   ^  ^     ^      ^
    //   |  |  =>  \     |
    //   |  |       \    |
    //  B1  B2     B1   B2
    //
    // Start with a Delta of 0.23rad
    // ^    ^        ^   ^
    //  \   | =>     |   |
    //   \  |        |   |
    //  B1  B2      B1   B2
    TEST_FIXTURE (Fixture_dxJointUniversal_B1_and_B2_At_Zero_Axis1_Inverse_of_X,
                  test_dJointSetUniversalAxis1Offset_B1_0_23rad)
    {
        CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

        dReal angle1, angle2;
        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (0, angle1, 1e-4);
        CHECK_CLOSE (0, angle2, 1e-4);

        dVector3 axis;
        dJointGetUniversalAxis1 (jId, axis);

        dReal ang1 = REAL(0.23);
        dReal ang2 = REAL(0.0);

        dMatrix3 R;
        dRFromAxisAndAngle (R, axis[0], axis[1], axis[2], ang1);
        dBodySetRotation (bId1, R);

        CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);


        dJointSetUniversalAxis1Offset (jId, axis[0], axis[1], axis[2],  ang1, ang2);
        CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (ang1, angle1, 1e-4);
        CHECK_CLOSE (-ang2, angle2, 1e-4);


        dRSetIdentity(R); // Set the rotation of the body to be zero
        dBodySetRotation (bId1, R);

        CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (0, angle1, 1e-4);
        CHECK_CLOSE (0, angle2, 1e-4);
    }


    // Rotate 2nd body -0.23rad around Z then back to original position
    //
    //   ^  ^         ^  ^
    //   |  |  =>    /   |
    //   |  |       /    |
    //  B1  B2     B1   B2
    //
    // Start with a Delta of 0.23rad
    //     ^ ^        ^   ^
    //    /  | =>     |   |
    //   /   |        |   |
    //  B1  B2      B1   B2
    TEST_FIXTURE (Fixture_dxJointUniversal_B1_and_B2_At_Zero_Axis1_Inverse_of_X,
                  test_dJointSetUniversalAxisOffset_B1_Minus0_23rad)
    {
        CHECK_CLOSE (dJointGetUniversalAngle1 (jId), 0, 1e-4);

        dMatrix3 R;
        dRFromAxisAndAngle (R, 1, 0, 0, -REAL(0.23));
        dBodySetRotation (bId1, R);

        CHECK_CLOSE (REAL(0.23), dJointGetUniversalAngle1 (jId), 1e-4);

        dVector3 axis;
        dJointGetUniversalAxis1 (jId, axis);
        dJointSetUniversalAxis1Offset (jId, axis[0], axis[1], axis[2],  REAL(0.23), 0);
        CHECK_CLOSE (REAL(0.23), dJointGetUniversalAngle1 (jId), 1e-4);

        dRFromAxisAndAngle (R, 1, 0, 0, 0);
        dBodySetRotation (bId1, R);

        CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
    }




    // Rotate the body by 90deg around X then back to original position.
    // The body is attached at the second position of the joint:
    // dJointAttache(jId, 0, bId);
    //
    //   ^
    //   |  => <---
    //   |
    //  B1      B1
    //
    // Start with a Delta of 90deg
    //            ^
    //  <---  =>  |
    //            |
    //   B1      B1
    TEST_FIXTURE (Fixture_dxJointUniversal_B1_At_Zero_Default_Axes,
                  test_dJointSetUniversalAxisOffset_1Body_B1_90Deg)
    {
        CHECK_CLOSE (dJointGetUniversalAngle1 (jId), 0, 1e-4);

        dMatrix3 R;
        dRFromAxisAndAngle (R, 1, 0, 0, M_PI/2.0);
        dBodySetRotation (bId1, R);

        CHECK_CLOSE (M_PI/2.0, dJointGetUniversalAngle1 (jId), 1e-4);

        dVector3 axis;
        dJointGetUniversalAxis1 (jId, axis);
        dJointSetUniversalAxis1Offset (jId, axis[0], axis[1], axis[2],  M_PI/2.0, 0);
        CHECK_CLOSE (M_PI/2.0, dJointGetUniversalAngle1 (jId), 1e-4);

        dRFromAxisAndAngle (R, 1, 0, 0, 0);
        dBodySetRotation (bId1, R);

        CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
    }

    // Rotate the body by -0.23rad around X then back to original position.
    // The body is attached at the second position of the joint:
    // dJointAttache(jId, 0, bId);
    //
    //   ^         ^
    //   |  =>    /
    //   |       /
    //  B1      B1
    //
    // Start with a Delta of -0.23rad
    //     ^     ^
    //    /  =>  |
    //   /       |
    //   B1     B1
    TEST_FIXTURE (Fixture_dxJointUniversal_B1_At_Zero_Default_Axes,
                  test_dJointSetUniversalAxisOffset_1Body_B1_Minus0_23rad)
    {
        CHECK_CLOSE (dJointGetUniversalAngle1 (jId), 0, 1e-4);

        dMatrix3 R;
        dRFromAxisAndAngle (R, 1, 0, 0, -REAL(0.23));
        dBodySetRotation (bId1, R);

        CHECK_CLOSE (-REAL(0.23), dJointGetUniversalAngle1 (jId), 1e-4);

        dVector3 axis;
        dJointGetUniversalAxis1 (jId, axis);
        dJointSetUniversalAxis1Offset (jId, axis[0], axis[1], axis[2],  -REAL(0.23), 0);
        CHECK_CLOSE (-REAL(0.23), dJointGetUniversalAngle1 (jId), 1e-4);

        dRFromAxisAndAngle (R, 1, 0, 0, 0);
        dBodySetRotation (bId1, R);

        CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
    }



    // Only one body body1 at (0,0,0)
    // The joint is an Universal Joint.
    // Axis the inverse of the X axis
    // Anchor at (0, 0, 0)
    //
    //       ^Y
    //       |
    //       |
    //       |
    //       |
    //       |
    // Z <-- X
    struct Fixture_dxJointUniversal_B1_At_Zero_Axis_Inverse_of_X
    {
        Fixture_dxJointUniversal_B1_At_Zero_Axis_Inverse_of_X()
        {
            wId = dWorldCreate();

            bId1 = dBodyCreate (wId);
            dBodySetPosition (bId1, 0, 0, 0);

            jId   = dJointCreateUniversal (wId, 0);
            joint = (dxJointUniversal*) jId;


            dJointAttach (jId, bId1, NULL);
            dJointSetUniversalAnchor (jId, 0, 0, 0);

            axis[0] = -1;
            axis[1] = 0;
            axis[2] = 0;
            dJointSetUniversalAxis1(jId, axis[0], axis[1], axis[2]);
        }

        ~Fixture_dxJointUniversal_B1_At_Zero_Axis_Inverse_of_X()
        {
            dWorldDestroy (wId);
        }

        dWorldID wId;

        dBodyID bId1;


        dJointID jId;
        dxJointUniversal* joint;

        dVector3 axis;
    };

    // Rotate B1 by 90deg around X then back to original position
    //
    //   ^
    //   |  => <---
    //   |
    //  B1      B1
    //
    // Start with a Delta of 90deg
    //            ^
    //  <---  =>  |
    //            |
    //   B1      B1
    TEST_FIXTURE (Fixture_dxJointUniversal_B1_At_Zero_Axis_Inverse_of_X,
                  test_dJointSetUniversalAxisOffset_1Body_B1_90Deg)
    {
        CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);

        dVector3 axis;
        dJointGetUniversalAxis1(jId, axis);

        dReal ang1 = d2r(REAL(90.0));

        dMatrix3 R;
        dRFromAxisAndAngle (R, axis[0], axis[1], axis[2], ang1);
        dBodySetRotation (bId1, R);

        CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);

        dJointSetUniversalAxis1Offset (jId, axis[0], axis[1], axis[2], ang1, 0);
        CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);

        dRSetIdentity(R); // Set the rotation of the body to be zero
        dBodySetRotation (bId1, R);

        CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
    }

    // Rotate B1 by -0.23rad around X then back to original position
    //
    //   ^         ^
    //   |  =>    /
    //   |       /
    //  B1      B1
    //
    // Start with a Delta of -0.23rad
    //     ^     ^
    //    /  =>  |
    //   /       |
    //   B1     B1
    TEST_FIXTURE (Fixture_dxJointUniversal_B1_At_Zero_Axis_Inverse_of_X,
                  test_dJointSetUniversalAxisOffset_1Body_B1_Minus0_23rad)
    {
        CHECK_CLOSE (dJointGetUniversalAngle1 (jId), 0, 1e-4);

        dMatrix3 R;
        dRFromAxisAndAngle (R, 1, 0, 0, -REAL(0.23));
        dBodySetRotation (bId1, R);

        CHECK_CLOSE (REAL(0.23), dJointGetUniversalAngle1 (jId), 1e-4);

        dVector3 axis;
        dJointGetUniversalAxis1 (jId, axis);
        dJointSetUniversalAxis1Offset (jId, axis[0], axis[1], axis[2],  REAL(0.23), 0);
        CHECK_CLOSE (REAL(0.23), dJointGetUniversalAngle1 (jId), 1e-4);

        dRFromAxisAndAngle (R, 1, 0, 0, 0);
        dBodySetRotation (bId1, R);

        CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
    }







    // Rotate B2 by 90deg around X then back to original position
    //
    //   ^
    //   |  => <---
    //   |
    //  B2      B2
    //
    // Start with a Delta of 90deg
    //            ^
    //  <---  =>  |
    //            |
    //   B2      B2
    TEST_FIXTURE (Fixture_dxJointUniversal_B2_At_Zero_Default_Axes,
                  test_dJointSetUniversalAxisOffset_1Body_B2_90Deg)
    {
        CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);


        dVector3 axis;
        dJointGetUniversalAxis2 (jId, axis);

        dReal ang2 = d2r(REAL(90.0));

        dMatrix3 R;
        dRFromAxisAndAngle (R, axis[0], axis[1], axis[2], ang2);
        dBodySetRotation (bId2, R);

        CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);

        dJointSetUniversalAxis2Offset (jId, axis[0], axis[1], axis[2], 0, -ang2);
        CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);


        dRSetIdentity(R); // Set the rotation of the body to be zero
        dBodySetRotation (bId2, R);

        CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);
    }

    // Rotate B2 by -0.23rad around Y then back to original position
    //
    //   ^         ^
    //   |  =>    /
    //   |       /
    //  B2      B2
    //
    // Start with an offset of -0.23rad
    //     ^     ^
    //    /  =>  |
    //   /       |
    //   B2     B2
    TEST_FIXTURE (Fixture_dxJointUniversal_B2_At_Zero_Default_Axes,
                  test_dJointSetUniversalAxis2Offset_1Body_B2_Minus0_23rad)
    {
        CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

        dReal angle1, angle2;
        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (0, angle1, 1e-4);
        CHECK_CLOSE (0, angle2, 1e-4);

        dVector3 axis;
        dJointGetUniversalAxis2 (jId, axis);

        dReal ang1 = 0;
        dReal ang2 = REAL(-0.23);


        dMatrix3 R;
        dRFromAxisAndAngle (R, axis[0], axis[1], axis[2], ang2);
        dBodySetRotation (bId2, R);


        CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (ang1, angle1, 1e-4);
        CHECK_CLOSE (-ang2, angle2, 1e-4);


        dJointSetUniversalAxis2Offset (jId, axis[0], axis[1], axis[2],
                                       ang1, -ang2);
        CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);

        dRSetIdentity(R); // Set the rotation of the body to be zero
        dBodySetRotation (bId2, R);

        CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (0, angle1, 1e-4);
        CHECK_CLOSE (0, angle2, 1e-4);
    }












    // The 2 bodies are positionned at (0,0,0),  and (0,0,0)
    // The bodis have no rotation.
    // The joint is a Universal Joint
    // The axis of the joint are at random (Still at 90deg w.r.t each other)
    // Anchor at (0, 0, 0)
    struct Fixture_dxJointUniversal_B1_and_B2_Axis_Random
    {
        Fixture_dxJointUniversal_B1_and_B2_Axis_Random()
        {
            wId = dWorldCreate();

            bId1 = dBodyCreate (wId);
            dBodySetPosition (bId1, -1, -2, -3);

            bId2 = dBodyCreate (wId);
            dBodySetPosition (bId2, 11, 22, 33);


            jId   = dJointCreateUniversal (wId, 0);


            dJointAttach (jId, bId1, bId2);

            dVector3 axis1;
            axis1[0] =  REAL(0.53);
            axis1[1] = -REAL(0.71);
            axis1[2] =  REAL(0.43);
            dNormalize3(axis1);

            dVector3 axis;
            axis[0] =  REAL(1.2);
            axis[1] =  REAL(0.87);
            axis[2] = -REAL(0.33);

            dVector3 axis2;
            dCalcVectorCross3(axis2, axis1, axis);

            dJointSetUniversalAxis1(jId, axis1[0], axis1[1], axis1[2]);
            dJointSetUniversalAxis2(jId, axis2[0], axis2[1], axis2[2]);
        }

        ~Fixture_dxJointUniversal_B1_and_B2_Axis_Random()
        {
            dWorldDestroy (wId);
        }


        dWorldID wId;

        dBodyID bId1;
        dBodyID bId2;


        dJointID jId;
    };


    // Rotate first body 90deg around Axis1 then back to original position
    //
    //   ^  ^           ^       Z ^
    //   |  |  => <---  |         |
    //   |  |           |         |
    //  B1  B2     B1   B2      X .----->Y
    //                          N.B. X is going out of the screen
    // Set Axis1 with an Offset of 90deg
    //      ^        ^   ^
    // <--- |  =>    |   |
    //      |        |   |
    //  B1  B2      B1   B2
    TEST_FIXTURE (Fixture_dxJointUniversal_B1_and_B2_Axis_Random,
                  test_dJointSetUniversalAxisOffset_B1_90deg_Axis_Random)
    {
        CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);
        dReal angle1, angle2;
        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (0, angle1, 1e-4);
        CHECK_CLOSE (0, angle2, 1e-4);


        dVector3 axis;
        dJointGetUniversalAxis1 (jId, axis);

        dReal angle = d2r(90);
        dMatrix3 R;
        dRFromAxisAndAngle (R, axis[0], axis[1], axis[2], angle);
        dBodySetRotation (bId1, R);


        CHECK_CLOSE (angle, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (angle, angle1, 1e-4);
        CHECK_CLOSE (0, angle2, 1e-4);



        dJointSetUniversalAxis1Offset (jId, axis[0], axis[1], axis[2], angle, 0);
        CHECK_CLOSE (angle, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (angle, angle1, 1e-4);
        CHECK_CLOSE (0, angle2, 1e-4);


        dRFromAxisAndAngle (R, axis[0], axis[1], axis[2], 0);
        dBodySetRotation (bId1, R);

        CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
        CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

        dJointGetUniversalAngles(jId, &angle1, &angle2);
        CHECK_CLOSE (0, angle1, 1e-4);
        CHECK_CLOSE (0, angle2, 1e-4);
    }

} // End of SUITE TestdxJointUniversal

