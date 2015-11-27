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

#include "config.h"
#include "../../ode/src/joints/amotor.h"

const dReal tol = 1e-5;

SUITE (TestdxJointAMotor)
{
    struct FixtureBase {
        dWorldID world;
        dBodyID body;
        dJointID joint;

        FixtureBase()
        {
            world = dWorldCreate();
            body = dBodyCreate(world);
            joint = dJointCreateAMotor(world, 0);
        }

        ~FixtureBase()
        {
            dJointDestroy(joint);
            dBodyDestroy(body);
            dWorldDestroy(world);
        }
    };


    struct FixtureXUser: FixtureBase {
        FixtureXUser()
        {
            // body only allowed to rotate around X axis
            dBodySetFiniteRotationMode(body, 1);
            dBodySetFiniteRotationAxis(body, 1, 0, 0);
            dJointAttach(joint, body, 0);
            dJointSetAMotorNumAxes(joint, 2);
            dJointSetAMotorAxis(joint, 0, 2, 0, 1, 0);
            dJointSetAMotorAxis(joint, 1, 2, 0, 0, 1);
            dJointSetAMotorParam(joint, dParamVel, 0);
            dJointSetAMotorParam(joint, dParamFMax, dInfinity);
            dJointSetAMotorParam(joint, dParamVel2, 0);
            dJointSetAMotorParam(joint, dParamFMax2, dInfinity);
        }
    };

    TEST_FIXTURE(FixtureXUser, rotate_x)
    {
        const dReal h = 1;
        const dReal v = 1;
        dMatrix3 identity = {1, 0, 0, 0,
                             0, 1, 0, 0,
                             0, 0, 1, 0};
        dBodySetRotation(body, identity);
        dBodySetAngularVel(body, v, 0, 0);
        dWorldQuickStep(world, h);
        const dReal* rot = dBodyGetRotation(body);
        CHECK_CLOSE(1, rot[0], tol);
        CHECK_CLOSE(0, rot[4], tol);
        CHECK_CLOSE(0, rot[8], tol);

        CHECK_CLOSE(0, rot[1], tol);
        CHECK_CLOSE(dCos(v*h), rot[5], tol);
        CHECK_CLOSE(dSin(v*h), rot[9], tol);

        CHECK_CLOSE(0, rot[2], tol);
        CHECK_CLOSE(-dSin(v*h), rot[6], tol);
        CHECK_CLOSE( dCos(v*h), rot[10], tol);
    }

    TEST_FIXTURE(FixtureXUser, rotate_yz)
    {
        const dReal h = 1;
        const dReal v = 1;
        dMatrix3 identity = {1, 0, 0, 0,
                             0, 1, 0, 0,
                             0, 0, 1, 0};
        dBodySetRotation(body, identity);

        dVector3 axis_y;
        dJointGetAMotorAxis(joint, 0, axis_y);
        CHECK_CLOSE(0, axis_y[0], tol);
        CHECK_CLOSE(1, axis_y[1], tol);
        CHECK_CLOSE(0, axis_y[2], tol);

        dVector3 axis_z;
        dJointGetAMotorAxis(joint, 1, axis_z);
        CHECK_CLOSE(0, axis_z[0], tol);
        CHECK_CLOSE(0, axis_z[1], tol);
        CHECK_CLOSE(1, axis_z[2], tol);

        dBodySetAngularVel(body, 0, v, v);
        dWorldStep(world, h);
        const dReal* rot = dBodyGetRotation(body);
        CHECK_CLOSE(1, rot[0], tol);
        CHECK_CLOSE(0, rot[4], tol);
        CHECK_CLOSE(0, rot[8], tol);

        CHECK_CLOSE(0, rot[1], tol);
        CHECK_CLOSE(1, rot[5], tol);
        CHECK_CLOSE(0, rot[9], tol);

        CHECK_CLOSE(0, rot[2], tol);
        CHECK_CLOSE(0, rot[6], tol);
        CHECK_CLOSE(1, rot[10], tol);
    }


    TEST_FIXTURE(FixtureBase, sanity_check)
    {
        dMatrix3 R;
        dRFromAxisAndAngle(R, 1, 1, 1, 10*M_PI/180);
        dBodySetRotation(body, R);

        dVector3 res;

        dJointAttach(joint, body, 0);
        dJointSetAMotorNumAxes(joint, 3);
        CHECK_EQUAL(3, dJointGetAMotorNumAxes(joint));

        // axes relative to world
        dJointSetAMotorAxis(joint, 0, 0, 1, 0, 0);
        dJointGetAMotorAxis(joint, 0, res);
        CHECK_EQUAL(0, dJointGetAMotorAxisRel(joint, 0));
        CHECK_CLOSE(1, res[0], tol);
        CHECK_CLOSE(0, res[1], tol);
        CHECK_CLOSE(0, res[2], tol);

        dJointSetAMotorAxis(joint, 1, 0, 0, 1, 0);
        dJointGetAMotorAxis(joint, 1, res);
        CHECK_EQUAL(0, dJointGetAMotorAxisRel(joint, 1));
        CHECK_CLOSE(0, res[0], tol);
        CHECK_CLOSE(1, res[1], tol);
        CHECK_CLOSE(0, res[2], tol);

        dJointSetAMotorAxis(joint, 2, 0, 0, 0, 1);
        dJointGetAMotorAxis(joint, 2, res);
        CHECK_EQUAL(0, dJointGetAMotorAxisRel(joint, 2));
        CHECK_CLOSE(0, res[0], tol);
        CHECK_CLOSE(0, res[1], tol);
        CHECK_CLOSE(1, res[2], tol);

        // axes relative to body1
        dJointSetAMotorAxis(joint, 0, 1, 1, 0, 0);
        dJointGetAMotorAxis(joint, 0, res);
        CHECK_EQUAL(1, dJointGetAMotorAxisRel(joint, 0));
        CHECK_CLOSE(1, res[0], tol);
        CHECK_CLOSE(0, res[1], tol);
        CHECK_CLOSE(0, res[2], tol);

        dJointSetAMotorAxis(joint, 1, 1, 0, 1, 0);
        dJointGetAMotorAxis(joint, 1, res);
        CHECK_EQUAL(1, dJointGetAMotorAxisRel(joint, 1));
        CHECK_CLOSE(0, res[0], tol);
        CHECK_CLOSE(1, res[1], tol);
        CHECK_CLOSE(0, res[2], tol);

        dJointSetAMotorAxis(joint, 2, 1, 0, 0, 1);
        dJointGetAMotorAxis(joint, 2, res);
        CHECK_EQUAL(1, dJointGetAMotorAxisRel(joint, 2));
        CHECK_CLOSE(0, res[0], tol);
        CHECK_CLOSE(0, res[1], tol);
        CHECK_CLOSE(1, res[2], tol);

        // axes relative to body2
        dJointSetAMotorAxis(joint, 0, 2, 1, 0, 0);
        dJointGetAMotorAxis(joint, 0, res);
        CHECK_EQUAL(2, dJointGetAMotorAxisRel(joint, 0));
        CHECK_CLOSE(1, res[0], tol);
        CHECK_CLOSE(0, res[1], tol);
        CHECK_CLOSE(0, res[2], tol);

        dJointSetAMotorAxis(joint, 1, 2, 0, 1, 0);
        dJointGetAMotorAxis(joint, 1, res);
        CHECK_EQUAL(2, dJointGetAMotorAxisRel(joint, 1));
        CHECK_CLOSE(0, res[0], tol);
        CHECK_CLOSE(1, res[1], tol);
        CHECK_CLOSE(0, res[2], tol);

        dJointSetAMotorAxis(joint, 2, 2, 0, 0, 1);
        dJointGetAMotorAxis(joint, 2, res);
        CHECK_EQUAL(2, dJointGetAMotorAxisRel(joint, 2));
        CHECK_CLOSE(0, res[0], tol);
        CHECK_CLOSE(0, res[1], tol);
        CHECK_CLOSE(1, res[2], tol);

        // reverse attachment to force internal reversal
        dJointAttach(joint, 0, body);
        // axes relative to world
        dJointSetAMotorAxis(joint, 0, 0, 1, 0, 0);
        dJointGetAMotorAxis(joint, 0, res);
        CHECK_EQUAL(0, dJointGetAMotorAxisRel(joint, 0));
        CHECK_CLOSE(1, res[0], tol);
        CHECK_CLOSE(0, res[1], tol);
        CHECK_CLOSE(0, res[2], tol);

        dJointSetAMotorAxis(joint, 1, 0, 0, 1, 0);
        dJointGetAMotorAxis(joint, 1, res);
        CHECK_EQUAL(0, dJointGetAMotorAxisRel(joint, 1));
        CHECK_CLOSE(0, res[0], tol);
        CHECK_CLOSE(1, res[1], tol);
        CHECK_CLOSE(0, res[2], tol);

        dJointSetAMotorAxis(joint, 2, 0, 0, 0, 1);
        dJointGetAMotorAxis(joint, 2, res);
        CHECK_EQUAL(0, dJointGetAMotorAxisRel(joint, 2));
        CHECK_CLOSE(0, res[0], tol);
        CHECK_CLOSE(0, res[1], tol);
        CHECK_CLOSE(1, res[2], tol);

        // axes relative to body1
        dJointSetAMotorAxis(joint, 0, 1, 1, 0, 0);
        dJointGetAMotorAxis(joint, 0, res);
        CHECK_EQUAL(1, dJointGetAMotorAxisRel(joint, 0));
        CHECK_CLOSE(1, res[0], tol);
        CHECK_CLOSE(0, res[1], tol);
        CHECK_CLOSE(0, res[2], tol);

        dJointSetAMotorAxis(joint, 1, 1, 0, 1, 0);
        dJointGetAMotorAxis(joint, 1, res);
        CHECK_EQUAL(1, dJointGetAMotorAxisRel(joint, 1));
        CHECK_CLOSE(0, res[0], tol);
        CHECK_CLOSE(1, res[1], tol);
        CHECK_CLOSE(0, res[2], tol);

        dJointSetAMotorAxis(joint, 2, 1, 0, 0, 1);
        dJointGetAMotorAxis(joint, 2, res);
        CHECK_EQUAL(1, dJointGetAMotorAxisRel(joint, 2));
        CHECK_CLOSE(0, res[0], tol);
        CHECK_CLOSE(0, res[1], tol);
        CHECK_CLOSE(1, res[2], tol);

        // axes relative to body2
        dJointSetAMotorAxis(joint, 0, 2, 1, 0, 0);
        dJointGetAMotorAxis(joint, 0, res);
        CHECK_EQUAL(2, dJointGetAMotorAxisRel(joint, 0));
        CHECK_CLOSE(1, res[0], tol);
        CHECK_CLOSE(0, res[1], tol);
        CHECK_CLOSE(0, res[2], tol);

        dJointSetAMotorAxis(joint, 1, 2, 0, 1, 0);
        dJointGetAMotorAxis(joint, 1, res);
        CHECK_EQUAL(2, dJointGetAMotorAxisRel(joint, 1));
        CHECK_CLOSE(0, res[0], tol);
        CHECK_CLOSE(1, res[1], tol);
        CHECK_CLOSE(0, res[2], tol);

        dJointSetAMotorAxis(joint, 2, 2, 0, 0, 1);
        dJointGetAMotorAxis(joint, 2, res);
        CHECK_EQUAL(2, dJointGetAMotorAxisRel(joint, 2));
        CHECK_CLOSE(0, res[0], tol);
        CHECK_CLOSE(0, res[1], tol);
        CHECK_CLOSE(1, res[2], tol);
    }


    struct FixtureXEuler : FixtureBase {
        FixtureXEuler()
        {
            // body only allowed to rotate around X axis
            dJointAttach(joint, 0, body);
            dJointSetAMotorMode(joint, dAMotorEuler);
            dJointSetAMotorAxis(joint, 0, 0, 1, 0, 0);
            dJointSetAMotorAxis(joint, 2, 0, 0, 0, 1);
        }
    };


    TEST_FIXTURE(FixtureXEuler, check_axes)
    {
        // test patch #181 bug fix
        dVector3 axis_x;
        dJointGetAMotorAxis(joint, 0, axis_x);
        CHECK_CLOSE(1, axis_x[0], tol);
        CHECK_CLOSE(0, axis_x[1], tol);
        CHECK_CLOSE(0, axis_x[2], tol);

        dVector3 axis_y;
        dJointGetAMotorAxis(joint, 1, axis_y);
        CHECK_CLOSE(0, axis_y[0], tol);
        CHECK_CLOSE(1, axis_y[1], tol);
        CHECK_CLOSE(0, axis_y[2], tol);

        dVector3 axis_z;
        dJointGetAMotorAxis(joint, 2, axis_z);
        CHECK_CLOSE(0, axis_z[0], tol);
        CHECK_CLOSE(0, axis_z[1], tol);
        CHECK_CLOSE(1, axis_z[2], tol);
    }

} // End of SUITE TestdxJointAMotor
