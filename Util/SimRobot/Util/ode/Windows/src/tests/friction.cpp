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
// ode/src/joint.cpp
//
//
////////////////////////////////////////////////////////////////////////////////
#include <algorithm>
#include <UnitTest++.h>
#include <ode/ode.h>
#include "../ode/src/config.h"
#include "../ode/src/joints/joints.h"


/*
 * Tests for contact friction
 */

SUITE(JointContact)
{
    struct ContactSetup
    {
        dWorldID world;
        dBodyID body1;
        dBodyID body2;
        dJointID joint;

        ContactSetup()
        {
            world = dWorldCreate();
            body1 = dBodyCreate(world);
            body2 = dBodyCreate(world);

            dBodySetPosition(body1, -1, 0, 0);
            dBodySetPosition(body2,  1, 0, 0);
        }

        ~ContactSetup()
        {
            dBodyDestroy(body1);
            dBodyDestroy(body2);
            dWorldDestroy(world);
        }
    };
    
    TEST_FIXTURE(ContactSetup,
                 test_ZeroMu)
    {
        dxJoint::Info1 info1;
        dReal dummy_J[3][16] = {{0}};
        int dummy_findex[3];

        dReal info2_fps = 100;
        dReal info2_erp = 0;
        dReal *J1 = dummy_J[0];
        dReal *J2 = dummy_J[0] + 8;
        dReal *rhscfm = dummy_J[0] + 6;
        dReal *lohi = dummy_J[0] + 14;
        unsigned rowskip = 16;
        int *findex = dummy_findex;

#define ZERO_ALL do {                                           \
            memset(dummy_J, 0, sizeof dummy_J);                 \
            std::fill(dummy_findex, dummy_findex+3, -1);        \
        }                                                       \
        while (0)

        dContact contact;
        contact.surface.mode = dContactMu2 | dContactFDir1 | dContactApprox1;

        contact.geom.pos[0] = 0;
        contact.geom.pos[1] = 0;
        contact.geom.pos[2] = 0;

        // normal points into body1
        contact.geom.normal[0] = -1;
        contact.geom.normal[1] = 0;
        contact.geom.normal[2] = 0;

        contact.geom.depth = 0;

        contact.geom.g1 = 0;
        contact.geom.g2 = 0;

        // we ask for fdir1 = +Y, so fdir2 = normal x fdir1 = -Z
        contact.fdir1[0] = 0;
        contact.fdir1[1] = 1;
        contact.fdir1[2] = 0;

        /*
         * First, test with mu = 0, mu2 = 1
         * Because there is no friction on the first direction (+Y) the body
         * is allowed to translate in the Y axis and rotate around the Z axis.
         *
         * That is, the only constraint will be for the second dir (-Z):
         * so J[1] = [  0  0 -1    0  1  0    0  0  1    0  1  0 ]
         */        
        contact.surface.mu = 0;
        contact.surface.mu2 = 1;
        joint = dJointCreateContact(world, 0, &contact);
        dJointAttach(joint, body1, body2);
        joint->getInfo1(&info1);
        CHECK_EQUAL(2, (int)info1.m);
        ZERO_ALL;
        joint->getInfo2(info2_fps, info2_erp, rowskip, J1, J2,
            rowskip, rhscfm, lohi, findex);
        CHECK_CLOSE(0, dummy_J[1][0], 1e-6);
        CHECK_CLOSE(0, dummy_J[1][1], 1e-6);
        CHECK_CLOSE(-1, dummy_J[1][2], 1e-6);
        CHECK_CLOSE(0, dummy_J[1][3], 1e-6);
        CHECK_CLOSE(1, dummy_J[1][4], 1e-6);
        CHECK_CLOSE(0, dummy_J[1][5], 1e-6);
        CHECK_CLOSE(0, dummy_J[1][8], 1e-6);
        CHECK_CLOSE(0, dummy_J[1][9], 1e-6);
        CHECK_CLOSE(1, dummy_J[1][10], 1e-6);
        CHECK_CLOSE(0, dummy_J[1][11], 1e-6);
        CHECK_CLOSE(1, dummy_J[1][12], 1e-6);
        CHECK_CLOSE(0, dummy_J[1][13], 1e-6);
        CHECK_EQUAL(0, dummy_findex[1]); // because of dContactApprox1
        dJointDestroy(joint);


        /*
         * Now try with no frictino in the second direction. The Jacobian should look like:
         * J[1] = [  0  1  0    0  0  1    0 -1  0    0  0  1 ]
         */
        // try again, with zero mu2
        contact.surface.mu = 1;
        contact.surface.mu2 = 0;
        joint = dJointCreateContact(world, 0, &contact);
        dJointAttach(joint, body1, body2);
        joint->getInfo1(&info1);
        CHECK_EQUAL(2, (int)info1.m);
        ZERO_ALL;
        joint->getInfo2(info2_fps, info2_erp, rowskip, J1, J2,
            rowskip, rhscfm, lohi, findex);
        CHECK_CLOSE(0, dummy_J[1][0], 1e-6);
        CHECK_CLOSE(1, dummy_J[1][1], 1e-6);
        CHECK_CLOSE(0, dummy_J[1][2], 1e-6);
        CHECK_CLOSE(0, dummy_J[1][3], 1e-6);
        CHECK_CLOSE(0, dummy_J[1][4], 1e-6);
        CHECK_CLOSE(1, dummy_J[1][5], 1e-6);
        CHECK_CLOSE(0, dummy_J[1][8], 1e-6);
        CHECK_CLOSE(-1, dummy_J[1][9], 1e-6);
        CHECK_CLOSE(0, dummy_J[1][10], 1e-6);
        CHECK_CLOSE(0, dummy_J[1][11], 1e-6);
        CHECK_CLOSE(0, dummy_J[1][12], 1e-6);
        CHECK_CLOSE(1, dummy_J[1][13], 1e-6);
        CHECK_EQUAL(0, dummy_findex[1]);  // because of dContactApprox1
        dJointDestroy(joint);
    }

}
