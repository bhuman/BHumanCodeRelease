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
// ode/src/joinst/dball.cpp
//
//
////////////////////////////////////////////////////////////////////////////////

#include <UnitTest++.h>
#include <ode/ode.h>

#include "../../ode/src/config.h"


using namespace std;

SUITE (TestdxJointDBall)
{
    struct SimpleFixture {
        dWorldID w;
        dBodyID b1, b2;
        dJointID j;

        SimpleFixture() :
            w(dWorldCreate()),
            b1(dBodyCreate(w)),
            b2(dBodyCreate(w)),
            j(dJointCreateDBall(w, 0))
        {
            dJointAttach(j, b1, b2);
        }

        ~SimpleFixture()
        {
            dJointDestroy(j);
            dBodyDestroy(b1);
            dBodyDestroy(b2);
            dWorldDestroy(w);
        }
    };

    TEST_FIXTURE(SimpleFixture, testTargetDistance)
    {
        dBodySetPosition(b1, -1, -2, -3);
        dBodySetPosition(b2, 3, 5, 7);
        dJointAttach(j, b1, b2); // this recomputes the deduced target distance
        CHECK_CLOSE(dJointGetDBallDistance(j), dSqrt(REAL(165.0)), 1e-4);

        // moving body should not change target distance
        dBodySetPosition(b1, 2,3,4);
        CHECK_CLOSE(dJointGetDBallDistance(j), dSqrt(REAL(165.0)), 1e-4);

        // setting target distance manually should override the deduced one
        dJointSetDBallDistance(j, REAL(6.0));
        CHECK_EQUAL(dJointGetDBallDistance(j), REAL(6.0));
    }

}
