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

#include <UnitTest++.h>
#include <ode/ode.h>
#include <ode/odemath.h>



TEST(test_dNormalization3)
{
    const dVector3 x = {1,0,0,0};
    const dVector3 y = {0,1,0,0};
    const dVector3 z = {0,0,1,0};
    dVector3 v;

    // Check when value in first component (i.e. [0])
    v[0] = REAL(1.0);
    v[1] = REAL(0.0);
    v[2] = REAL(0.0);
    dSafeNormalize3(v);
    CHECK_ARRAY_CLOSE(x, v, 3, 1e-6);
    CHECK_EQUAL(dCalcVectorLength3(v), REAL(1.0));

    v[0] = REAL(0.1);
    v[1] = REAL(0.0);
    v[2] = REAL(0.0);
    dSafeNormalize3(v);
    CHECK_ARRAY_CLOSE(x, v, 3, 1e-6);
    CHECK_EQUAL(dCalcVectorLength3(v), REAL(1.0));

    v[0] = REAL(1e-20);
    v[1] = REAL(0.0);
    v[2] = REAL(0.0);
    dSafeNormalize3(v);
    CHECK_ARRAY_CLOSE(x, v, 3, 1e-6);
    CHECK_EQUAL(dCalcVectorLength3(v), REAL(1.0));


    // Check when value in first component (i.e. [0])
    v[0] = REAL(0.0);
    v[1] = REAL(1.0);
    v[2] = REAL(0.0);
    dSafeNormalize3(v);
    CHECK_ARRAY_CLOSE(y, v, 3, 1e-6);
    CHECK_EQUAL(dCalcVectorLength3(v), REAL(1.0));

    v[0] = REAL(0.0);
    v[1] = REAL(0.1);
    v[2] = REAL(0.0);
    dSafeNormalize3(v);
    CHECK_ARRAY_CLOSE(y, v, 3, 1e-6);
    CHECK_EQUAL(dCalcVectorLength3(v), REAL(1.0));

    v[0] = REAL(0.0);
    v[1] = REAL(1e-20);
    v[2] = REAL(0.0);
    dSafeNormalize3(v);
    CHECK_ARRAY_CLOSE(y, v, 3, 1e-6);
    CHECK_EQUAL(dCalcVectorLength3(v), REAL(1.0));


    // Check when value in first component (i.e. [0])
    v[0] = REAL(0.0);
    v[1] = REAL(0.0);
    v[2] = REAL(1.0);
    dSafeNormalize3(v);
    CHECK_ARRAY_CLOSE(z, v, 3, 1e-6);
    CHECK_EQUAL(dCalcVectorLength3(v), REAL(1.0));

    v[0] = REAL(0.0);
    v[1] = REAL(0.0);
    v[2] = REAL(0.1);
    dSafeNormalize3(v);
    CHECK_ARRAY_CLOSE(z, v, 3, 1e-6);
    CHECK_EQUAL(dCalcVectorLength3(v), REAL(1.0));

    v[0] = REAL(0.0);
    v[1] = REAL(0.0);
    v[2] = REAL(1e-20);
    dSafeNormalize3(v);
    CHECK_ARRAY_CLOSE(z, v, 3, 1e-6);
    CHECK_EQUAL(dCalcVectorLength3(v), REAL(1.0));


    // Check negative
    // Check when value in first component (i.e. [0])
    v[0] = REAL(-1.0);
    v[1] = REAL(0.0);
    v[2] = REAL(0.0);
    dSafeNormalize3(v);
    CHECK_EQUAL(dCalcVectorLength3(v), REAL(1.0));

    v[0] = REAL(-0.1);
    v[1] = REAL(0.0);
    v[2] = REAL(0.0);
    dSafeNormalize3(v);
    CHECK_EQUAL(dCalcVectorLength3(v), REAL(1.0));

    v[0] = REAL(-1e-20);
    v[1] = REAL(0.0);
    v[2] = REAL(0.0);
    dSafeNormalize3(v);
    CHECK_EQUAL(dCalcVectorLength3(v), REAL(1.0));


    // Check when value in first component (i.e. [0])
    v[0] = REAL(0.0);
    v[1] = REAL(-1.0);
    v[2] = REAL(0.0);
    dSafeNormalize3(v);
    CHECK_EQUAL(dCalcVectorLength3(v), REAL(1.0));

    v[0] = REAL(0.0);
    v[1] = REAL(-0.1);
    v[2] = REAL(0.0);
    dSafeNormalize3(v);
    CHECK_EQUAL(dCalcVectorLength3(v), REAL(1.0));

    v[0] = REAL(0.0);
    v[1] = REAL(-1e-20);
    v[2] = REAL(0.0);
    dSafeNormalize3(v);
    CHECK_EQUAL(dCalcVectorLength3(v), REAL(1.0));


    // Check when value in first component (i.e. [0])
    v[0] = REAL(0.0);
    v[1] = REAL(0.0);
    v[2] = REAL(-1.0);
    dSafeNormalize3(v);
    CHECK_EQUAL(dCalcVectorLength3(v), REAL(1.0));

    v[0] = REAL(0.0);
    v[1] = REAL(0.0);
    v[2] = REAL(-0.1);
    dSafeNormalize3(v);
    CHECK_EQUAL(dCalcVectorLength3(v), REAL(1.0));

    v[0] = REAL(0.0);
    v[1] = REAL(0.0);
    v[2] = REAL(-1e-20);
    dSafeNormalize3(v);
    CHECK_EQUAL(dCalcVectorLength3(v), REAL(1.0));


    v[0] = REAL(9999999999.0);
    v[1] = REAL(0.0);
    v[2] = REAL(1e-20);
    dSafeNormalize3(v);
    CHECK_EQUAL(dCalcVectorLength3(v), REAL(1.0));


    v[0] = REAL(9999999999.0);
    v[1] = REAL(9999.0);
    v[2] = REAL(9.0);
    dSafeNormalize3(v);
    CHECK_EQUAL(dCalcVectorLength3(v), REAL(1.0));

}


TEST(test_dOrthogonalizeR)
{
    {
        dMatrix3 r1 = { 1, 0, 0, 0,
                        0, 1, 0, 0,
                        0, 0, 1, 0
                      };
        dMatrix3 r2;
        memcpy(r2, r1, sizeof(dMatrix3));
        dOrthogonalizeR(r2);
        CHECK_ARRAY_EQUAL(r1, r2, 12);
    }
    {
        dMatrix3 r1 = { 0, 1, 0, 0,
                        0, 0, 1, 0,
                        1, 0, 0, 0
                      };
        dMatrix3 r2;
        memcpy(r2, r1, sizeof(dMatrix3));
        dOrthogonalizeR(r2);
        CHECK_ARRAY_EQUAL(r1, r2, 12);
    }
    {
        dMatrix3 r1 = { 0, 0, 1, 0,
                        1, 0, 0, 0,
                        0, 1, 0, 0
                      };
        dMatrix3 r2;
        memcpy(r2, r1, sizeof(dMatrix3));
        dOrthogonalizeR(r2);
        CHECK_ARRAY_EQUAL(r1, r2, 12);
    }
    {
        dMatrix3 r1 = { -1, 0,  0, 0,
                        0, 1,  0, 0,
                        0, 0, -1, 0
                      };
        dMatrix3 r2;
        memcpy(r2, r1, sizeof(dMatrix3));
        dOrthogonalizeR(r2);
        CHECK_ARRAY_EQUAL(r1, r2, 12);
    }
    {
        dMatrix3 r1 = { 0, -1, 0, 0,
                        0,  0, 1, 0,
                        -1,  0, 0, 0
                      };
        dMatrix3 r2;
        memcpy(r2, r1, sizeof(dMatrix3));
        dOrthogonalizeR(r2);
        CHECK_ARRAY_EQUAL(r1, r2, 12);
    }
    {
        dMatrix3 r1 = { 0, 0, -1, 0,
                        0, -1, 0, 0,
                        -1, 0, 0, 0
                      };
        dMatrix3 r2;
        memcpy(r2, r1, sizeof(dMatrix3));
        dOrthogonalizeR(r2);
        CHECK_ARRAY_EQUAL(r1, r2, 12);
    }

}
