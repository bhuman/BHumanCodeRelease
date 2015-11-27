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

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"

#ifdef dDOUBLE
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawLine dsDrawLineD
#define dsDrawSphere dsDrawSphereD
#endif

dReal theta = M_PI / 4;
dReal ratio = 1, speed = 5, rho_1 = 1, rho_2 = 1, backlash = 0.1;
int mode = 0;

dWorldID world;
dSpaceID space;
dBodyID body1, body2;
dGeomID geom1, geom2;
dJointID hinge1, hinge2, transmission;
dJointFeedback feedback;

void setup() {
    dMatrix3 R;
    
    switch (mode) {
    case 0:
	// Parallel axes.

	dBodySetPosition(body1, 1, 0, 1);
	dBodySetPosition(body2, -1, 0, 1);

	dRSetIdentity (R);
	dBodySetRotation (body1, R);
	dBodySetRotation (body2, R);

	dJointSetHingeAnchor(hinge2, -1, 0, 1);
	dJointSetHingeAxis(hinge2, 0, 0, 1);

	dJointSetHingeAnchor(hinge1, 1, 0, 1);
	dJointSetHingeAxis(hinge1, 0, 0, 1);

	dJointSetTransmissionMode(transmission, dTransmissionParallelAxes);
	dJointSetTransmissionRatio(transmission, ratio);
	dJointSetTransmissionAnchor1(transmission, 1, 0, 1);
	dJointSetTransmissionAnchor2(transmission, -1, 0, 1);
	dJointSetTransmissionAxis(transmission, 0, 0, 1);

	break;
    case 1:
	// Intersecting axes.

	dBodySetPosition(body1, 1, 0, 1);
	dBodySetPosition(body2, -1, 0, 2);

	dRSetIdentity (R);
	dBodySetRotation (body1, R);

	dRFromZAxis (R, cos(theta), 0, sin(theta));
	dBodySetRotation (body2, R);

	dJointSetHingeAnchor(hinge2, -1, 0, 2);
	dJointSetHingeAxis(hinge2, cos(theta), 0, sin(theta));

	dJointSetHingeAnchor(hinge1, 1, 0, 1);
	dJointSetHingeAxis(hinge1, 0, 0, 1);

	dJointSetTransmissionMode(transmission, dTransmissionIntersectingAxes);
	dJointSetTransmissionAnchor1(transmission, 1, 0, 1);
	dJointSetTransmissionAnchor2(transmission, -1, 0, 2);
	dJointSetTransmissionAxis1(transmission, 0, 0, -1);
	dJointSetTransmissionAxis2(transmission, cos(theta), 0, sin(theta));

	break;
    case 2:
	// Chain.
	
	dBodySetPosition(body1, 2, 0, 1);
	dBodySetPosition(body2, -2, 0, 1);

	dRSetIdentity (R);
	dBodySetRotation (body1, R);
	dBodySetRotation (body2, R);

	dJointSetHingeAnchor(hinge2, -2, 0, 1);
	dJointSetHingeAxis(hinge2, 0, 0, 1);

	dJointSetHingeAnchor(hinge1, 2, 0, 1);
	dJointSetHingeAxis(hinge1, 0, 0, 1);

	dJointSetTransmissionMode(transmission, dTransmissionChainDrive);
	dJointSetTransmissionAnchor1(transmission, 2, 0, 1);
	dJointSetTransmissionAnchor2(transmission, -2, 0, 1);
	dJointSetTransmissionRadius1(transmission, rho_1);
	dJointSetTransmissionRadius2(transmission, rho_2);
	dJointSetTransmissionAxis(transmission, 0, 0, 1);

	break;
    }

    dJointSetTransmissionBacklash(transmission, backlash);

    dJointSetHingeParam(hinge2, dParamVel, speed);
    dJointSetHingeParam(hinge2, dParamFMax, 50);

    dJointSetHingeParam(hinge1, dParamVel, 0);
    dJointSetHingeParam(hinge1, dParamFMax, 2);

    dBodySetLinearVel(body1, 0, 0, 0);
    dBodySetLinearVel(body2, 0, 0, 0);
    dBodySetAngularVel(body1, 0, 0, 0);
    dBodySetAngularVel(body2, 0, 0, 0);
}

void start()
{
    dMass mass;
    
    world = dWorldCreate();
    dWorldSetGravity (world,0,0,-9.8);

    dWorldSetERP(world, 0.2);

    space = dSimpleSpaceCreate (0);
    
    body1 = dBodyCreate(world);
    body2 = dBodyCreate(world);

    dBodySetFiniteRotationMode(body1, 1);
    dBodySetFiniteRotationMode(body2, 1);

    geom1 = dCreateCylinder(space, 0.2, 0.5);
    dGeomSetBody(geom1, body1);
    dMassSetCylinder(&mass, 100, 3, 0.2, 0.5);
    dBodySetMass(body1, &mass);
    
    geom2 = dCreateCylinder(space, 0.2, 0.5);
    dGeomSetBody(geom2, body2);
    dMassSetCylinder(&mass, 100, 3, 0.2, 0.5);
    dBodySetMass(body2, &mass);

    hinge1 = dJointCreateHinge(world, 0);
    dJointAttach(hinge1, body1, 0);
    
    hinge2 = dJointCreateHinge(world, 0);
    dJointAttach(hinge2, body2, 0);

    transmission = dJointCreateTransmission(world, 0);
    dJointAttach(transmission, body1, body2);
    dJointSetFeedback(transmission, &feedback);

    setup();

    // initial camera position
    static float xyz[3] = {1.15,-2.78,4.1};
    static float hpr[3] = {105,-45.5,0};
    dsSetViewpoint (xyz,hpr);

    fprintf (stderr,
	     "The green wheel is driving the red one. To control it use the following:\n"
	     "   '[' : decrease wheel ratio\n"
	     "   ']' : increase wheel ratio\n"
	     "   ',' : decrease driving wheel speed\n"
	     "   '.' : increase driving wheel speed\n"
	     "   '-' : decrease backlash\n"
	     "   '=' : increase backlash\n"
	     "   '1' : switch to parallel axes gears mode\n"
	     "   '2' : switch to intersecting axes gears mode\n"
	     "   '3' : switch to chain (or belt) mode\n"
);
}

void stop()
{
    dSpaceDestroy(space);

    dWorldDestroy(world);
}

void drawGeom(dGeomID g)
{
    int gclass = dGeomGetClass(g);
    const dReal *pos = dGeomGetPosition(g);
    const dReal *rot = dGeomGetRotation(g);

    switch (gclass) {
        case dCylinderClass:
        {
            dReal length, radius;

	    if (g == geom1) {
		dsSetColorAlpha(1, 0, 0, 1);
	    } else {
		dsSetColorAlpha(0, 1, 0, 1);
	    }

            dsSetTexture (DS_WOOD);
            dGeomCylinderGetParams(g, &radius, &length);
            dsDrawCylinder(pos, rot, length, radius);
            break;
        }
        
        default:
	{
	    abort();
	}
    }
}

void simLoop(int pause)
{
    if (!pause) {

        const dReal step = 0.003;
        const unsigned nsteps = 4;

        for (unsigned i=0; i<nsteps; ++i) {
            dWorldQuickStep(world, step);
        }
    }

#if 0
    {
	const dReal *omega_1, *omega_2;

	omega_1 = dBodyGetAngularVel(body1);
	omega_2 = dBodyGetAngularVel(body2);

	printf ("T1: %f, %f, %f\n",
		feedback.t1[0], feedback.t1[1], feedback.t1[2]);

	printf ("T2: %f, %f, %f\n",
		feedback.t2[0], feedback.t2[1], feedback.t2[2]);

	printf ("F1: %f, %f, %f\n",
		feedback.f1[0], feedback.f1[1], feedback.f1[2]);

	printf ("F2: %f, %f, %f\n",
		feedback.f2[0], feedback.f2[1], feedback.f2[2]);
    }
#endif

    // now we draw everything
    unsigned ngeoms = dSpaceGetNumGeoms(space);
    for (unsigned i=0; i<ngeoms; ++i) {
        dGeomID g = dSpaceGetGeom(space, i);

        drawGeom(g);
    }

    const dReal *R_1 = dGeomGetRotation(geom1);
    const dReal *R_2 = dGeomGetRotation(geom2);
    dVector3 c_1, c_2, a_1, a_2;

    dJointGetTransmissionContactPoint1(transmission, c_1);
    dJointGetTransmissionContactPoint2(transmission, c_2);
    dJointGetTransmissionAnchor1(transmission, a_1);
    dJointGetTransmissionAnchor2(transmission, a_2);

    dsSetColorAlpha(1, 0, 0, 0.5);
    dsDrawCylinder(a_1, R_1, 0.05, dCalcPointsDistance3(c_1, a_1));
    dsSetColorAlpha(0, 1, 0, 0.5);
    dsDrawCylinder(a_2, R_2, 0.05, dCalcPointsDistance3(c_2, a_2));

    dsSetColorAlpha(1, 0, 0, 0.5);
    dsDrawSphere (c_1, R_1, 0.05);
    dsDrawSphere (c_2, R_1, 0.05);

    dsSetColorAlpha(1, 1, 0, 0.5);
    if (mode == dTransmissionChainDrive) {
        dsDrawLine(c_1, c_2);
    }
}

static void command (int cmd)
{
    if (cmd == '[') {
	switch(mode) {
	case dTransmissionParallelAxes:
	    if (ratio > 0.125) {
		ratio *= 0.5;

		fprintf (stderr, "Gear ratio set to %.3f.\n", ratio);
	    }
	    break;
	case dTransmissionIntersectingAxes:
	    if (theta > 0.1) {
		theta -= 0.1;

		fprintf (stderr, "Gear angle set to %.3f deg.\n",
			 theta / M_PI * 180);
	    }
	    break;
	case dTransmissionChainDrive:
	    if (rho_2 > 0.125) {
		rho_2 /= 2;

		fprintf (stderr, "Sprocket ratio set to %.3f.\n", rho_2 / rho_1);
	    }
	    break;
	}

	setup();
    } else if (cmd == ']') {
	switch(mode) {
	case dTransmissionParallelAxes:
	    if (ratio < 8) {
		ratio *= 2;

		fprintf (stderr, "Gear ratio set to %.3f.\n", ratio);
	    }
	    break;
	case dTransmissionIntersectingAxes:
	    if (theta < 0.9) {
		theta += 0.1;

		fprintf (stderr, "Gear angle set to %.3f deg.\n",
			 theta / M_PI * 180);
	    }
	    break;
	case dTransmissionChainDrive:
	    if (rho_2 < 2) {
		rho_2 *= 2;

		fprintf (stderr, "Sprocket ratio set to %.3f.\n", rho_2 / rho_1);
	    }
	    break;
	}

	setup();
    } else if (cmd == '.') {
	speed += 5;

	fprintf (stderr, "Driving wheel speed set to %g rad/s.\n", speed);

	dJointSetHingeParam(hinge2, dParamVel, speed);
    } else if (cmd == ',') {
	speed -= 5;

	fprintf (stderr, "Driving wheel speed set to %g rad/s.\n", speed);

	dJointSetHingeParam(hinge2, dParamVel, speed);
    } else if (cmd == '/') {
	if (dJointGetHingeParam(hinge2, dParamFMax) > 0) {
	    dJointSetHingeParam(hinge2, dParamFMax, 0);
	} else {
	    dJointSetHingeParam(hinge2, dParamFMax, 50);
	}

    } else if (cmd == '-') {
	backlash -= 0.1;

	fprintf (stderr, "Backlash set to %g m.\n", backlash);

        dJointSetTransmissionBacklash(transmission, backlash);
    } else if (cmd == '=') {
	backlash += 0.1;

	fprintf (stderr, "Backlash set to %g m.\n", backlash);

        dJointSetTransmissionBacklash(transmission, backlash);
    } else if (cmd == '1') {
	mode = dTransmissionParallelAxes;
	setup();
    } else if (cmd == '2') {
	mode = dTransmissionIntersectingAxes;
	setup();
    } else if (cmd == '3') {
	mode = dTransmissionChainDrive;
	setup();
    }
}

int main(int argc, char **argv)
{
    // setup pointers to drawstuff callback functions
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start = &start;
    fn.step = &simLoop;
    fn.command = &command;
    fn.stop = stop;
    fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;
    
    // create world
    dInitODE();

    // run demo
    dsSimulationLoop (argc, argv, 800, 600, &fn);

    dCloseODE();
    return 0;
}
