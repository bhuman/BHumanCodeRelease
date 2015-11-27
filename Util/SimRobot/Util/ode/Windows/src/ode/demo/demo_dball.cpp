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
#define dsDrawSphere dsDrawSphereD
#define dsDrawBox dsDrawBoxD
#define dsDrawLine dsDrawLineD
#endif


dWorldID world;
dSpaceID space;
dBodyID body1;
dBodyID body2;
dJointID joint1, joint2;

void start()
{
    world = dWorldCreate();
    dWorldSetGravity (world,0,0,-9.8);

    dWorldSetDamping(world, 1e-4, 1e-5);
//    dWorldSetERP(world, 1);

    space = dSimpleSpaceCreate (0);
    
    body1 = dBodyCreate(world);
    body2 = dBodyCreate(world);

    dBodySetPosition(body1, 0, 0, 3);
    dBodySetPosition(body2, 0, 0, 1);


    dGeomID g;
    dMass mass;
    
    g = dCreateBox(space, 0.2, 0.2, 1);
    dGeomSetBody(g, body1);
    dMassSetBox(&mass, 1, 0.2, 0.2, 1);
    dBodySetMass(body1, &mass);
    
    g = dCreateBox(space, 0.2, 0.2, 1);
    dGeomSetBody(g, body2);
    dMassSetBox(&mass, 1, 0.2, 0.2, 1);
    dBodySetMass(body2, &mass);

    joint1 = dJointCreateDBall(world, 0);
    dJointAttach(joint1, body1, 0);
    dJointSetDBallAnchor1(joint1, 0, 0, 3.5);
    dJointSetDBallAnchor2(joint1, 0, 0, 4.5);

    joint2 = dJointCreateDBall(world, 0);
    dJointAttach(joint2, body1, body2);
    dJointSetDBallAnchor1(joint2, 0, 0, 2.5);
    dJointSetDBallAnchor2(joint2, 0, 0, 1.5);


    // initial camera position
    static float xyz[3] = {3.8966, -2.0614, 4.0300};
    static float hpr[3] = {153.5, -16.5, 0};
    dsSetViewpoint (xyz,hpr);
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
        case dSphereClass:
            dsSetColorAlpha(0, 0.75, 0.5, 1);
            dsSetTexture (DS_CHECKERED);
            dsDrawSphere(pos, rot, dGeomSphereGetRadius(g));
            break;
        case dBoxClass:
        {
            dVector3 lengths;
            dsSetColorAlpha(1, 1, 0, 1);
            dsSetTexture (DS_WOOD);
            dGeomBoxGetLengths(g, lengths);
            dsDrawBox(pos, rot, lengths);
            break;
        }
        
        default:
        {}
    }
}

void simLoop(int pause)
{
    if (!pause) {

        static dReal t = 0;

        const dReal step = 0.005;
        const unsigned nsteps = 4;

        for (unsigned i=0; i<nsteps; ++i) {

            dReal f = sin(t*1.2)*0.8;
            dBodyAddForceAtRelPos(body1,
                                  f, 0, 0, 
                                  0, 0, -0.5); // at the lower end

            dReal g = sin(t*0.7)*0.8;
            dBodyAddForceAtRelPos(body2,
                                  0, g, 0, 
                                  0, 0, -0.5); // at the lower end
            t += step;

            dWorldQuickStep(world, step);
        }
    }
    
    // now we draw everything
    unsigned ngeoms = dSpaceGetNumGeoms(space);
    for (unsigned i=0; i<ngeoms; ++i) {
        dGeomID g = dSpaceGetGeom(space, i);

        drawGeom(g);
    }

    dVector3 a11, a12;
    dJointGetDBallAnchor1(joint1, a11);
    dJointGetDBallAnchor2(joint1, a12);
    dsSetColor(1, 0, 0);
    dsDrawLine(a11, a12);

    //printf("Error 1: %f\n", fabs(dJointGetDBallDistance(joint1) - dCalcPointsDistance3(a11, a12)));

    dVector3 a21, a22;
    dJointGetDBallAnchor1(joint2, a21);
    dJointGetDBallAnchor2(joint2, a22);
    dsSetColor(0, 1, 0);
    dsDrawLine(a21, a22);

    //printf("Error 2: %f\n", fabs(dJointGetDBallDistance(joint2) - dCalcPointsDistance3(a21, a22)));
}



int main(int argc, char **argv)
{
    // setup pointers to drawstuff callback functions
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start = &start;
    fn.step = &simLoop;
    fn.command = 0;
    fn.stop = stop;
    fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;
    
    // create world
    dInitODE();

    // run demo
    dsSimulationLoop (argc, argv, 800, 600, &fn);

    dCloseODE();
    return 0;
}
