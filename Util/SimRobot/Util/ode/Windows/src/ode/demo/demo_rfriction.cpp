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

/*
Angular friction demo:

A bunch of ramps of different pitch.
A bunch of spheres with rolling friction.
*/


#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

// select correct drawing functions
#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#endif


// some constants
#define GRAVITY 10	// the global gravity to use
#define RAMP_COUNT 8

static const dReal rampX = 6.0f;
static const dReal rampY = 0.5f;
static const dReal rampZ = 0.25f;
static const dReal sphereRadius = 0.25f;
static const dReal maxRamp = M_PI/4.0f; // Needs to be less than pi/2
static const dReal rampInc = maxRamp/RAMP_COUNT;

// dynamics and collision objects
static dWorldID world = 0;
static dSpaceID space = 0;
static dJointGroupID contactgroup = 0;
static dGeomID ground;

static dReal mu = REAL(0.37); // the global mu to use
static dReal rho = REAL(0.1); // the global rho to use
static dReal omega = REAL(25.0);

static dGeomID rampGeom[RAMP_COUNT];
static dBodyID sphereBody[RAMP_COUNT];
static dGeomID sphereGeom[RAMP_COUNT];


// this is called by dSpaceCollide when two objects in space are
// potentially colliding.

static void nearCallback (void *, dGeomID o1, dGeomID o2)
{
    int i;

    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
  
    if (b1==0 && b2==0) return;

    dContact contact[3];		
    for (int ii=0; ii<3; ii++) {
        contact[ii].surface.mode = dContactApprox1 | dContactRolling;
        contact[ii].surface.mu = mu;
        contact[ii].surface.rho = rho;
    }
    if (int numc = dCollide (o1,o2,3,&contact[0].geom,sizeof(dContact))) {
        for (i=0; i<numc; i++) {
            dJointID c = dJointCreateContact (world,contactgroup,contact+i);
            dJointAttach (c,b1,b2);
        }
    }
}


// start simulation - set viewpoint

static void start()
{
    dAllocateODEDataForThread(dAllocateMaskAll);

    static float xyz[3] = {0,-3.0f,3.0f};
    static float hpr[3] = {90.0000,-15.0000,0.0000};
    dsSetViewpoint (xyz,hpr);
    printf ("Press:\n"
            "\t'[' or ']' to change initial angular velocity\n"
            "\t'm' to increase sliding friction\n"
            "\t'n' to decrease sliding friction\n"
            "\t'j' to increase rolling friction\n"
            "\t'h' to decrease rolling friction\n"
            "\t'r' to reset simulation.\n");
}

/**
  Delete the bodies, etc.
*/
static void clear()
{
    if (contactgroup) dJointGroupDestroy (contactgroup);
    if (space) dSpaceDestroy (space);
    if (world) dWorldDestroy (world);
}



/**
  Cleanup if necessary and rebuild the
  world.
*/
static void reset()
{
    clear();

    // create world
    world = dWorldCreate();
    space = dHashSpaceCreate (0);
    contactgroup = dJointGroupCreate (0);
    dWorldSetGravity (world,0,0,-GRAVITY);
    ground = dCreatePlane (space,0,0,1,0);

    // Calculate mass for sphere a capsule with water density.
    dMass sphereMass;
    dMassSetSphere(&sphereMass,1000,sphereRadius);

    for (int ii=0;ii<RAMP_COUNT;++ii) {
        dQuaternion q;

        dReal angle = (ii+1)*rampInc;
        dReal cosA = dCos(angle);
        dReal sinA = dSin(angle);
        dReal rampW = rampX/cosA; // Box width that preserves ground distance
        dReal zPos = REAL(0.5)*(sinA*rampW-cosA*rampZ); // Position that makes end meet ground
        dReal yPos = ii*1.25*rampY;
        dReal xPos = 0;
    
    
        // Create the ramp
        rampGeom[ii] = dCreateBox(space,rampW,rampY,rampZ);
        dQFromAxisAndAngle(q,0,1,0,angle);
        dGeomSetQuaternion(rampGeom[ii],q);
        dGeomSetPosition(rampGeom[ii],xPos,yPos,zPos);

        // Create the spheres
        xPos = -REAL(0.5)*rampX + sphereRadius;
        zPos = sinA*rampW + sphereRadius;
        sphereBody[ii] = dBodyCreate(world);
        dBodySetMass(sphereBody[ii],&sphereMass);
        sphereGeom[ii] = dCreateSphere(space,sphereRadius);
        dGeomSetBody(sphereGeom[ii],sphereBody[ii]);
        dBodySetPosition(sphereBody[ii],xPos,yPos,zPos);
        dBodySetAngularVel(sphereBody[ii],0,omega,0);
    }
}


static void command (int cmd)
{
    switch (cmd) {
        case 'h': case 'H':
            rho-=0.02; 
            if (rho<0) rho=0;
            break;	
        case 'j': case 'J':
            rho+=0.02;
            if (rho>1) rho=1;
            break;
        case 'n': case 'N':
            mu-=0.02;
            if (mu<0) mu=0;
            break;
        case 'm': case 'M':
            mu+=0.02;
            if (mu>1) mu=1;
            break;
        case 'r': case 'R':
            reset();
            break;
        case ']':
            omega+=1;
            break;
        case '[':
            omega-=1;
            break;
    }
}

// simulation loop

static void simLoop (int pause)
{
    if (!pause) {
        dSpaceCollide (space,0,&nearCallback);
        dWorldStep (world,0.017); // 60 fps
        // remove all contact joints
        dJointGroupEmpty (contactgroup);
    }

    // Render ramps and spheres
    dsSetTexture (DS_WOOD);
    for (int ii=0;ii<RAMP_COUNT;++ii) {
        dVector3 sides;

        dsSetColor (1,0.5,0);
        dGeomBoxGetLengths(rampGeom[ii],sides);
        dsDrawBox (dGeomGetPosition(rampGeom[ii]),dGeomGetRotation(rampGeom[ii]),sides);

        dsSetColor(0,0,1);
        dsDrawSphere (dGeomGetPosition(sphereGeom[ii]),dGeomGetRotation(sphereGeom[ii]), sphereRadius);
    }
}


int main (int argc, char **argv)
{
    // setup pointers to drawstuff callback functions
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start = &start;
    fn.step = &simLoop;
    fn.command = &command;
    fn.stop = 0;
    fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;

    dInitODE2(0);
    reset();

    // run simulation
    dsSimulationLoop (argc,argv,352,288,&fn);

    clear();
    dCloseODE();
    return 0;
}
