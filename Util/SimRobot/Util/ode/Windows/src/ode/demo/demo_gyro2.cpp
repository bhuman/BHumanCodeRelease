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

// dynamics and collision objects
static dWorldID world = 0;

static const dReal dt = 1/REAL(60.0); // 60 fps
// Water density if units are meters and kg
static const dReal density = 1000;  

// A long skinny thing
static dVector3 sides = {2,.5,.25}; 
// Initial angular velocity
static dVector3 omega = {5,1,2};
static dVector3 torque = {0,10,0};
static dBodyID noGyroBody;
static dBodyID expGyroBody;
static dBodyID impGyroBody;

// start simulation - set viewpoint

static void start()
{
  dAllocateODEDataForThread(dAllocateMaskAll);

  static float xyz[3] = {0,-4.0f,3.0f};
  static float hpr[3] = {90.0000,-15.0000,0.0000};
  dsSetViewpoint (xyz,hpr);
  printf ("Press:\n"
      "\t'a' to apply a torque\n"
			"\t'r' to reset simulation.\n");
}

/**
  Delete the bodies, etc.
*/
static void clear()
{
  if (world) dWorldDestroy (world);
  world = 0;
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

  // Calculate mass for a box;
  dMass boxMass;
  dMassSetBox(&boxMass,density,sides[0],sides[1],sides[2]);

  noGyroBody  = dBodyCreate(world);// Conservation of ang-velocity
  expGyroBody = dBodyCreate(world);// Explicit conservation of ang-momentum
  impGyroBody = dBodyCreate(world);// Implicit conservation of ang-momentum

  dBodySetMass( noGyroBody , &boxMass );
  dBodySetMass( expGyroBody, &boxMass );
  dBodySetMass( impGyroBody, &boxMass );

  // Try to avoid collisions.
  dReal sep = dCalcVectorLength3(sides);
  dBodySetPosition( noGyroBody , -sep, 0, sep);
  dBodySetPosition( expGyroBody,    0, 0, sep);
  dBodySetPosition( impGyroBody,  sep, 0, sep);

  // Set the initial angular velocity
  dBodySetAngularVel( noGyroBody , omega[0], omega[1], omega[2]);
  dBodySetAngularVel( expGyroBody, omega[0], omega[1], omega[2]);
  dBodySetAngularVel( impGyroBody, omega[0], omega[1], omega[2]);

  dBodySetGyroscopicMode( noGyroBody, 0);
  // We compute this ourselves using the math
  // that was in the old stepper.
  dBodySetGyroscopicMode(expGyroBody, 0); 
  // Keep things from crashing by limiting
  // the angular speed of the explicit body.
  // Note that this isn't necessary for
  // the other two bodies.
  dBodySetMaxAngularSpeed( expGyroBody, 40 );
}

static void command (int cmd)
{
	switch (cmd) {
    case 'a': case 'A':
      dBodyAddTorque( noGyroBody, torque[0], torque[1], torque[2]);
      dBodyAddTorque(expGyroBody, torque[0], torque[1], torque[2]);
      dBodyAddTorque(impGyroBody, torque[0], torque[1], torque[2]);
      break;
    case 'r': case 'R':
  		reset();
	  	break;
  }
  
}

/**
  This is the explicit computation of
  gyroscopic forces.
*/
static void expStep(dBodyID body)
{
  // Explicit computation
  dMatrix3 I,tmp;
  dMass m;
  dBodyGetMass(body,&m);
  const dReal* R = dBodyGetRotation(body);
  // compute inertia tensor in global frame
  dMultiply2_333 (tmp,m.I,R);
  dMultiply0_333 (I,R,tmp);
  // compute explicit rotational force
  // we treat 'tmp'like a vector, but that's okay.
  const dReal* w = dBodyGetAngularVel(body);
  dMultiply0_331 (tmp,I,w);
  dVector3 tau;
  dCalcVectorCross3(tau,tmp,w);
  dBodyAddTorque(body,tau[0],tau[1],tau[2]);
}


// simulation loop
static void simLoop (int pause)
{
  if (!pause) {
    expStep(expGyroBody);
    dWorldStep (world,dt); 
  }

  dsSetTexture (DS_WOOD);
  dsSetColor(1,0,0);
  dsDrawBox(dBodyGetPosition(noGyroBody ),dBodyGetRotation(noGyroBody ),sides);
  dsSetColor(1,1,0);
  dsDrawBox(dBodyGetPosition(expGyroBody),dBodyGetRotation(expGyroBody),sides);
  dsSetColor(0,1,0);
  dsDrawBox(dBodyGetPosition(impGyroBody),dBodyGetRotation(impGyroBody),sides);
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
