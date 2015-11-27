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
  This demo shows how to use dContactMotionN in a lifting platform.
*/
//#include <unistd.h> // for usleep()
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
#define dsDrawConvex dsDrawConvexD
#endif


// some constants

#define NUM 100			// max number of objects
#define DENSITY (5.0)		// density of all objects
#define GPB 3			// maximum number of geometries per body
#define MAX_CONTACTS 8		// maximum number of contact points per body
#define USE_GEOM_OFFSET 1

// dynamics and collision objects

struct MyObject {
    dBodyID body;			// the body
    dGeomID geom[GPB];		// geometries representing this body
};

static int num=0;		// number of objects in simulation
static int nextobj=0;		// next object to recycle if num==NUM
static dWorldID world;
static dSpaceID space;
static MyObject obj[NUM];
static dJointGroupID contactgroup;
static int show_aabb = 0;	// show geom AABBs?
static int show_contacts = 0;	// show contact points?
static int random_pos = 1;	// drop objects from random position?
static int write_world = 0;
static int show_body = 0;

static dGeomID platform, ground;

dVector3 platpos = {0, 0, 0};
int mov_type = 2;
dReal mov_time = 0;


const dReal mov1_speed = 0.2;

dVector3 mov2_vel = { 0.2, 0.1, 0.25};




/****************************************************************
 *  Movement 1: move platform up, reset every 80 units of time. *
 *      This is the simplest case                               *
 ****************************************************************/
static void moveplat_1(dReal stepsize)
{
    mov_time += stepsize;
    if (mov_time > 80)
        mov_time = 0;

    platpos[0] = platpos[1] = 0;
    // the platform moves up (Z) at constant speed: mov1_speed
    platpos[2] = mov1_speed * mov_time;
}

// Generate contact info for movement 1
static void contactplat_1(dContact &contact)
{
    contact.surface.mode |= dContactMotionN;
    contact.surface.motionN = mov1_speed;
}



/****************************************************************
 *  Movement 2: move platform along direction mov2_vel, reset   *
 *  every 80 units of time.                                     *
 *      This is the most general case: the geom moves along     *
 *      an arbitrary direction.                                 *
 ****************************************************************/
static void moveplat_2(dReal stepsize)
{
    mov_time += stepsize;
    if (mov_time > 80)
        mov_time = 0;

    // the platform moves at constant speed: mov2_speed
    platpos[0] = mov2_vel[0] * mov_time;
    platpos[1] = mov2_vel[1] * mov_time;
    platpos[2] = mov2_vel[2] * mov_time;
}

// Generate contact info for movement 1
static void contactplat_2(dContact &contact)
{
    /*
      For arbitrary contact directions we need to project the moving
      geom's velocity against the contact normal and fdir1, fdir2
      (obtained with dPlaneSpace()). Assuming moving geom=g2
      (so the contact joint is in the moving geom's reference frame):
      motion1 = dCalcVectorDot3(fdir1, vel);
      motion2 = dCalcVectorDot3(fdir2, vel);
      motionN = dCalcVectorDot3(normal, vel);

      For geom=g1 just negate motionN and motion2. fdir1 is an arbitrary
      vector, so there's no need to negate motion1.

    */
    contact.surface.mode |= 
        dContactMotionN |                   // velocity along normal
        dContactMotion1 | dContactMotion2 | // and along the contact plane
        dContactFDir1;                      // don't forget to set the direction 1


    // This is a convenience function: given a vector, it finds other 2 perpendicular vectors
    dVector3 motiondir1, motiondir2;
    dPlaneSpace(contact.geom.normal, motiondir1, motiondir2);
    for (int i=0; i<3; ++i)
        contact.fdir1[i] = motiondir1[i];
    

    dReal inv = 1;
    if (contact.geom.g1 == platform)
        inv = -1;
    
    contact.surface.motion1 = dCalcVectorDot3(mov2_vel, motiondir1);
    contact.surface.motion2 = inv * dCalcVectorDot3(mov2_vel, motiondir2);
    contact.surface.motionN = inv * dCalcVectorDot3(mov2_vel, contact.geom.normal);

}





static void nearCallback (void *, dGeomID o1, dGeomID o2)
{
    dMatrix3 RI;
    static const dReal ss[3] = {0.02,0.02,0.02};

    dContact contact[MAX_CONTACTS];
    int numc = dCollide (o1, o2, MAX_CONTACTS,
                         &contact[0].geom, sizeof(dContact));

    if (numc)
        dRSetIdentity(RI);

    bool isplatform = (o1 == platform) || (o2 == platform);

    for (int i=0; i< numc; i++) {
        contact[i].surface.mode = dContactBounce;
        contact[i].surface.mu = 1;
        contact[i].surface.bounce = 0.25;
        contact[i].surface.bounce_vel = 0.01;
        
        if (isplatform) {
            switch (mov_type) {
                case 1:
                    contactplat_1(contact[i]);
                    break;
                case 2:
                    contactplat_2(contact[i]);
                    break;
            }
        }

        dJointID c = dJointCreateContact (world,contactgroup,contact+i);
        dJointAttach (c, dGeomGetBody(o1), dGeomGetBody(o2));
        if (show_contacts) 
            dsDrawBox (contact[i].geom.pos, RI, ss);
    }
}


// start simulation - set viewpoint

static float xyz[3] = {2.1106f,-1.3007,2.f};
static float hpr[3] = {150.f,-13.5000f,0.0000f};

static void start()
{
    //dAllocateODEDataForThread(dAllocateMaskAll);
    dsSetViewpoint (xyz,hpr);
    printf ("To drop another object, press:\n");
    printf ("   b for box.\n");
    printf ("   s for sphere.\n");
    printf ("   c for capsule.\n");
    printf ("   y for cylinder.\n");
    printf ("Press m to change the movement type\n");
    printf ("Press space to reset the platform\n");
    printf ("To toggle showing the geom AABBs, press a.\n");
    printf ("To toggle showing the contact points, press t.\n");
    printf ("To toggle dropping from random position/orientation, press r.\n");
    printf ("To save the current state to 'state.dif', press 1.\n");
}


char locase (char c)
{
    if (c >= 'A' && c <= 'Z') return c - ('a'-'A');
    else return c;
}


// called when a key pressed

static void command (int cmd)
{
    size_t i;
    int k;
    dReal sides[3];
    dMass m;
    int setBody;
  
    cmd = locase (cmd);
    if (cmd == 'b' || cmd == 's' || cmd == 'c' || cmd == 'y')
        {
            setBody = 0;
            if (num < NUM) {
                i = num;
                num++;
            }
            else {
                i = nextobj;
                nextobj++;
                if (nextobj >= num) nextobj = 0;

                // destroy the body and geoms for slot i
                if (obj[i].body) {
                  dBodyDestroy (obj[i].body);
                }
                for (k=0; k < GPB; k++) {
                    if (obj[i].geom[k]) {
                      dGeomDestroy (obj[i].geom[k]);
                    }
                }
                memset (&obj[i],0,sizeof(obj[i]));
            }

            obj[i].body = dBodyCreate (world);
            for (k=0; k<3; k++) sides[k] = dRandReal()*0.5+0.1;

            dMatrix3 R;
            if (random_pos) 
                {
                    dBodySetPosition (obj[i].body,
                                      dRandReal()*2-1 + platpos[0],
                                      dRandReal()*2-1 + platpos[1],
                                      dRandReal()+2 + platpos[2]);
                    dRFromAxisAndAngle (R,dRandReal()*2.0-1.0,dRandReal()*2.0-1.0,
                                        dRandReal()*2.0-1.0,dRandReal()*10.0-5.0);
                }
            else 
                {
                    dBodySetPosition (obj[i].body, 
                                      platpos[0],
                                      platpos[1],
                                      platpos[2]+2);
                    dRSetIdentity (R);
                }
            dBodySetRotation (obj[i].body,R);
            dBodySetData (obj[i].body,(void*) i);

            if (cmd == 'b') {
                dMassSetBox (&m,DENSITY,sides[0],sides[1],sides[2]);
                obj[i].geom[0] = dCreateBox (space,sides[0],sides[1],sides[2]);
            }
            else if (cmd == 'c') {
                sides[0] *= 0.5;
                dMassSetCapsule (&m,DENSITY,3,sides[0],sides[1]);
                obj[i].geom[0] = dCreateCapsule (space,sides[0],sides[1]);
            }
            else if (cmd == 'y') {
                dMassSetCylinder (&m,DENSITY,3,sides[0],sides[1]);
                obj[i].geom[0] = dCreateCylinder (space,sides[0],sides[1]);
            }
            else if (cmd == 's') {
                sides[0] *= 0.5;
                dMassSetSphere (&m,DENSITY,sides[0]);
                obj[i].geom[0] = dCreateSphere (space,sides[0]);
            }

            if (!setBody)
                for (k=0; k < GPB; k++) {
                    if (obj[i].geom[k]) {
                      dGeomSetBody (obj[i].geom[k],obj[i].body);
                    }
                }

            dBodySetMass (obj[i].body,&m);
        }
    else if (cmd == 'a') {
        show_aabb ^= 1;
    }
    else if (cmd == 't') {
        show_contacts ^= 1;
    }
    else if (cmd == 'r') {
        random_pos ^= 1;
    }
    else if (cmd == '1') {
        write_world = 1;
    }
    else if (cmd == ' ') {
        mov_time = 0;
    }
    else if (cmd == 'm') {
        mov_type = mov_type==1 ? 2 : 1;
        mov_time = 0;
    }
}


// draw a geom

void drawGeom (dGeomID g, const dReal *pos, const dReal *R, int show_aabb)
{
    int i;
	
    if (!g) return;
    if (!pos) pos = dGeomGetPosition (g);
    if (!R) R = dGeomGetRotation (g);

    int type = dGeomGetClass (g);
    if (type == dBoxClass) {
        dVector3 sides;
        dGeomBoxGetLengths (g,sides);
        dsDrawBox (pos,R,sides);
    }
    else if (type == dSphereClass) {
        dsDrawSphere (pos,R,dGeomSphereGetRadius (g));
    }
    else if (type == dCapsuleClass) {
        dReal radius,length;
        dGeomCapsuleGetParams (g,&radius,&length);
        dsDrawCapsule (pos,R,length,radius);
    }
    else if (type == dCylinderClass) {
        dReal radius,length;
        dGeomCylinderGetParams (g,&radius,&length);
        dsDrawCylinder (pos,R,length,radius);
    }

    if (show_body) {
        dBodyID body = dGeomGetBody(g);
        if (body) {
            const dReal *bodypos = dBodyGetPosition (body); 
            const dReal *bodyr = dBodyGetRotation (body); 
            dReal bodySides[3] = { 0.1, 0.1, 0.1 };
            dsSetColorAlpha(0,1,0,1);
            dsDrawBox(bodypos,bodyr,bodySides); 
        }
    }
    if (show_aabb) {
        // draw the bounding box for this geom
        dReal aabb[6];
        dGeomGetAABB (g,aabb);
        dVector3 bbpos;
        for (i=0; i<3; i++) bbpos[i] = 0.5*(aabb[i*2] + aabb[i*2+1]);
        dVector3 bbsides;
        for (i=0; i<3; i++) bbsides[i] = aabb[i*2+1] - aabb[i*2];
        dMatrix3 RI;
        dRSetIdentity (RI);
        dsSetColorAlpha (1,0,0,0.5);
        dsDrawBox (bbpos,RI,bbsides);
    }
}


// simulation loop

static void updatecam()
{
    xyz[0] = platpos[0] + 3.3;
    xyz[1] = platpos[1] - 1.8;
    xyz[2] = platpos[2] + 2;
    dsSetViewpoint (xyz, hpr);
}

static void simLoop (int pause)
{
    const dReal stepsize = 0.02;

    dsSetColor (0,0,2);
    dSpaceCollide (space,0,&nearCallback);
    if (!pause) {
        
        if (mov_type == 1)
            moveplat_1(stepsize);
        else
            moveplat_2(stepsize);

        dGeomSetPosition(platform, platpos[0], platpos[1], platpos[2]);
        updatecam();
        dWorldQuickStep (world,stepsize);
        //dWorldStep (world,stepsize);
    }

    if (write_world) {
        FILE *f = fopen ("state.dif","wt");
        if (f) {
            dWorldExportDIF (world,f,"X");
            fclose (f);
        }
        write_world = 0;
    }
  
    // remove all contact joints
    dJointGroupEmpty (contactgroup);

    dsSetColor (1,1,0);
    dsSetTexture (DS_WOOD);
    for (int i=0; i<num; i++) {
        for (int j=0; j < GPB; j++) {
            if (! dBodyIsEnabled (obj[i].body)) {
                dsSetColor (1,0.8,0);
            }
            else {
                dsSetColor (1,1,0);
            }
            drawGeom (obj[i].geom[j],0,0,show_aabb);
        }
    }
    dsSetColor (1,0,0);
    drawGeom (platform,0,0,show_aabb);
    //usleep(5000);
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

    // create world
    dInitODE();
    world = dWorldCreate();

#if 1
    space = dHashSpaceCreate (0);
#elif 0
    dVector3 center = {0,0,0}, extents = { 100, 100, 100};
    space = dQuadTreeSpaceCreate(0, center, extents, 5);
#elif 0
    space = dSweepAndPruneSpaceCreate (0, dSAP_AXES_XYZ);
#else
    space = dSimpleSpaceCreate(0);
#endif

    contactgroup = dJointGroupCreate (0);
    dWorldSetGravity (world,0,0,-0.5);
    dWorldSetCFM (world,1e-5);
    
    dWorldSetLinearDamping(world, 0.00001);
    dWorldSetAngularDamping(world, 0.005);
    dWorldSetMaxAngularSpeed(world, 200);

    dWorldSetContactSurfaceLayer (world,0.001);
    ground = dCreatePlane (space,0,0,1,0);
    
    memset (obj,0,sizeof(obj));

    // create lift platform
    platform = dCreateBox(space, 4, 4, 1);

    dGeomSetCategoryBits(ground, 1ul);
    dGeomSetCategoryBits(platform, 2ul);
    dGeomSetCollideBits(ground, ~2ul);
    dGeomSetCollideBits(platform, ~1ul);

    // run simulation
    dsSimulationLoop (argc,argv,352,288,&fn);

    dJointGroupDestroy (contactgroup);
    dSpaceDestroy (space);
    dWorldDestroy (world);
    dCloseODE();
    return 0;
}


// Local Variables:
// c-basic-offset:4
// End:
