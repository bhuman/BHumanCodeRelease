/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001-2003 Russell L. Smith.       *
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
#include "bunny_geom.h"

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

// select correct drawing functions

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawLine dsDrawLineD
#define dsDrawTriangle dsDrawTriangleD
#endif


// some constants

#define NUM 200			// max number of objects
#define DENSITY (5.0)		// density of all objects
#define GPB 3			// maximum number of geometries per body
#define MAX_CONTACTS 64		// maximum number of contact points per body


// dynamics and collision objects

struct MyObject {
  dBodyID body;			// the body
  dGeomID geom[GPB];		// geometries representing this body

  // Trimesh only - double buffered matrices for 'last transform' setup
  dReal matrix_dblbuff[ 16 * 2 ];
  int last_matrix_index;
};

static int num=0;		// number of objects in simulation
static int nextobj=0;		// next object to recycle if num==NUM
static dWorldID world;
static dSpaceID space;
static MyObject obj[NUM];
static dJointGroupID contactgroup;
static int selected = -1;	// selected object
static int show_aabb = 0;	// show geom AABBs?
static int show_contacts = 0;	// show contact points?
static int random_pos = 1;	// drop objects from random position?

typedef dReal dVector3R[3];

dGeomID TriMesh1;
dGeomID TriMesh2;
static dTriMeshDataID TriData1, TriData2;  // reusable static trimesh data

// this is called by dSpaceCollide when two objects in space are
// potentially colliding.

static void nearCallback (void *, dGeomID o1, dGeomID o2)
{
  int i;
  // if (o1->body && o2->body) return;

  // exit without doing anything if the two bodies are connected by a joint
  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  if (b1 && b2 && dAreConnectedExcluding (b1,b2,dJointTypeContact)) return;

  dContact contact[MAX_CONTACTS];   // up to MAX_CONTACTS contacts per box-box
  for (i=0; i<MAX_CONTACTS; i++) {
    contact[i].surface.mode = dContactBounce | dContactSoftCFM;
    contact[i].surface.mu = dInfinity;
    contact[i].surface.mu2 = 0;
    contact[i].surface.bounce = 0.1;
    contact[i].surface.bounce_vel = 0.1;
    contact[i].surface.soft_cfm = 0.01;
  }
  if (int numc = dCollide (o1,o2,MAX_CONTACTS,&contact[0].geom,
			   sizeof(dContact))) {
    dMatrix3 RI;
    dRSetIdentity (RI);
    const dReal ss[3] = {0.02,0.02,0.02};
    for (i=0; i<numc; i++) {
      dJointID c = dJointCreateContact (world,contactgroup,contact+i);
      dJointAttach (c,b1,b2);
      if (show_contacts) dsDrawBox (contact[i].geom.pos,RI,ss);
    }
  }
}


// start simulation - set viewpoint

static void start()
{
  dAllocateODEDataForThread(dAllocateMaskAll);

  static float xyz[3] = {2.1640f,-1.3079f,1.7600f};
  static float hpr[3] = {125.5000f,-17.0000f,0.0000f};
  dsSetViewpoint (xyz,hpr);
  printf ("To drop another object, press:\n");
  printf ("   b for box.\n");
  printf ("   s for sphere.\n");
  printf ("   y for cylinder.\n");
  printf ("   c for capsule.\n");
  printf ("   x for a composite object.\n");
  printf ("   m for a trimesh.\n");
  printf ("To select an object, press space.\n");
  printf ("To disable the selected object, press d.\n");
  printf ("To enable the selected object, press e.\n");
  printf ("To toggle showing the geom AABBs, press a.\n");
  printf ("To toggle showing the contact points, press t.\n");
  printf ("To toggle dropping from random position/orientation, press r.\n");
}


char locase (char c)
{
  if (c >= 'A' && c <= 'Z') return c - ('a'-'A');
  else return c;
}


// called when a key pressed

static void command (int cmd)
{
    int i,j,k;
    dReal sides[3];
    dMass m;
    bool setBody = false;

    cmd = locase (cmd);
    if (cmd == 'b' || cmd == 's' || cmd == 'c' || cmd == 'x' || cmd == 'm' || cmd == 'y' ) {
        if (num < NUM) {
            i = num;
            num++;
        }
        else {
            i = nextobj;
            nextobj++;
            if (nextobj >= num) nextobj = 0;

            // destroy the body and geoms for slot i
            dBodyDestroy (obj[i].body);
            for (k=0; k < GPB; k++) {
                if (obj[i].geom[k]) dGeomDestroy (obj[i].geom[k]);
            }
            memset (&obj[i],0,sizeof(obj[i]));
        }

        obj[i].body = dBodyCreate (world);
        for (k=0; k<3; k++) sides[k] = dRandReal()*0.5+0.1;

        dMatrix3 R;
        if (random_pos) {
            dBodySetPosition (obj[i].body,
                              dRandReal()*2-1,dRandReal()*2-1,dRandReal()+3);
            dRFromAxisAndAngle (R,dRandReal()*2.0-1.0,dRandReal()*2.0-1.0,
                                dRandReal()*2.0-1.0,dRandReal()*10.0-5.0);
        }
        else {
            dReal maxheight = 0;
            for (k=0; k<num; k++) {
                const dReal *pos = dBodyGetPosition (obj[k].body);
                if (pos[2] > maxheight) maxheight = pos[2];
            }
            dBodySetPosition (obj[i].body, 0,0,maxheight+1);
            dRFromAxisAndAngle (R,0,0,1,dRandReal()*10.0-5.0);
        }
        dBodySetRotation (obj[i].body,R);
        dBodySetData (obj[i].body,(void*)(size_t)i);

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
            sides[1] *= 0.5;
            dMassSetCylinder (&m,DENSITY,3,sides[0],sides[1]);
            obj[i].geom[0] = dCreateCylinder (space,sides[0],sides[1]);
        }
	else if (cmd == 's') {
            sides[0] *= 0.5;
            dMassSetSphere (&m,DENSITY,sides[0]);
            obj[i].geom[0] = dCreateSphere (space,sides[0]);
        }
        else if (cmd == 'm') {
            dTriMeshDataID new_tmdata = dGeomTriMeshDataCreate();
            dGeomTriMeshDataBuildSingle(new_tmdata, &Vertices[0], 3 * sizeof(float), VertexCount, 
                                        (dTriIndex*)&Indices[0], IndexCount, 3 * sizeof(dTriIndex));

            obj[i].geom[0] = dCreateTriMesh(space, new_tmdata, 0, 0, 0);

            // remember the mesh's dTriMeshDataID on its userdata for convenience.
            dGeomSetData(obj[i].geom[0], new_tmdata);

            dMassSetTrimesh( &m, DENSITY, obj[i].geom[0] );
            printf("mass at %f %f %f\n", m.c[0], m.c[1], m.c[2]);
            dGeomSetPosition(obj[i].geom[0], -m.c[0], -m.c[1], -m.c[2]);
            dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
        }
        else if (cmd == 'x') {

            setBody = true;
            // start accumulating masses for the composite geometries
            dMass m2;
            dMassSetZero (&m);

            dReal dpos[GPB][3];	// delta-positions for composite geometries
            dMatrix3 drot[GPB];
      
            // set random delta positions
            for (j=0; j<GPB; j++)
                for (k=0; k<3; k++)
                    dpos[j][k] = dRandReal()*0.3-0.15;
    
            for (k=0; k<GPB; k++) {
                if (k==0) {
                    dReal radius = dRandReal()*0.25+0.05;
                    obj[i].geom[k] = dCreateSphere (space,radius);
                    dMassSetSphere (&m2,DENSITY,radius);
                } else if (k==1) {
                    obj[i].geom[k] = dCreateBox(space,sides[0],sides[1],sides[2]);
                    dMassSetBox(&m2,DENSITY,sides[0],sides[1],sides[2]);
                } else {
                    dReal radius = dRandReal()*0.1+0.05;
                    dReal length = dRandReal()*1.0+0.1;
                    obj[i].geom[k] = dCreateCapsule(space,radius,length);
                    dMassSetCapsule(&m2,DENSITY,3,radius,length);
                }

                dRFromAxisAndAngle(drot[k],dRandReal()*2.0-1.0,dRandReal()*2.0-1.0,
                                   dRandReal()*2.0-1.0,dRandReal()*10.0-5.0);
                dMassRotate(&m2,drot[k]);
		
                dMassTranslate(&m2,dpos[k][0],dpos[k][1],dpos[k][2]);

                // add to the total mass
                dMassAdd(&m,&m2);

            }
            for (k=0; k<GPB; k++) {
                dGeomSetBody(obj[i].geom[k],obj[i].body);
                dGeomSetOffsetPosition(obj[i].geom[k],
                                       dpos[k][0]-m.c[0],
                                       dpos[k][1]-m.c[1],
                                       dpos[k][2]-m.c[2]);
                dGeomSetOffsetRotation(obj[i].geom[k], drot[k]);
            }
            dMassTranslate(&m,-m.c[0],-m.c[1],-m.c[2]);
            dBodySetMass(obj[i].body,&m);

        }

        if (!setBody) { // avoid calling for composite geometries
            for (k=0; k < GPB; k++)
                if (obj[i].geom[k])
                    dGeomSetBody(obj[i].geom[k],obj[i].body);

            dBodySetMass(obj[i].body,&m);
        }
    }

    if (cmd == ' ') {
        selected++;
        if (selected >= num) selected = 0;
        if (selected < 0) selected = 0;
    }
    else if (cmd == 'd' && selected >= 0 && selected < num) {
        dBodyDisable (obj[selected].body);
    }
    else if (cmd == 'e' && selected >= 0 && selected < num) {
        dBodyEnable (obj[selected].body);
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
}


// draw a geom

void drawGeom (dGeomID g, const dReal *pos, const dReal *R, int show_aabb)
{
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

  if (show_aabb) {
    // draw the bounding box for this geom
    dReal aabb[6];
    dGeomGetAABB (g,aabb);
    dVector3 bbpos;
    for (int i=0; i<3; i++) bbpos[i] = 0.5*(aabb[i*2] + aabb[i*2+1]);
    dVector3 bbsides;
    for (int j=0; j<3; j++) bbsides[j] = aabb[j*2+1] - aabb[j*2];
    dMatrix3 RI;
    dRSetIdentity (RI);
    dsSetColorAlpha (1,0,0,0.5);
    dsDrawBox (bbpos,RI,bbsides);
  }
}


// set previous transformation matrix for trimesh
void setCurrentTransform(dGeomID geom)
{
 const dReal* Pos = dGeomGetPosition(geom);
 const dReal* Rot = dGeomGetRotation(geom);

 const dReal Transform[16] = 
 {
   Rot[0], Rot[4], Rot[8],  0,
   Rot[1], Rot[5], Rot[9],  0,
   Rot[2], Rot[6], Rot[10], 0,
   Pos[0], Pos[1], Pos[2],  1
 };

 dGeomTriMeshSetLastTransform( geom, *(dMatrix4*)(&Transform) );

}


// simulation loop

static void simLoop (int pause)
{
  dsSetColor (0,0,2);
  dSpaceCollide (space,0,&nearCallback);


#if 1
  // What is this for??? - Bram
  if (!pause) 
  {
    for (int i=0; i<num; i++)
      for (int j=0; j < GPB; j++)
        if (obj[i].geom[j])
          if (dGeomGetClass(obj[i].geom[j]) == dTriMeshClass)
            setCurrentTransform(obj[i].geom[j]);
 
    setCurrentTransform(TriMesh1);
    setCurrentTransform(TriMesh2);
  }
#endif

  //if (!pause) dWorldStep (world,0.05);
  if (!pause) dWorldQuickStep (world,0.05);

  for (int j = 0; j < dSpaceGetNumGeoms(space); j++){
	  dSpaceGetGeom(space, j);
  }

  // remove all contact joints
  dJointGroupEmpty (contactgroup);

  dsSetColor (1,1,0);
  dsSetTexture (DS_WOOD);
  for (int i=0; i<num; i++) {
    for (int j=0; j < GPB; j++) {
      if (obj[i].geom[j]) {
        if (i==selected) {
          dsSetColor (0,0.7,1);
        }
        else if (! dBodyIsEnabled (obj[i].body)) {
          dsSetColor (1,0,0);
        }
        else {
          dsSetColor (1,1,0);
        }
      
        if (dGeomGetClass(obj[i].geom[j]) == dTriMeshClass) {
          dTriIndex* Indices = (dTriIndex*)::Indices;

          // assume all trimeshes are drawn as bunnies
          const dReal* Pos = dGeomGetPosition(obj[i].geom[j]);
          const dReal* Rot = dGeomGetRotation(obj[i].geom[j]);
        
          for (int ii = 0; ii < IndexCount / 3; ii++) {
            const dReal v[9] = { // explicit conversion from float to dReal
              Vertices[Indices[ii * 3 + 0] * 3 + 0],
              Vertices[Indices[ii * 3 + 0] * 3 + 1],
              Vertices[Indices[ii * 3 + 0] * 3 + 2],
              Vertices[Indices[ii * 3 + 1] * 3 + 0],
              Vertices[Indices[ii * 3 + 1] * 3 + 1],
              Vertices[Indices[ii * 3 + 1] * 3 + 2],
              Vertices[Indices[ii * 3 + 2] * 3 + 0],
              Vertices[Indices[ii * 3 + 2] * 3 + 1],
              Vertices[Indices[ii * 3 + 2] * 3 + 2]
            };
            dsDrawTriangle(Pos, Rot, &v[0], &v[3], &v[6], 1);
          }

          // tell the tri-tri collider the current transform of the trimesh --
          // this is fairly important for good results.
          
		  // Fill in the (4x4) matrix.
		  dReal* p_matrix = obj[i].matrix_dblbuff + ( obj[i].last_matrix_index * 16 );

		  p_matrix[ 0 ] = Rot[ 0 ];	p_matrix[ 1 ] = Rot[ 1 ];	p_matrix[ 2 ] = Rot[ 2 ];	p_matrix[ 3 ] = 0;
		  p_matrix[ 4 ] = Rot[ 4 ];	p_matrix[ 5 ] = Rot[ 5 ];	p_matrix[ 6 ] = Rot[ 6 ];	p_matrix[ 7 ] = 0;
		  p_matrix[ 8 ] = Rot[ 8 ];	p_matrix[ 9 ] = Rot[ 9 ];	p_matrix[10 ] = Rot[10 ];	p_matrix[11 ] = 0;
		  p_matrix[12 ] = Pos[ 0 ];	p_matrix[13 ] = Pos[ 1 ];	p_matrix[14 ] = Pos[ 2 ];	p_matrix[15 ] = 1;

		  // Flip to other matrix.
		  obj[i].last_matrix_index = !obj[i].last_matrix_index;

		  dGeomTriMeshSetLastTransform( obj[i].geom[j], 
			  *(dMatrix4*)( obj[i].matrix_dblbuff + obj[i].last_matrix_index * 16 ) );
  
        } else {
          drawGeom (obj[i].geom[j],0,0,show_aabb);
        }
      }
    }
  }

  dTriIndex* Indices = (dTriIndex*)::Indices;

  {const dReal* Pos = dGeomGetPosition(TriMesh1);
  const dReal* Rot = dGeomGetRotation(TriMesh1);

  {for (int i = 0; i < IndexCount / 3; i++){
    const dReal v[9] = { // explicit conversion from float to dReal
      Vertices[Indices[i * 3 + 0] * 3 + 0],
      Vertices[Indices[i * 3 + 0] * 3 + 1],
      Vertices[Indices[i * 3 + 0] * 3 + 2],
      Vertices[Indices[i * 3 + 1] * 3 + 0],
      Vertices[Indices[i * 3 + 1] * 3 + 1],
      Vertices[Indices[i * 3 + 1] * 3 + 2],
      Vertices[Indices[i * 3 + 2] * 3 + 0],
      Vertices[Indices[i * 3 + 2] * 3 + 1],
      Vertices[Indices[i * 3 + 2] * 3 + 2]
    };
    dsDrawTriangle(Pos, Rot, &v[0], &v[3], &v[6], 0);
  }}}

  {const dReal* Pos = dGeomGetPosition(TriMesh2);
  const dReal* Rot = dGeomGetRotation(TriMesh2);

  {for (int i = 0; i < IndexCount / 3; i++){
    const dReal v[9] = { // explicit conversion from float to dReal
      Vertices[Indices[i * 3 + 0] * 3 + 0],
      Vertices[Indices[i * 3 + 0] * 3 + 1],
      Vertices[Indices[i * 3 + 0] * 3 + 2],
      Vertices[Indices[i * 3 + 1] * 3 + 0],
      Vertices[Indices[i * 3 + 1] * 3 + 1],
      Vertices[Indices[i * 3 + 1] * 3 + 2],
      Vertices[Indices[i * 3 + 2] * 3 + 0],
      Vertices[Indices[i * 3 + 2] * 3 + 1],
      Vertices[Indices[i * 3 + 2] * 3 + 2]
    };
    dsDrawTriangle(Pos, Rot, &v[0], &v[3], &v[6], 1);
  }}}
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
  dInitODE2(0);
  world = dWorldCreate();
 
  space = dSimpleSpaceCreate(0);
  contactgroup = dJointGroupCreate (0);
  dWorldSetGravity (world,0,0,-0.5);
  dWorldSetCFM (world,1e-5);
  dCreatePlane (space,0,0,1,0);
  memset (obj,0,sizeof(obj));

  // note: can't share tridata if intending to trimesh-trimesh collide
  TriData1 = dGeomTriMeshDataCreate();
  dGeomTriMeshDataBuildSingle(TriData1, &Vertices[0], 3 * sizeof(float), VertexCount, (dTriIndex*)&Indices[0], IndexCount, 3 * sizeof(dTriIndex));
  TriData2 = dGeomTriMeshDataCreate();
  dGeomTriMeshDataBuildSingle(TriData2, &Vertices[0], 3 * sizeof(float), VertexCount, (dTriIndex*)&Indices[0], IndexCount, 3 * sizeof(dTriIndex));
  
  TriMesh1 = dCreateTriMesh(space, TriData1, 0, 0, 0);
  TriMesh2 = dCreateTriMesh(space, TriData2, 0, 0, 0);
  dGeomSetData(TriMesh1, TriData1);
  dGeomSetData(TriMesh2, TriData2);
  
  {dGeomSetPosition(TriMesh1, 0, 0, 0.9);
  dMatrix3 Rotation;
  dRFromAxisAndAngle(Rotation, 1, 0, 0, M_PI / 2);
  dGeomSetRotation(TriMesh1, Rotation);}

  {dGeomSetPosition(TriMesh2, 1, 0, 0.9);
  dMatrix3 Rotation;
  dRFromAxisAndAngle(Rotation, 1, 0, 0, M_PI / 2);
  dGeomSetRotation(TriMesh2, Rotation);}
  
  dThreadingImplementationID threading = dThreadingAllocateMultiThreadedImplementation();
  dThreadingThreadPoolID pool = dThreadingAllocateThreadPool(4, 0, dAllocateFlagBasicData, NULL);
  dThreadingThreadPoolServeMultiThreadedImplementation(pool, threading);
  // dWorldSetStepIslandsProcessingMaxThreadCount(world, 1);
  dWorldSetStepThreadingImplementation(world, dThreadingImplementationGetFunctions(threading), threading);

  // run simulation
  dsSimulationLoop (argc,argv,352,288,&fn);

  dThreadingImplementationShutdownProcessing(threading);
  dThreadingFreeThreadPool(pool);
  dWorldSetStepThreadingImplementation(world, NULL, NULL);
  dThreadingFreeImplementation(threading);

  dJointGroupDestroy (contactgroup);
  dSpaceDestroy (space);
  dWorldDestroy (world);
  dCloseODE();
  return 0;
}
