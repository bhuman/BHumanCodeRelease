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
  This program demonstrates how the PU joint works.
  A PU joint is a combination of a Universal joint and a Slider joint.
  It is a universal joint with a slider between the anchor point and
  body 1.


  The upper yellow body is fixed to the world
  The lower yellow body is attached to the upper body by a PU joint
  The green object is one aprt of the slider.
  The purple object is the second part of the slider.
  The red object represent the axis1 of the universal part.
  The blue object represent the axis2 of the universal part.
  The gray object represent the anchor2 of the PU joint.
*/


#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <iostream>
#include <math.h>
#include "texturepath.h"

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif
// select correct drawing functions
#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#endif

enum IDX_CYL_DIM
{
  RADIUS,
  LENGTH,
  NUM_CYL_DIM
};


const dVector3 boxDim = {1,1,1};
const dVector3 extDim = {0.2,0.2,1.2};
const dVector3 ancDim = {0.2,0.2,0.5};
const dReal    axDim[NUM_CYL_DIM] = {0.1,1.0};


int type = dJointTypePU;


const dReal VEL_INC = 0.01; // Velocity increment

// physics parameters
const dReal PI = 3.14159265358979323846264338327950288419716939937510;


const dReal INT_EXT_RATIO = 0.8;

#define X 0
#define Y 1
#define Z 2

enum INDEX
{
  W = 0,
  D,
  EXT,
  INT,
  AXIS1,
  AXIS2,
  ANCHOR,
  GROUND,
  NUM_PARTS,
  ALL = NUM_PARTS,
  // INDEX for catBits
  JOINT,
  LAST_INDEX_CNT
};

const int catBits[LAST_INDEX_CNT] =
  {
    0x0001, ///< W Cylinder category
    0x0002, ///< D Cylinder category
    0x0004, ///< EXT sliderr category
    0x0008, ///< INT slider category
    0x0010, ///< AXIS1 universal category
    0x0020, ///< AXIS2 universal category
    0x0040, ///< ANCHOR category
    0x0080, ///< Ground category
    ~0L,    ///< All categories
    0x0004 | 0x0008 | 0x0010 | 0x0020 ///< JOINT category
  };

#define Mass1 10
#define Mass2 8


//camera view
static float xyz[3] = {6.0f,0.0f,6.0000f};
static float hpr[3] = {-180.000f,-25.5000f,0.0000f};


//world,space,body & geom
static dWorldID         world;
static dSpaceID         space;
static dJointGroupID    contactgroup;
static dBodyID          body[NUM_PARTS];
static dGeomID          geom[NUM_PARTS];

static dJoint *joint;



const dReal BOX_SIDES[3]  = {1.0,1.0,1.0};
const dReal OBS_SIDES[3]  = {0.4,0.4,0.4};
const dReal RECT_SIDES[3] = {0.3, 0.1, 0.2};


//collision detection
static void nearCallback (void *, dGeomID o1, dGeomID o2)
{
  int i,n;

  const int N = 10;
  dContact contact[N];
  n = dCollide (o1,o2,N,&contact[0].geom,sizeof (dContact) );
  if (n > 0) {
    for  (i=0; i<n; i++) {
      contact[i].surface.mode = (dContactSlip1 | dContactSlip2 |
                                 dContactSoftERP | dContactSoftCFM |
                                 dContactApprox1);
      contact[i].surface.mu = 0.1;
      contact[i].surface.slip1 = 0.02;
      contact[i].surface.slip2 = 0.02;
      contact[i].surface.soft_erp = 0.1;
      contact[i].surface.soft_cfm = 0.0001;
      dJointID c = dJointCreateContact (world,contactgroup,&contact[i]);
      dJointAttach (c,dGeomGetBody (contact[i].geom.g1),dGeomGetBody (contact[i].geom.g2) );
    }
  }
}

static void printKeyBoardShortCut()
{
  printf ("Press 'h' for this help.\n");
  printf ("Press 'q' to add force on BLUE body along positive x direction.\n");
  printf ("Press 'w' to add force on BLUE body along negative x direction.\n");

  printf ("Press 'a' to add force on BLUE body along positive y direction.\n");
  printf ("Press 's' to add force on BLUE body along negative y direction.\n");

  printf ("Press 'z' to add force on BLUE body along positive z direction.\n");
  printf ("Press 'x' to add force on BLUE body along negative z direction.\n");

  printf ("Press 'e' to add torque on BLUE body around positive x direction \n");
  printf ("Press 'r' to add torque on BLUE body around negative x direction \n");

  printf ("Press 'd' to add torque on BLUE body around positive y direction \n");
  printf ("Press 'f' to add torque on BLUE body around negative y direction \n");

  printf ("Press 'c' to add torque on BLUE body around positive z direction \n");
  printf ("Press 'v' to add torque on BLUE body around negative z direction \n");

  printf ("Press '.' to increase joint velocity along the prismatic direction.\n");
  printf ("Press ',' to decrease joint velocity along the prismatic direction.\n");

  printf ("Press 'l' Toggle ON/OFF the limits on all the axis\n");
  printf ("Press 'g' Toggle ON/OFF the gravity\n");


  printf ("Press 'p' to print the position, angle and rates of the joint.\n");
}


// start simulation - set viewpoint
static void start()
{
  dAllocateODEDataForThread(dAllocateMaskAll);

  dsSetViewpoint (xyz,hpr);
  printf ("This program demonstrates how the PU joint works.\n");
  printf ("A PU joint is a combination of a Universal joint and a Slider joint.\n");
  printf ("It is a universal joint with a slider between the anchor point and \n");
  printf ("body 1.\n\n");
  printf ("The upper yellow body is fixed to the world\n");
  printf ("The lower yellow body is attached to the upper body by a PU joint\n");
  printf ("The green object is one aprt of the slider.\n");
  printf ("The purple object is the second part of the slider.\n");
  printf ("The red object represent the axis1 of the universal part. \n");
  printf ("The blue object represent the axis2 of the universal part. \n");
  printf ("The gray object represent the anchor2 of the PU joint. \n");
  printKeyBoardShortCut();
}

// function to update camera position at each step.
void update()
{
//   static FILE *file = fopen("x:/sim/src/libode/tstsrcSF/export.dat", "w");

//   static int cnt = 0;
//   char str[24];
//   sprintf(str, "%06d",cnt++);

//   dWorldExportDIF(world, file, str);
}


// called when a key pressed
static void command (int cmd)
{
  switch (cmd) {
case 'h' : case 'H' : case '?' :
      printKeyBoardShortCut();
      break;

      // Force
  case 'q' : case 'Q' :
      dBodyAddForce(body[D],40,0,0);
      break;
  case 'w' : case 'W' :
      dBodyAddForce(body[D],-40,0,0);
      break;

  case 'a' : case 'A' :
      dBodyAddForce(body[D],0,40,0);
      break;
  case 's' : case 'S' :
      dBodyAddForce(body[D],0,-40,0);
      break;

  case 'z' : case 'Z' :
      dBodyAddForce(body[D],0,0,40);
      break;
  case 'x' : case 'X' :
      dBodyAddForce(body[D],0,0,-40);
      break;

      // Torque
  case 'e': case 'E':
      dBodyAddTorque(body[D],0.1,0,0);
      break;
  case 'r': case 'R':
      dBodyAddTorque(body[D],-0.1,0,0);
      break;

  case 'd': case 'D':
      dBodyAddTorque(body[D],0, 0.1,0);
      break;
  case 'f': case 'F':
      dBodyAddTorque(body[D],0,-0.1,0);
      break;

  case 'c': case 'C':
      dBodyAddTorque(body[D],0,0,0.1);
      break;
  case 'v': case 'V':
      dBodyAddTorque(body[D],0,0,0.1);
      break;

      // Velocity of joint
  case ',': case '<' : {
      dReal vel = joint->getParam (dParamVel3) - VEL_INC;
      joint->setParam (dParamVel3, vel);
	  joint->setParam (dParamFMax3, 2);
      std::cout<<"Velocity = "<<vel<<"  FMax = 2"<<'\n';
    }
    break;

  case '.': case '>' : {
      dReal vel = joint->getParam (dParamVel3) + VEL_INC;
      joint->setParam (dParamVel3, vel);
	  joint->setParam (dParamFMax3, 2);
      std::cout<<"Velocity = "<<vel<<"  FMax = 2"<<'\n';
    }
    break;

  case 'l': case 'L' : {
      dReal aLimit, lLimit, fmax;
      if (  joint->getParam (dParamFMax) ) {
        aLimit = dInfinity;
        lLimit = dInfinity;
        fmax = 0;
      }
      else {
        aLimit = 0.25*PI;
        lLimit = 0.5*axDim[LENGTH];
        fmax = 0.02;
      }

      joint->setParam (dParamFMax1, fmax);
      joint->setParam (dParamFMax2, fmax);
      joint->setParam (dParamFMax3, fmax);

      switch (joint->getType() ) {
        case dJointTypePR : {
          dPRJoint *pr = reinterpret_cast<dPRJoint *> (joint);
          pr->setParam (dParamLoStop, -lLimit);
          pr->setParam (dParamHiStop, -lLimit);
          pr->setParam (dParamLoStop2, aLimit);
          pr->setParam (dParamHiStop2, -aLimit);
        }
        break;
        case dJointTypePU : {
          dPUJoint *pu = reinterpret_cast<dPUJoint *> (joint);
          pu->setParam (dParamLoStop1, -aLimit);
          pu->setParam (dParamHiStop1, aLimit);
          pu->setParam (dParamLoStop2, -aLimit);
          pu->setParam (dParamHiStop2, aLimit);
          pu->setParam (dParamLoStop3, -lLimit);
          pu->setParam (dParamHiStop3, lLimit);
        }
        break;
        default: {} // keep the compiler happy
      }
    }

    break;

  case 'g': case 'G' : {
      dVector3 g;
      dWorldGetGravity(world, g);
      if (  g[2]< -0.1 )
        dWorldSetGravity(world, 0, 0, 0);
      else
        dWorldSetGravity(world, 0, 0, -0.5);

    }

case 'p' :case 'P' : {
      switch (joint->getType() ) {
        case dJointTypeSlider : {
          dSliderJoint *sj = reinterpret_cast<dSliderJoint *> (joint);
          std::cout<<"Position ="<<sj->getPosition() <<"\n";
        }
        break;
        case dJointTypePU : {
          dPUJoint *pu = reinterpret_cast<dPUJoint *> (joint);
          std::cout<<"Position ="<<pu->getPosition() <<"\n";
          std::cout<<"Position Rate="<<pu->getPositionRate() <<"\n";
          std::cout<<"Angle1 ="<<pu->getAngle1() <<"\n";
          std::cout<<"Angle1 Rate="<<pu->getAngle1Rate() <<"\n";
          std::cout<<"Angle2 ="<<pu->getAngle2() <<"\n";
          std::cout<<"Angle2 Rate="<<pu->getAngle2Rate() <<"\n";
        }
        break;
        default: {} // keep the compiler happy
      }
    }
    break;
  }
}

static void drawBox (dGeomID id, int R, int G, int B)
{
  if (!id)
    return;

  const dReal *pos = dGeomGetPosition (id);
  const dReal *rot = dGeomGetRotation (id);
  dsSetColor (R,G,B);

  dVector3 l;
  dGeomBoxGetLengths (id, l);
  dsDrawBox (pos, rot, l);
}


// simulation loop
static void simLoop (int pause)
{
  static bool todo = false;
  if ( todo ) { // DEBUG
    static int cnt = 0;
    ++cnt;

    if (cnt == 5)
      command ( 'q' );
    if (cnt == 10)
      dsStop();
  }




  if (!pause) {
    double simstep = 0.01; // 10ms simulation steps
    double dt = dsElapsedTime();

    int nrofsteps = (int) ceilf (dt/simstep);
    if (!nrofsteps)
      nrofsteps = 1;

    for (int i=0; i<nrofsteps && !pause; i++) {
      dSpaceCollide (space,0,&nearCallback);
      dWorldStep (world, simstep);

      dJointGroupEmpty (contactgroup);
    }

    update();


    dReal radius, length;

    dsSetTexture (DS_WOOD);

    drawBox (geom[W], 1,1,0);


    drawBox (geom[EXT], 0,1,0);

    dVector3 anchorPos;



    dReal ang1 = 0;
    dReal ang2 = 0;
    dVector3 axisP, axisR1, axisR2;

    if ( dJointTypePU == type ) {
      dPUJoint *pu = dynamic_cast<dPUJoint *> (joint);
      ang1 = pu->getAngle1();
      ang2 = pu->getAngle2();
      pu->getAxis1 (axisR1);
      pu->getAxis2 (axisR2);
      pu->getAxisP (axisP);

      dJointGetPUAnchor (pu->id(), anchorPos);
    }
    else if ( dJointTypePR == type ) {
      dPRJoint *pr = dynamic_cast<dPRJoint *> (joint);
      pr->getAxis1 (axisP);
      pr->getAxis2 (axisR1);

      dJointGetPRAnchor (pr->id(), anchorPos);
    }


    // Draw the axisR
    if ( geom[INT] ) {
      dsSetColor (1,0,1);
      dVector3 l;
      dGeomBoxGetLengths (geom[INT], l);

      const dReal *rotBox = dGeomGetRotation (geom[W]);

      dVector3 pos;
      for (int i=0; i<3; ++i)
        pos[i] = anchorPos[i] - 0.5*extDim[Z]*axisP[i];
      dsDrawBox (pos, rotBox, l);
    }

    dsSetTexture (DS_CHECKERED);
    if ( geom[AXIS1] ) {
      dQuaternion q, qAng;
      dQFromAxisAndAngle (qAng,axisR1[X], axisR1[Y], axisR1[Z], ang1);
      dGeomGetQuaternion (geom[AXIS1], q);

      dQuaternion qq;
      dQMultiply1 (qq, qAng, q);
      dMatrix3 R;
      dQtoR (qq,R);


      dGeomCylinderGetParams (geom[AXIS1], &radius, &length);
      dsSetColor (1,0,0);
      dsDrawCylinder (anchorPos, R, length, radius);
    }

    if ( dJointTypePU == type && geom[AXIS2] ) {
      //dPUJoint *pu = dynamic_cast<dPUJoint *> (joint);

      dQuaternion q, qAng, qq, qq1;
      dGeomGetQuaternion (geom[AXIS2], q);

      dQFromAxisAndAngle (qAng, 0, 1, 0, ang2);
      dQMultiply1 (qq, qAng, q);


      dQFromAxisAndAngle (qAng,axisR1[X], axisR1[Y], axisR1[Z], ang1);

      dQMultiply1 (qq1, qAng, qq);


      dMatrix3 R;
      dQtoR (qq1,R);


      dGeomCylinderGetParams (geom[AXIS2], &radius, &length);
      dsSetColor (0,0,1);
      dsDrawCylinder (anchorPos, R, length, radius);
    }

    dsSetTexture (DS_WOOD);

    // Draw the anchor
    if ( geom[ANCHOR] ) {
      dsSetColor (1,1,1);
      dVector3 l;
      dGeomBoxGetLengths (geom[ANCHOR], l);

      const dReal *rotBox = dGeomGetRotation (geom[D]);
      const dReal *posBox = dGeomGetPosition (geom[D]);

      dVector3 e;
      for (int i=0; i<3; ++i)
        e[i] = posBox[i] - anchorPos[i];
      dNormalize3 (e);

      dVector3 pos;
      for (int i=0; i<3; ++i)
        pos[i] = anchorPos[i] + 0.5 * l[Z]*e[i];
      dsDrawBox (pos, rotBox, l);
    }

    drawBox (geom[D], 1,1,0);
  }
}


void Help (char **argv)
{
  printf ("%s ", argv[0]);
  printf (" -h | --help   : print this help\n");
  printf (" -p | --PRJoint : Use a PR joint instead of PU joint\n");
  printf (" -t | --texture-path path  : Path to the texture.\n");
  printf ("                             Default = %s\n", DRAWSTUFF_TEXTURE_PATH);
  printf ("--------------------------------------------------\n");
  printf ("Hit any key to continue:");
  getchar();

  exit (0);
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

  if (argc >= 2 ) {
    for (int i=1; i < argc; ++i) {
      if (  0 == strcmp ("-h", argv[i]) || 0 == strcmp ("--help", argv[i]) )
        Help (argv);

      if (  0 == strcmp ("-p", argv[i]) || 0 == strcmp ("--PRJoint", argv[i]) )
        type = dJointTypePR;

      if (0 == strcmp ("-t", argv[i]) || 0 == strcmp ("--texture-path", argv[i]) ) {
        int j = i+1;
        if ( j+1 > argc      ||  // Check if we have enough arguments
             argv[j] == '\0' ||  // We should have a path here
             argv[j][0] == '-' ) // We should have a path not a command line
          Help (argv);
        else
          fn.path_to_textures = argv[++i]; // Increase i since we use this argument
      }
    }
  }

  dInitODE2(0);

  world = dWorldCreate();
  dWorldSetERP (world, 0.8);

  space = dSimpleSpaceCreate (0);
  contactgroup = dJointGroupCreate (0);
  geom[GROUND] = dCreatePlane (space, 0,0,1,0);
  dGeomSetCategoryBits (geom[GROUND], catBits[GROUND]);
  dGeomSetCollideBits (geom[GROUND], catBits[ALL]);

  dMass m;

  // Create the body attached to the World
  body[W] = dBodyCreate (world);
  // Main axis of cylinder is along X=1
  m.setBox (1, boxDim[X], boxDim[Y], boxDim[Z]);
  m.adjust (Mass1);
  geom[W] = dCreateBox (space, boxDim[X], boxDim[Y], boxDim[Z]);
  dGeomSetBody (geom[W], body[W]);
  dGeomSetCategoryBits (geom[W], catBits[W]);
  dGeomSetCollideBits (geom[W], catBits[ALL] & (~catBits[W]) & (~catBits[JOINT]) );
  dBodySetMass (body[W], &m);





  // Create the dandling body
  body[D] = dBodyCreate (world);
  // Main axis of capsule is along X=1
  m.setBox (1, boxDim[X], boxDim[Y], boxDim[Z]);
  m.adjust (Mass1);
  geom[D] = dCreateBox (space, boxDim[X], boxDim[Y], boxDim[Z]);
  dGeomSetBody (geom[D], body[D]);
  dGeomSetCategoryBits (geom[D], catBits[D]);
  dGeomSetCollideBits (geom[D], catBits[ALL] & (~catBits[D]) & (~catBits[JOINT]) );
  dBodySetMass (body[D], &m);


  // Create the external part of the slider joint
  geom[EXT] = dCreateBox (0, extDim[X], extDim[Y], extDim[Z]);
  dGeomSetCategoryBits (geom[EXT], catBits[EXT]);
  dGeomSetCollideBits (geom[EXT],
                       catBits[ALL] & (~catBits[JOINT]) & (~catBits[W]) & (~catBits[D]) );

  // Create the internal part of the slider joint
  geom[INT] = dCreateBox (0, INT_EXT_RATIO*extDim[X],
                          INT_EXT_RATIO*extDim[Y],
                          INT_EXT_RATIO*extDim[Z]);
  dGeomSetCategoryBits (geom[INT], catBits[INT]);
  dGeomSetCollideBits (geom[INT],
                       catBits[ALL] & (~catBits[JOINT]) & (~catBits[W]) & (~catBits[D]) );


  dMatrix3 R;
  // Create the first axis of the universal joi9nt
  //Rotation of 90deg around y
  geom[AXIS1] = dCreateCylinder(0, axDim[RADIUS], axDim[LENGTH]);
  dRFromAxisAndAngle(R, 0,1,0, 0.5*PI);
  dGeomSetRotation(geom[AXIS1], R);
  dGeomSetCategoryBits(geom[AXIS1], catBits[AXIS1]);
  dGeomSetCollideBits(geom[AXIS1],
                      catBits[ALL]  & ~catBits[JOINT] & ~catBits[W] & ~catBits[D]);


  // Create the second axis of the universal joint
  geom[AXIS2] = dCreateCylinder(0, axDim[RADIUS], axDim[LENGTH]);
  //Rotation of 90deg around y
  dRFromAxisAndAngle(R, 1,0,0, 0.5*PI);
  dGeomSetRotation(geom[AXIS2], R);
  dGeomSetCategoryBits(geom[AXIS2], catBits[AXIS2]);
  dGeomSetCollideBits(geom[AXIS2],
                      catBits[ALL]  & ~catBits[JOINT] & ~catBits[W] & ~catBits[D]);

  // Create the anchor
  geom[ANCHOR] = dCreateBox (0, ancDim[X], ancDim[Y], ancDim[Z]);
  dGeomSetCategoryBits(geom[ANCHOR], catBits[ANCHOR]);
  dGeomSetCollideBits(geom[ANCHOR],
                      catBits[ALL] & (~catBits[JOINT]) & (~catBits[W]) & (~catBits[D]) );



  if (body[W]) {
    dBodySetPosition(body[W], 0, 0, 5);
  }


  if (geom[EXT]) {
    dGeomSetPosition(geom[EXT], 0,0,3.8);
  }
  if (geom[INT]) {
    dGeomSetPosition(geom[INT], 0,0,2.6);
  }
  if (geom[AXIS1]) {
    dGeomSetPosition(geom[AXIS1], 0,0,2.5);
  }
  if (geom[AXIS2]) {
    dGeomSetPosition(geom[AXIS2], 0,0,2.5);
  }

  if (geom[ANCHOR]) {
    dGeomSetPosition(geom[ANCHOR], 0,0,2.25);
  }

  if (body[D]) {
    dBodySetPosition(body[D], 0,0,1.5);
  }



  // Attache the upper box to the world
  dJointID fixed = dJointCreateFixed (world,0);
  dJointAttach (fixed , NULL, body[W]);
  dJointSetFixed (fixed );

  if (type == dJointTypePR) {
    dPRJoint *pr = new dPRJoint (world, 0);
    pr->attach (body[W], body[D]);
    pr->setAxis1 (0, 0, -1);
    pr->setAxis2 (1, 0, 0);
    joint = pr;

    dJointSetPRAnchor (pr->id(), 0, 0, 2.5);
  }
  else {
    dPUJoint *pu = new dPUJoint (world, 0);
    pu->attach (body[W], body[D]);
    pu->setAxis1 (1, 0, 0);
    pu->setAxis2 (0, 1, 0);
    pu->setAxisP (0, 0, -1);
    joint = pu;

    dJointSetPUAnchor (pu->id(), 0, 0, 2.5);
  }


  // run simulation
  dsSimulationLoop (argc,argv,400,300,&fn);

  delete joint;
  dJointGroupDestroy (contactgroup);
  dSpaceDestroy (space);
  dWorldDestroy (world);
  dCloseODE();
  return 0;
}

