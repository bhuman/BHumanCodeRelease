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

#include <vector>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#endif

static int levels = 5;
static int ncards = 0;

static dSpaceID space;
static dWorldID world;
static dJointGroupID contactgroup;

struct Card {
    dBodyID body;
    dGeomID geom;
    static const dReal sides[3];

    Card()
    {
        body = dBodyCreate(world);
        geom = dCreateBox(space, sides[0], sides[1], sides[2]);
        dGeomSetBody(geom, body);
        dGeomSetData(geom, this);
        dMass mass;
        mass.setBox(1, sides[0], sides[1], sides[2]);
        dBodySetMass(body, &mass);
    }

    ~Card()
    {
        dBodyDestroy(body);
        dGeomDestroy(geom);
    }
    
    void draw() const
    {
        dsDrawBox(dBodyGetPosition(body),
                  dBodyGetRotation(body), sides);
    }
};
static const dReal cwidth=.5, cthikness=.02, clength=1;
const dReal Card::sides[3] = { cwidth, cthikness, clength };


std::vector<Card*> cards;

int getncards(int levels)
{
    return (3*levels*levels + levels) / 2;
}

void place_cards()
{
    ncards = getncards(levels);
    // destroy removed cards (if any)
    int oldcards = cards.size();
    for (int i=ncards; i<oldcards; ++i)
        delete cards[i];
    cards.resize(ncards);
    // construct new cards (if any)
    for (int i=oldcards; i<ncards; ++i)
        cards[i] = new Card;
    
    // for each level
    int c = 0;
    dMatrix3 right, left, hrot;
    dReal angle = 20*M_PI/180.;
    dRFromAxisAndAngle(right, 1, 0, 0, -angle);
    dRFromAxisAndAngle(left, 1, 0, 0, angle);

    dRFromAxisAndAngle(hrot, 1, 0, 0, 91*M_PI/180.);
    
    dReal eps = 0.05;
    dReal vstep = cos(angle)*clength + eps;
    dReal hstep = sin(angle)*clength + eps;
    
    for (int lvl=0; lvl<levels; ++lvl) {
        // there are 3*(levels-lvl)-1 cards in each level, except last
        int n = (levels-lvl);
        dReal height = (lvl)*vstep + vstep/2;
        // inclined cards
        for (int i=0; i<2*n; ++i, ++c) {
            dBodySetPosition(cards[c]->body, 
                    0,
                    -n*hstep + hstep*i,
                    height
                    );
            if (i%2)
                dBodySetRotation(cards[c]->body, left);
            else
                dBodySetRotation(cards[c]->body, right);
        }
        
        if (n==1) // top of the house
            break;
        
        // horizontal cards
        for (int i=0; i<n-1; ++i, ++c) {
            dBodySetPosition(cards[c]->body,
                    0,
                    -(n-1 - (clength-hstep)/2)*hstep + 2*hstep*i,
                    height + vstep/2);
            dBodySetRotation(cards[c]->body, hrot);
        }
    }
    
}


void start()
{
    puts("Controls:");
    puts("   SPACE - reposition cards");
    puts("   -     - one less level");
    puts("   =     - one more level");
}

static void nearCallback (void *, dGeomID o1, dGeomID o2)
{
    // exit without doing anything if the two bodies are connected by a joint
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);

    const int MAX_CONTACTS = 8;
    dContact contact[MAX_CONTACTS];
    
    int numc = dCollide (o1, o2, MAX_CONTACTS,
                        &contact[0].geom,
                        sizeof(dContact));
    
    for (int i=0; i<numc; i++) {
        contact[i].surface.mode = dContactApprox1;
        contact[i].surface.mu = 5;
        dJointID c = dJointCreateContact (world, contactgroup, contact+i);
        dJointAttach (c, b1, b2);
    }
}


void simLoop(int pause)
{
    if (!pause) {
        dSpaceCollide (space, 0, &nearCallback);
        dWorldQuickStep(world, 0.01);
        dJointGroupEmpty(contactgroup);
    }
    
    dsSetColor (1,1,0);
    for (int i=0; i<ncards; ++i) {
        dsSetColor (1, dReal(i)/ncards, 0);
        cards[i]->draw();
    }
    
}

void command(int c)
{
    switch (c) {
        case '=':
            levels++;
            place_cards();
            break;
        case '-':
            levels--;
            if (levels <= 0)
                levels++;
            place_cards();
            break;
        case ' ':
            place_cards();
            break;
    }
}

int main(int argc, char **argv)
{
    dInitODE();
    
    // setup pointers to drawstuff callback functions
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start = &start;
    fn.step = &simLoop;
    fn.command = &command;
    fn.stop = 0;
    fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;
    
    
    world = dWorldCreate();
    dWorldSetGravity(world, 0, 0, -0.5);
    dWorldSetQuickStepNumIterations(world, 50); // <-- increase for more stability
    
    space = dSimpleSpaceCreate(0);
    contactgroup = dJointGroupCreate(0);
    dGeomID ground = dCreatePlane(space, 0, 0, 1, 0);
    
    place_cards();
    
    // run simulation
    dsSimulationLoop (argc, argv, 640, 480, &fn);
    
    levels = 0;
    place_cards();
    
    dJointGroupDestroy(contactgroup);
    dWorldDestroy(world);
    dGeomDestroy(ground);
    dSpaceDestroy(space);
    
    dCloseODE();
}
