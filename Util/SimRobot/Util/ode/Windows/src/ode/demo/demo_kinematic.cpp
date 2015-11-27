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

#include <iostream>
#include <set>
#include <algorithm>
#include <functional>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawCylinder dsDrawCylinderD
#endif


using namespace std;

dWorld *world;
dSpace *space;
dPlane *ground;
dBody *kbody;
dBox *kbox;
dJointGroup joints;
dCylinder *kpole;
dBody *matraca;
dBox *matraca_geom;
dHingeJoint *hinge;

struct Box {
    dBody body;
    dBox geom;
    Box() :
        body(*world),
        geom(*space, 0.2, 0.2, 0.2)
    {
        dMass mass;
        mass.setBox(10, 0.2, 0.2, 0.2);
        body.setMass(mass);
        geom.setData(this);
        geom.setBody(body);
    }
    void draw() const
    {
        dVector3 lengths;
        geom.getLengths(lengths);
        dsSetTexture(DS_WOOD);
        dsSetColor(0,1,0);
        dsDrawBox(geom.getPosition(), geom.getRotation(), lengths);
    }
};

set<Box*> boxes;
set<Box*> to_remove;

void dropBox()
{
    Box *box = new Box();
    
    dReal px = (rand() / float(RAND_MAX)) * 2 - 1;
    dReal py = (rand() / float(RAND_MAX)) * 2 - 1;
    dReal pz = 2.5;
    box->body.setPosition(px, py, pz);
    
    boxes.insert(box);
}

void queueRemoval(dGeomID g)
{
    Box *b = (Box*)dGeomGetData(g);
    to_remove.insert(b);
}

void removeQueued()
{
    while (!to_remove.empty()) {
        Box *b = *to_remove.begin();
        to_remove.erase(b);
        boxes.erase(b);
        delete b;
    }
}


void nearCallback(void *, dGeomID g1, dGeomID g2)
{
    if (g1 == ground->id()) {
        queueRemoval(g2);
        return;
    }
    if (g2 == ground->id()) {
        queueRemoval(g1);
        return;
    }

    dBodyID b1 = dGeomGetBody(g1);
    dBodyID b2 = dGeomGetBody(g2);
    
    if (b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact))
        return;

    const int MAX_CONTACTS = 10;
    dContact contact[MAX_CONTACTS];
    int n = dCollide(g1, g2, MAX_CONTACTS, &contact[0].geom, sizeof(dContact));
    for (int i=0; i<n; ++i) {
        contact[i].surface.mode = 0;
        contact[i].surface.mu = 1;
        dJointID j = dJointCreateContact (*world, joints.id(), contact+i);
        dJointAttach(j, b1, b2);
    }
}


void
simLoop(int pause)
{
    if (!pause) {
        const dReal timestep = 0.04;

        // this does a hard-coded circular motion animation
        static float t=0;
        t += timestep/4;
        if (t > 2*M_PI)
            t = 0;
        dVector3 next_pos = { dCos(t), dSin(t), REAL(0.5)};
        dVector3 vel;
        // vel = (next_pos - cur_pos) / timestep
        dSubtractVectors3(vel, next_pos, kbody->getPosition());
        dScaleVector3(vel, 1/timestep);
        kbody->setLinearVel(vel);
        // end of hard-coded animation
        
        space->collide(0, nearCallback);
        removeQueued();
        
        world->quickStep(timestep);
        joints.clear();
    }

    dVector3 lengths;

    // the moving platform
    kbox->getLengths(lengths);
    dsSetTexture(DS_WOOD);
    dsSetColor(.3, .3, 1);
    dsDrawBox(kbox->getPosition(), kbox->getRotation(), lengths);
    dReal length, radius;
    kpole->getParams(&radius, &length);
    dsSetTexture(DS_CHECKERED);
    dsSetColor(1, 1, 0);
    dsDrawCylinder(kpole->getPosition(), kpole->getRotation(), length, radius);
    
    // the matraca
    matraca_geom->getLengths(lengths);
    dsSetColor(1,0,0);
    dsSetTexture(DS_WOOD);
    dsDrawBox(matraca_geom->getPosition(), matraca_geom->getRotation(), lengths);

    // and the boxes
    for_each(boxes.begin(), boxes.end(), mem_fun(&Box::draw));
}

void command(int c)
{
    switch (c) {
        case ' ':
            dropBox();
            break;
    }
}

int main(int argc, char **argv)
{
    dInitODE();

    // setup pointers to drawstuff callback functions
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start = 0;
    fn.step = &simLoop;
    fn.command = &command;
    fn.stop = 0;
    fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;
    
    cout << endl << "*** Press SPACE to drop boxes **" << endl;
    
    space = new dSimpleSpace();
    ground = new dPlane(*space, 0, 0, 1, 0);
    
    world = new dWorld;
    world->setGravity(0, 0, -.5);
    
    kbody = new dBody(*world);
    kbody->setKinematic();
    const dReal kx = 1, ky = 0, kz = .5;
    kbody->setPosition(kx, ky, kz);
    kbox = new dBox(*space, 3, 3, .5);
    kbox->setBody(*kbody);
    kpole = new dCylinder(*space, .125, 1.5);
    kpole->setBody(*kbody);
    dGeomSetOffsetPosition(kpole->id(), 0, 0, 0.8);
    
    matraca = new dBody(*world);
    matraca->setPosition(kx+0, ky+1, kz+1);
    matraca_geom = new dBox(*space, 0.5, 2, 0.75);
    matraca_geom->setBody(*matraca);
    dMass mass;
    mass.setBox(1, 0.5, 2, 0.75);
    matraca->setMass(mass);
    
    hinge = new dHingeJoint(*world);
    hinge->attach(*kbody, *matraca);
    hinge->setAnchor(kx, ky, kz+1);
    hinge->setAxis(0, 0, 1);
    
    dsSimulationLoop (argc, argv, 640, 480, &fn);
    
    dCloseODE();
}
