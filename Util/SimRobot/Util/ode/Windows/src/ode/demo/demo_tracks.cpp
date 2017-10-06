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

//#include <iostream>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"

#ifdef dDOUBLE
#define dsDrawSphere dsDrawSphereD
#define dsDrawBox dsDrawBoxD
#define dsDrawTriangle dsDrawTriangleD
#define dsDrawLine dsDrawLineD
#endif



const dReal ball_radius = 0.4;
const dReal balls_sep = 2; // separation between the balls

/* Choose one test case
 */
#define TEST_CASE 0

#if TEST_CASE == 0
const dReal track_len = 10;
const dReal track_height = 1;
const dReal track_width = 0.1;
const dReal track_gauge = 1;
const dReal track_elevation = 2;
const dReal track_angle = 80 * M_PI/180.;
const dReal track_incl = 10 * M_PI/180.;
#elif TEST_CASE == 1
const dReal track_len = 10;
const dReal track_height = 1;
const dReal track_width = 0.1;
const dReal track_gauge = 1.9*ball_radius;
const dReal track_elevation = 2;
const dReal track_angle = 0 * M_PI/180.;
const dReal track_incl = 10 * M_PI/180.;
#elif TEST_CASE == 2
const dReal track_len = 10;
const dReal track_height = 1;
const dReal track_width = 0.1;
const dReal track_gauge = 1.9*ball_radius;
const dReal track_elevation = 2;
const dReal track_angle = 15 * M_PI/180.;
const dReal track_incl = 10 * M_PI/180.;
#elif TEST_CASE == 3
const dReal track_len = 10;
const dReal track_height = .7;
const dReal track_width = 0.1;
const dReal track_gauge = track_height*1.1;
const dReal track_elevation = 2;
const dReal track_angle = 90 * M_PI/180.;
const dReal track_incl = 10 * M_PI/180.;
#else
#error "TEST_CAST to a valid value!"
#endif



dWorldID world;
dSpaceID space;
dJointGroupID contact_group;
dGeomID ground;
dGeomID ball1_geom, ball2_geom;
dTriMeshDataID mesh_data;
dGeomID mesh_geom;

dBodyID ball1_body, ball2_body;

const unsigned n_box_verts = 8;
dVector3 box_verts[n_box_verts] = {
    {-track_len/2, -track_width/2,  track_height/2}, // 0
    { track_len/2, -track_width/2,  track_height/2}, // 1
    { track_len/2,  track_width/2,  track_height/2}, // 2
    {-track_len/2,  track_width/2,  track_height/2}, // 3
    { track_len/2, -track_width/2, -track_height/2}, // 4
    {-track_len/2, -track_width/2, -track_height/2}, // 5
    {-track_len/2,  track_width/2, -track_height/2}, // 6
    { track_len/2,  track_width/2, -track_height/2}  // 7
};

const unsigned n_box_faces = 12;
dTriIndex box_faces[n_box_faces * 3] = {
    0, 1, 2,
    0, 2, 3,
    1, 4, 7,
    1, 7, 2,
    4, 5, 6,
    4, 6, 7,
    5, 0, 3,
    5, 3, 6,
    3, 2, 7,
    3, 7, 6,
    0, 5, 4,
    0, 4, 1
};


const unsigned n_track_verts = n_box_verts * 2;
const unsigned n_track_faces = n_box_faces * 2;

dVector3 track_verts[n_track_verts];
dTriIndex track_faces[n_track_faces * 3];



void resetBall(dBodyID b, unsigned idx)
{
    dBodySetPosition(b,
                     0.5*track_len*cos(track_incl) // Z
                     - 0.5*track_height*sin(track_incl)
                     - ball_radius, // X
                     balls_sep*idx, // Y
                     track_elevation + ball_radius// Z
                     + 0.5*track_len*sin(track_incl)
                     + 0.5*track_height*cos(track_incl));
    dMatrix3 r = {1, 0, 0, 0,
                  0, 1, 0, 0,
                  0, 0, 1, 0};
    dBodySetRotation(b, r);
    dBodySetLinearVel(b, 0, 0, 0);
    dBodySetAngularVel(b, 0, 0, 0);

}


void resetSim()
{
    resetBall(ball1_body, 0);
    resetBall(ball2_body, 1);
}


void start()
{
    dAllocateODEDataForThread(dAllocateMaskAll);

    world = dWorldCreate();
    dWorldSetGravity (world,0,0,-9.8);

    contact_group = dJointGroupCreate(0);

    space = dSimpleSpaceCreate (0);


    // first, the ground plane
    // it has to coincide with the plane we have in drawstuff
    ground = dCreatePlane(space, 0, 0, 1, 0);


    // now a ball
    dMass m;
    dMassSetSphere(&m, 0.1, ball_radius);

    ball1_geom = dCreateSphere(space, ball_radius);
    ball1_body = dBodyCreate(world);
    dGeomSetBody(ball1_geom, ball1_body);
    dBodySetMass(ball1_body, &m);

    ball2_geom = dCreateSphere(space, ball_radius);
    ball2_body = dBodyCreate(world);
    dGeomSetBody(ball2_geom, ball2_body);
    dBodySetMass(ball2_body, &m);




    // tracks made out of boxes
    dGeomID trk;
    dMatrix3 r1, r2, r3;
    dVector3 ro = {0, -(0.5*track_gauge + 0.5*track_width), track_elevation};
    dMatrix3 s1, s2, s3;
    dVector3 so = {0, 0.5*track_gauge + 0.5*track_width, track_elevation};

    dRFromAxisAndAngle(r1, 1, 0, 0,  track_angle);
    dRFromAxisAndAngle(r2, 0, 1, 0, -track_incl);
    dMultiply0_333(r3, r2, r1);

    dRFromAxisAndAngle(s1, 1, 0, 0, -track_angle);
    dRFromAxisAndAngle(s2, 0, 1, 0, -track_incl);
    dMultiply0_333(s3, s2, s1);

    trk = dCreateBox(space, track_len, track_width, track_height);
    dGeomSetPosition(trk, ro[0], ro[1] + balls_sep, ro[2]);
    dGeomSetRotation(trk, r3);

    trk = dCreateBox(space, track_len, track_width, track_height);
    dGeomSetPosition(trk, so[0], so[1] + balls_sep, so[2]);
    dGeomSetRotation(trk, s3);



    

    // tracks made out of trimesh
    for (unsigned i=0; i<n_box_verts; ++i) {
        dVector3 p;
        dMultiply0_331(p, s3, box_verts[i]);
        dAddVectors3(p, p, so);
        dCopyVector3(track_verts[i], p);
    }
    // trimesh tracks 2, transform all vertices by s3
    for (unsigned i=0; i<n_box_verts; ++i) {
        dVector3 p;
        dMultiply0_331(p, r3, box_verts[i]);
        dAddVectors3(p, p, ro);
        dCopyVector3(track_verts[n_box_verts + i], p);
    }

    // copy face indices
    for (unsigned i=0; i<n_box_faces; ++i)
        for (unsigned j=0; j<3; ++j) // each face index
            track_faces[3*i+j] = box_faces[3*i+j];
    for (unsigned i=0; i<n_box_faces; ++i)
        for (unsigned j=0; j<3; ++j) // each face index
            track_faces[3*(i + n_box_faces)+j] = box_faces[3*i+j] + n_box_verts;

    mesh_data = dGeomTriMeshDataCreate();
    dGeomTriMeshDataBuildSimple(mesh_data,
                                track_verts[0], n_track_verts,
                                track_faces, 3*n_track_faces);
    mesh_geom = dCreateTriMesh(space, mesh_data, 0, 0, 0);





    resetSim();
    

    // initial camera position
    static float xyz[3] = {-5.9414,-0.4804,2.9800};
    static float hpr[3] = {32.5000,-10.0000,0.0000};
    dsSetViewpoint (xyz,hpr);

    dsSetSphereQuality(3);
}


void nearCallback(void *, dGeomID a, dGeomID b)
{
    const unsigned max_contacts = 8;
    dContact contacts[max_contacts];
    
    if (!dGeomGetBody(a) && !dGeomGetBody(b))
        return; // don't handle static geom collisions

    int n = dCollide(a, b, max_contacts, &contacts[0].geom, sizeof(dContact));
    //clog << "got " << n << " contacts" << endl;

    /* Simple contact merging:
     * If we have contacts that are too close with the same normal, keep only
     * the one with maximum depth.
     * The epsilon that defines what "too close" means can be a heuristic.
     */
    int new_n = 0;
    dReal epsilon = 1e-1; // default
    /* If we know one of the geoms is a sphere, we can base the epsilon on the
     *  sphere's radius.
     */
    dGeomID s = 0;
    if ((dGeomGetClass(a) == dSphereClass && (s = a)) || 
        (dGeomGetClass(b) == dSphereClass && (s = b))) {
        epsilon = dGeomSphereGetRadius(s) * 0.3;
    }


    for (int i=0; i<n; ++i) {

        // this block draws the contact points before merging, in red
        dMatrix3 r;
        dRSetIdentity(r);
        dsSetColor(1, 0, 0);
        dsSetTexture(DS_NONE);
        dsDrawSphere(contacts[i].geom.pos, r, 0.008);

        // let's offset the line a bit to avoid drawing overlap issues
        float xyzf[3], hprf[3];
        dsGetViewpoint(xyzf, hprf);
        dVector3 xyz = {dReal(xyzf[0]), dReal(xyzf[1]), dReal(xyzf[2])};
        dVector3 v;
        dSubtractVectors3(v, contacts[i].geom.pos, xyz);
        dVector3 c;
        dCalcVectorCross3(c, v, contacts[i].geom.pos);
        dNormalize3(c);
        dVector3 pos1;
        dAddScaledVectors3(pos1, contacts[i].geom.pos, c, 1, 0.005);
        dVector3 pos2;
        dAddScaledVectors3(pos2, pos1, contacts[i].geom.normal, 1, 0.05);
        dsDrawLine(pos1, pos2);
        // end of contacts drawing code



        int closest_point = i;
        for (int j=0; j<new_n; ++j) {
            dReal alignment = dCalcVectorDot3(contacts[i].geom.normal, contacts[j].geom.normal);
            if (alignment > 0.99 // about 8 degrees of difference
                &&
                dCalcPointsDistance3(contacts[i].geom.pos, contacts[j].geom.pos) < epsilon) {
                // they are too close
                closest_point = j;
                //clog << "found close points: " << j << " and " << i << endl;
                break;
            }
        }
        
        if (closest_point != i) {
            // we discard one of the points
            if (contacts[i].geom.depth > contacts[closest_point].geom.depth)
                // the new point is deeper, copy it over closest_point
                contacts[closest_point] = contacts[i];
        } else
            contacts[new_n++] = contacts[i]; // the point is preserved
    }
    //clog << "reduced from " << n << " to " << new_n << endl;
    n = new_n;

    for (int i=0; i<n; ++i) {
        contacts[i].surface.mode = dContactBounce | dContactApprox1 | dContactSoftERP;
        contacts[i].surface.mu = 10;
        contacts[i].surface.bounce = 0.2;
        contacts[i].surface.bounce_vel = 0;
        contacts[i].surface.soft_erp = 1e-3;
        //clog << "depth: " << contacts[i].geom.depth << endl;


        dJointID contact = dJointCreateContact(world, contact_group, &contacts[i]);
        dJointAttach(contact, dGeomGetBody(a), dGeomGetBody(b));

        dMatrix3 r;
        dRSetIdentity(r);
        dsSetColor(0, 0, 1);
        dsSetTexture(DS_NONE);
        dsDrawSphere(contacts[i].geom.pos, r, 0.01);
        dsSetColor(0, 1, 0);
        dVector3 pos2;
        dAddScaledVectors3(pos2, contacts[i].geom.pos, contacts[i].geom.normal, 1, 0.1);
        dsDrawLine(contacts[i].geom.pos, pos2);
    }
    //clog << "----" << endl;
}




void stop()
{
    dGeomDestroy(mesh_geom);
    dGeomTriMeshDataDestroy(mesh_data);

    dBodyDestroy(ball1_body);
    dBodyDestroy(ball2_body);

    dGeomDestroy(ground);

    dJointGroupDestroy(contact_group);

    dSpaceDestroy(space); // will destroy all geoms

    dWorldDestroy(world);
}


static void command (int cmd)
{
    switch (cmd) {
        case ' ':
            resetSim();
            break;
    }
}


void drawGeom(dGeomID g)
{
    int gclass = dGeomGetClass(g);
    const dReal *pos = dGeomGetPosition(g);
    const dReal *rot = dGeomGetRotation(g);

    switch (gclass) {
        case dSphereClass:
            dsSetColorAlpha(0, 0.75, 0.5, 0.5);
            dsSetTexture (DS_CHECKERED);
            dsDrawSphere(pos, rot, dGeomSphereGetRadius(g));
            break;
        case dBoxClass:
        {
            dVector3 lengths;
            dsSetColorAlpha(1, 1, 0, 0.5);
            dsSetTexture (DS_WOOD);
            dGeomBoxGetLengths(g, lengths);
            dsDrawBox(pos, rot, lengths);
            break;
        }
        case dTriMeshClass: 
        {
            int numi = dGeomTriMeshGetTriangleCount(g);

            for (int i=0; i<numi; ++i) {
                dVector3 v0, v1, v2;
                dGeomTriMeshGetTriangle(g, i, &v0, &v1, &v2);

                dsSetTexture (DS_WOOD);

                dsSetDrawMode(DS_WIREFRAME);
                dsSetColorAlpha(0, 0, 0, 1.0);
                dsDrawTriangle(pos, rot, v0, v1, v2, true);

                dsSetDrawMode(DS_POLYFILL);
                dsSetColorAlpha(1, 1, 0, 0.5);
                dsDrawTriangle(pos, rot, v0, v1, v2, true);
            }
            break;
        }
        
        default:
        {}
    }
}



void simLoop (int pause)
{
    if (!pause) {

        const dReal step = 0.02;
        const unsigned nsteps = 1;

        for (unsigned i=0; i<nsteps; ++i) {
            dSpaceCollide(space, 0, nearCallback);
            dWorldQuickStep(world, step);
            dJointGroupEmpty(contact_group);
        }
    } else {
        dSpaceCollide(space, 0, nearCallback);
        dJointGroupEmpty(contact_group);
    }
    
    // now we draw everything
    unsigned ngeoms = dSpaceGetNumGeoms(space);
    for (unsigned i=0; i<ngeoms; ++i) {
        dGeomID g = dSpaceGetGeom(space, i);

        if (g == ground)
            continue; // drawstuff is already drawing it for us

        drawGeom(g);
    }
    
    if (dBodyGetPosition(ball1_body)[0] < -track_len)
        resetSim();
}


int main (int argc, char **argv)
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
