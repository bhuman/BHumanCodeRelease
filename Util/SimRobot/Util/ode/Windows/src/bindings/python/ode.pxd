######################################################################
# Python Open Dynamics Engine Wrapper
# Copyright (C) 2004 PyODE developers (see file AUTHORS)
# All rights reserved.
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of EITHER:
#   (1) The GNU Lesser General Public License as published by the Free
#       Software Foundation; either version 2.1 of the License, or (at
#       your option) any later version. The text of the GNU Lesser
#       General Public License is included with this library in the
#       file LICENSE.
#   (2) The BSD-style license that is included with this library in
#       the file LICENSE-BSD.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files
# LICENSE and LICENSE-BSD for more details. 
######################################################################

cdef extern from "stdlib.h":

    void* malloc(long)
    void free(void*)

cdef extern from "stdio.h":
    int printf(char*)

# Include the basic floating point type -> dReal  (either float or double)
#include "_precision.pyx"
    
cdef extern from "ode/ode.h":

    ctypedef double dReal
    
    # Dummy structs
    cdef struct dxWorld:
        int _dummy
    cdef struct dxSpace:
        int _dummy
    cdef struct dxBody:
        int _dummy
    cdef struct dxGeom:
        int _dummy
    cdef struct dxJoint:
        int _dummy
    cdef struct dxJointGroup:
        int _dummy
    cdef struct dxTriMeshData:
        int _dummy
    cdef struct dxHeightfieldData:
        int _dummy

    # Types
    ctypedef dxWorld* dWorldID
    ctypedef dxSpace* dSpaceID
    ctypedef dxBody* dBodyID
    ctypedef dxGeom* dGeomID
    ctypedef dxJoint* dJointID
    ctypedef dxJointGroup* dJointGroupID
    ctypedef dxTriMeshData* dTriMeshDataID
    ctypedef dxHeightfieldData* dHeightfieldDataID
    ctypedef dReal dVector3[4]
    ctypedef dReal dVector4[4]
    ctypedef dReal dMatrix3[4*3]
    ctypedef dReal dMatrix4[4*4]
    ctypedef dReal dMatrix6[8*6]
    ctypedef dReal dQuaternion[4]

    cdef extern dReal dInfinity
    cdef extern int dAMotorUser
    cdef extern int dAMotorEuler

    ctypedef struct dMass:
        dReal    mass
        dVector4 c
        dMatrix3 I
        
    ctypedef struct dJointFeedback:
        dVector3 f1
        dVector3 t1
        dVector3 f2
        dVector3 t2

    ctypedef void dNearCallback(void* data, dGeomID o1, dGeomID o2)
    ctypedef dReal dHeightfieldGetHeight( void* p_user_data, int x, int z )

    ctypedef struct dSurfaceParameters:
        int mode
        dReal mu

        dReal mu2
        dReal bounce
        dReal bounce_vel
        dReal soft_erp
        dReal soft_cfm
        dReal motion1,motion2
        dReal slip1,slip2

    ctypedef struct dContactGeom:
        dVector3 pos
        dVector3 normal
        dReal depth
        dGeomID g1,g2

    ctypedef struct dContact:
        dSurfaceParameters surface
        dContactGeom geom
        dVector3 fdir1


    # World
    dWorldID dWorldCreate()
    void dWorldDestroy (dWorldID)

    void dCloseODE()
    void dInitODE()

    void dWorldSetGravity (dWorldID, dReal x, dReal y, dReal z)
    void dWorldGetGravity (dWorldID, dVector3 gravity)
    void dWorldSetERP (dWorldID, dReal erp)
    dReal dWorldGetERP (dWorldID)
    void dWorldSetCFM (dWorldID, dReal cfm)
    dReal dWorldGetCFM (dWorldID)
    void dWorldStep (dWorldID, dReal stepsize)
    void dWorldQuickStep (dWorldID, dReal stepsize)
    void dWorldSetQuickStepNumIterations (dWorldID, int num)
    int dWorldGetQuickStepNumIterations (dWorldID)
    void dWorldSetContactMaxCorrectingVel (dWorldID, dReal vel)
    dReal dWorldGetContactMaxCorrectingVel (dWorldID)
    void dWorldSetContactSurfaceLayer (dWorldID, dReal depth)
    dReal dWorldGetContactSurfaceLayer (dWorldID)
    void dWorldSetAutoDisableFlag (dWorldID, int do_auto_disable)
    int dWorldGetAutoDisableFlag (dWorldID)
    void dWorldSetAutoDisableLinearThreshold (dWorldID, dReal linear_threshold)
    dReal dWorldGetAutoDisableLinearThreshold (dWorldID)
    void dWorldSetAutoDisableAngularThreshold (dWorldID, dReal angular_threshold)
    dReal dWorldGetAutoDisableAngularThreshold (dWorldID)
    void dWorldSetAutoDisableSteps (dWorldID, int steps)
    int dWorldGetAutoDisableSteps (dWorldID)
    void dWorldSetAutoDisableTime (dWorldID, dReal time)
    dReal dWorldGetAutoDisableTime (dWorldID)
    dReal dWorldGetLinearDamping (dWorldID)
    void dWorldSetLinearDamping (dWorldID, dReal scale)
    dReal dWorldGetAngularDamping (dWorldID)
    void dWorldSetAngularDamping (dWorldID, dReal scale)
    void dWorldImpulseToForce (dWorldID, dReal stepsize,
                               dReal ix, dReal iy, dReal iz, dVector3 force)

    # Body
    dBodyID dBodyCreate (dWorldID)
    void dBodyDestroy (dBodyID)

    void  dBodySetData (dBodyID, void *data)
    void *dBodyGetData (dBodyID)

    void dBodySetPosition   (dBodyID, dReal x, dReal y, dReal z)
    void dBodySetRotation   (dBodyID, dMatrix3 R)
    void dBodySetQuaternion (dBodyID, dQuaternion q)
    void dBodySetLinearVel  (dBodyID, dReal x, dReal y, dReal z)
    void dBodySetAngularVel (dBodyID, dReal x, dReal y, dReal z)
    dReal * dBodyGetPosition   (dBodyID)
    dReal * dBodyGetRotation   (dBodyID)
    dReal * dBodyGetQuaternion (dBodyID)
    dReal * dBodyGetLinearVel  (dBodyID)
    dReal * dBodyGetAngularVel (dBodyID)

    void dBodySetMass (dBodyID, dMass *mass)
    void dBodyGetMass (dBodyID, dMass *mass)

    void dBodyAddForce            (dBodyID, dReal fx, dReal fy, dReal fz)
    void dBodyAddTorque           (dBodyID, dReal fx, dReal fy, dReal fz)
    void dBodyAddRelForce         (dBodyID, dReal fx, dReal fy, dReal fz)
    void dBodyAddRelTorque        (dBodyID, dReal fx, dReal fy, dReal fz)
    void dBodyAddForceAtPos       (dBodyID, dReal fx, dReal fy, dReal fz, dReal px, dReal py, dReal pz)
    void dBodyAddForceAtRelPos    (dBodyID, dReal fx, dReal fy, dReal fz, dReal px, dReal py, dReal pz)
    void dBodyAddRelForceAtPos    (dBodyID, dReal fx, dReal fy, dReal fz, dReal px, dReal py, dReal pz)
    void dBodyAddRelForceAtRelPos (dBodyID, dReal fx, dReal fy, dReal fz, dReal px, dReal py, dReal pz)

    dReal * dBodyGetForce   (dBodyID)
    dReal * dBodyGetTorque  (dBodyID)

    void dBodySetForce(dBodyID, dReal x, dReal y, dReal z)
    void dBodySetTorque(dBodyID, dReal x, dReal y, dReal z)

    void dBodyGetRelPointPos    (dBodyID, dReal px, dReal py, dReal pz, dVector3 result)
    void dBodyGetRelPointVel    (dBodyID, dReal px, dReal py, dReal pz, dVector3 result)
    void dBodyGetPointVel    (dBodyID, dReal px, dReal py, dReal pz,
                              dVector3 result)
    void dBodyGetPosRelPoint (dBodyID, dReal px, dReal py, dReal pz,
                              dVector3 result)
    void dBodyVectorToWorld   (dBodyID, dReal px, dReal py, dReal pz,
                               dVector3 result)
    void dBodyVectorFromWorld (dBodyID, dReal px, dReal py, dReal pz,
                               dVector3 result)

    void dBodySetFiniteRotationMode (dBodyID, int mode)
    void dBodySetFiniteRotationAxis (dBodyID, dReal x, dReal y, dReal z)

    int dBodyGetFiniteRotationMode (dBodyID)
    void dBodyGetFiniteRotationAxis (dBodyID, dVector3 result)

    int dBodyGetNumJoints (dBodyID b)
    dJointID dBodyGetJoint (dBodyID, int index)

    void dBodyEnable (dBodyID)
    void dBodyDisable (dBodyID)
    int dBodyIsEnabled (dBodyID)

    void dBodySetGravityMode (dBodyID b, int mode)
    int dBodyGetGravityMode (dBodyID b)

    void dBodySetDynamic (dBodyID)
    void dBodySetKinematic (dBodyID)
    int dBodyIsKinematic (dBodyID)

    void dBodySetMaxAngularSpeed (dBodyID, dReal max_speed)

    # Joints
    dJointID dJointCreateBall (dWorldID, dJointGroupID)
    dJointID dJointCreateHinge (dWorldID, dJointGroupID)
    dJointID dJointCreateSlider (dWorldID, dJointGroupID)
    dJointID dJointCreateContact (dWorldID, dJointGroupID, dContact *)
    dJointID dJointCreateUniversal (dWorldID, dJointGroupID)
    dJointID dJointCreatePR (dWorldID, dJointGroupID)
    dJointID dJointCreateHinge2 (dWorldID, dJointGroupID)
    dJointID dJointCreateFixed (dWorldID, dJointGroupID)
    dJointID dJointCreateNull (dWorldID, dJointGroupID)
    dJointID dJointCreateAMotor (dWorldID, dJointGroupID)
    dJointID dJointCreateLMotor (dWorldID, dJointGroupID)
    dJointID dJointCreatePlane2D (dWorldID, dJointGroupID)

    void dJointDestroy (dJointID)

    void dJointEnable (dJointID)
    void dJointDisable (dJointID)
    int dJointIsEnabled (dJointID)

    dJointGroupID dJointGroupCreate (int max_size)
    void dJointGroupDestroy (dJointGroupID)
    void dJointGroupEmpty (dJointGroupID)

    void dJointAttach (dJointID, dBodyID body1, dBodyID body2)
    void dJointSetData (dJointID, void *data)
    void *dJointGetData (dJointID)
    int dJointGetType (dJointID)
    dBodyID dJointGetBody (dJointID, int index)

    void dJointSetBallAnchor (dJointID, dReal x, dReal y, dReal z)
    void dJointSetHingeAnchor (dJointID, dReal x, dReal y, dReal z)
    void dJointSetHingeAxis (dJointID, dReal x, dReal y, dReal z)
    void dJointSetHingeParam (dJointID, int parameter, dReal value)
    void dJointAddHingeTorque(dJointID joint, dReal torque)
    void dJointSetSliderAxis (dJointID, dReal x, dReal y, dReal z)
    void dJointSetSliderParam (dJointID, int parameter, dReal value)
    void dJointAddSliderForce(dJointID joint, dReal force)
    void dJointSetHinge2Anchor (dJointID, dReal x, dReal y, dReal z)
    void dJointSetHinge2Axis1 (dJointID, dReal x, dReal y, dReal z)
    void dJointSetHinge2Axis2 (dJointID, dReal x, dReal y, dReal z)
    void dJointSetHinge2Param (dJointID, int parameter, dReal value)
    void dJointAddHinge2Torques(dJointID joint, dReal torque1, dReal torque2)
    void dJointSetUniversalAnchor (dJointID, dReal x, dReal y, dReal z)
    void dJointSetUniversalAxis1 (dJointID, dReal x, dReal y, dReal z)
    void dJointSetUniversalAxis2 (dJointID, dReal x, dReal y, dReal z)
    void dJointSetUniversalParam (dJointID, int parameter, dReal value)
    void dJointAddUniversalTorques(dJointID joint, dReal torque1, dReal torque2)
    void dJointSetFixed (dJointID)
    void dJointSetAMotorNumAxes (dJointID, int num)
    void dJointSetAMotorAxis (dJointID, int anum, int rel, dReal x, dReal y, dReal z)
    void dJointSetAMotorAngle (dJointID, int anum, dReal angle)
    void dJointSetAMotorParam (dJointID, int parameter, dReal value)
    void dJointSetAMotorMode (dJointID, int mode)
    void dJointAddAMotorTorques (dJointID, dReal torque1, dReal torque2, dReal torque3)
    void dJointSetLMotorAxis (dJointID, int anum, int rel, dReal x, dReal y, dReal z)
    void dJointSetLMotorNumAxes (dJointID, int num)
    void dJointSetLMotorParam (dJointID, int parameter, dReal value)
    
    void dJointGetBallAnchor (dJointID, dVector3 result)
    void dJointGetBallAnchor2 (dJointID, dVector3 result)
    void dJointGetHingeAnchor (dJointID, dVector3 result)
    void dJointGetHingeAnchor2 (dJointID, dVector3 result)
    void dJointGetHingeAxis (dJointID, dVector3 result)
    dReal dJointGetHingeParam (dJointID, int parameter)
    dReal dJointGetHingeAngle (dJointID)
    dReal dJointGetHingeAngleRate (dJointID)
    dReal dJointGetSliderPosition (dJointID)
    dReal dJointGetSliderPositionRate (dJointID)
    void dJointGetSliderAxis (dJointID, dVector3 result)
    dReal dJointGetSliderParam (dJointID, int parameter)
    void dJointGetHinge2Anchor (dJointID, dVector3 result)
    void dJointGetHinge2Anchor2 (dJointID, dVector3 result)
    void dJointGetHinge2Axis1 (dJointID, dVector3 result)
    void dJointGetHinge2Axis2 (dJointID, dVector3 result)
    dReal dJointGetHinge2Param (dJointID, int parameter)
    dReal dJointGetHinge2Angle1 (dJointID)
    dReal dJointGetHinge2Angle1Rate (dJointID)
    dReal dJointGetHinge2Angle2Rate (dJointID)
    void dJointGetUniversalAnchor (dJointID, dVector3 result)
    void dJointGetUniversalAnchor2 (dJointID, dVector3 result)
    void dJointGetUniversalAxis1 (dJointID, dVector3 result)
    void dJointGetUniversalAxis2 (dJointID, dVector3 result)
    dReal dJointGetUniversalParam (dJointID, int parameter)
    dReal dJointGetUniversalAngle1 (dJointID)
    dReal dJointGetUniversalAngle2 (dJointID)
    dReal dJointGetUniversalAngle1Rate (dJointID)
    dReal dJointGetUniversalAngle2Rate (dJointID)
    int dJointGetAMotorNumAxes (dJointID)
    void dJointGetAMotorAxis (dJointID, int anum, dVector3 result)
    int dJointGetAMotorAxisRel (dJointID, int anum)
    dReal dJointGetAMotorAngle (dJointID, int anum)
    dReal dJointGetAMotorAngleRate (dJointID, int anum)
    dReal dJointGetAMotorParam (dJointID, int parameter)
    int dJointGetAMotorMode (dJointID)
    int dJointGetLMotorNumAxes (dJointID)
    void dJointGetLMotorAxis (dJointID, int anum, dVector3 result)
    dReal dJointGetLMotorParam (dJointID, int parameter)
    void dJointSetPlane2DXParam (dJointID, int parameter, dReal value)
    void dJointSetPlane2DYParam (dJointID, int parameter, dReal value)
    void dJointSetPlane2DAngleParam (dJointID, int parameter, dReal value)
    dReal dJointGetPRPosition (dJointID j)
    void dJointSetPRAnchor (dJointID j, dReal x, dReal y, dReal z)
    void dJointSetPRAxis1 (dJointID j, dReal x, dReal y, dReal z)
    void dJointSetPRAxis2 (dJointID j, dReal x, dReal y, dReal z)
    void dJointGetPRAnchor (dJointID j, dVector3 result)
    void dJointGetPRAxis1 (dJointID j, dVector3 result)
    void dJointGetPRAxis2 (dJointID j, dVector3 result)

    void dJointSetFeedback (dJointID, dJointFeedback *)
    dJointFeedback *dJointGetFeedback (dJointID)

    int dAreConnected (dBodyID, dBodyID)

    # Mass
    void dMassSetZero (dMass *)
    void dMassSetParameters (dMass *, dReal themass,
             dReal cgx, dReal cgy, dReal cgz,
             dReal I11, dReal I22, dReal I33,
             dReal I12, dReal I13, dReal I23)
    void dMassSetSphere (dMass *, dReal density, dReal radius)
    void dMassSetSphereTotal (dMass *, dReal total_mass, dReal radius)
    void dMassSetCapsule (dMass *, dReal density, int direction, dReal radius, dReal length)
    void dMassSetCapsuleTotal (dMass *, dReal total_mass, int direction, dReal radius, dReal length)
    void dMassSetCylinder (dMass *, dReal density, int direction,
          dReal radius, dReal length)
    void dMassSetCylinderTotal (dMass *, dReal total_mass, int direction,
          dReal radius, dReal length)
    void dMassSetBox (dMass *, dReal density,
          dReal lx, dReal ly, dReal lz)
    void dMassSetBoxTotal (dMass *, dReal total_mass,
          dReal lx, dReal ly, dReal lz)
    void dMassAdjust (dMass *, dReal newmass)
    void dMassTranslate (dMass *, dReal x, dReal y, dReal z)
    void dMassRotate (dMass *, dMatrix3 R)
    void dMassAdd (dMass *a, dMass *b)

    # Space
#    dSpaceID dSimpleSpaceCreate(int space)
#    dSpaceID dHashSpaceCreate(int space)
    dSpaceID dSimpleSpaceCreate(dSpaceID space)
    dSpaceID dHashSpaceCreate(dSpaceID space)
    dSpaceID dQuadTreeSpaceCreate (dSpaceID space, dVector3 Center,
                                   dVector3 Extents, int Depth)

    void dSpaceDestroy (dSpaceID)
    void dSpaceAdd (dSpaceID, dGeomID)
    void dSpaceRemove (dSpaceID, dGeomID)
    int dSpaceQuery (dSpaceID, dGeomID)
    void dSpaceCollide (dSpaceID space, void *data, dNearCallback *callback)
    void dSpaceCollide2 (dGeomID o1, dGeomID o2, void *data, dNearCallback *callback)

    void dHashSpaceSetLevels (dSpaceID space, int minlevel, int maxlevel)
    void dHashSpaceGetLevels (dSpaceID space, int *minlevel, int *maxlevel)

    void dSpaceSetCleanup (dSpaceID space, int mode)
    int dSpaceGetCleanup (dSpaceID space)

    int dSpaceGetNumGeoms (dSpaceID)
    dGeomID dSpaceGetGeom (dSpaceID, int i)

    # Geom
    dGeomID dCreateSphere (dSpaceID space, dReal radius)
    dGeomID dCreateBox (dSpaceID space, dReal lx, dReal ly, dReal lz)
    dGeomID dCreatePlane (dSpaceID space, dReal a, dReal b, dReal c, dReal d)
    dGeomID dCreateCapsule (dSpaceID space, dReal radius, dReal length)
    dGeomID dCreateCylinder (dSpaceID space, dReal radius, dReal length)
    dGeomID dCreateGeomGroup (dSpaceID space)

    void dGeomSphereSetRadius (dGeomID sphere, dReal radius)
    void dGeomBoxSetLengths (dGeomID box, dReal lx, dReal ly, dReal lz)
    void dGeomPlaneSetParams (dGeomID plane, dReal a, dReal b, dReal c, dReal d)
    void dGeomCapsuleSetParams (dGeomID ccylinder, dReal radius, dReal length)
    void dGeomCylinderSetParams (dGeomID ccylinder, dReal radius, dReal length)

    dReal dGeomSphereGetRadius (dGeomID sphere)
    void  dGeomBoxGetLengths (dGeomID box, dVector3 result)
    void  dGeomPlaneGetParams (dGeomID plane, dVector4 result)
    void  dGeomCapsuleGetParams (dGeomID ccylinder, dReal *radius, dReal *length)
    void  dGeomCylinderGetParams (dGeomID ccylinder, dReal *radius, dReal *length)

    dReal dGeomSpherePointDepth (dGeomID sphere, dReal x, dReal y, dReal z)
    dReal dGeomBoxPointDepth (dGeomID box, dReal x, dReal y, dReal z)
    dReal dGeomPlanePointDepth (dGeomID plane, dReal x, dReal y, dReal z)
    dReal dGeomCapsulePointDepth (dGeomID ccylinder, dReal x, dReal y, dReal z)

    dGeomID dCreateRay (dSpaceID space, dReal length)
    void dGeomRaySetLength (dGeomID ray, dReal length)
    dReal dGeomRayGetLength (dGeomID ray)
    void dGeomRaySet (dGeomID ray, dReal px, dReal py, dReal pz,
          dReal dx, dReal dy, dReal dz)
    void dGeomRayGet (dGeomID ray, dVector3 start, dVector3 dir)

    void dGeomSetData (dGeomID, void *)
    void *dGeomGetData (dGeomID)
    void dGeomSetBody (dGeomID, dBodyID)
    dBodyID dGeomGetBody (dGeomID)
    void dGeomSetPosition (dGeomID, dReal x, dReal y, dReal z)
    void dGeomSetRotation (dGeomID, dMatrix3 R)
    void dGeomSetQuaternion (dGeomID, dQuaternion)
    dReal * dGeomGetPosition (dGeomID)
    dReal * dGeomGetRotation (dGeomID)
    void dGeomGetQuaternion (dGeomID, dQuaternion result)
    void dGeomSetOffsetPosition (dGeomID, dReal x, dReal y, dReal z)
    void dGeomSetOffsetRotation (dGeomID, dMatrix3 R)
    void dGeomClearOffset (dGeomID)
    dReal * dGeomGetOffsetPosition (dGeomID)
    dReal * dGeomGetOffsetRotation (dGeomID)
    void dGeomDestroy (dGeomID)
    void dGeomGetAABB (dGeomID, dReal aabb[6])
    dReal *dGeomGetSpaceAABB (dGeomID)
    int dGeomIsSpace (dGeomID)
    dSpaceID dGeomGetSpace (dGeomID)
    int dGeomGetClass (dGeomID)

    void dGeomSetCategoryBits(dGeomID, unsigned long bits)
    void dGeomSetCollideBits(dGeomID, unsigned long bits)
    unsigned long dGeomGetCategoryBits(dGeomID)
    unsigned long dGeomGetCollideBits(dGeomID)     

    void dGeomEnable (dGeomID)
    void dGeomDisable (dGeomID)
    int dGeomIsEnabled (dGeomID)

    void dGeomGroupAdd (dGeomID group, dGeomID x)
    void dGeomGroupRemove (dGeomID group, dGeomID x)
    int dGeomGroupGetNumGeoms (dGeomID group)
    dGeomID dGeomGroupGetGeom (dGeomID group, int i)

    dGeomID dCreateGeomTransform (dSpaceID space)
    void dGeomTransformSetGeom (dGeomID g, dGeomID obj)
    dGeomID dGeomTransformGetGeom (dGeomID g)
    void dGeomTransformSetCleanup (dGeomID g, int mode)
    int dGeomTransformGetCleanup (dGeomID g)
    void dGeomTransformSetInfo (dGeomID g, int mode)
    int dGeomTransformGetInfo (dGeomID g)

    int dCollide (dGeomID o1, dGeomID o2, int flags, dContactGeom *contact, int skip)

    # Trimesh
    dTriMeshDataID dGeomTriMeshDataCreate()
    void dGeomTriMeshDataDestroy(dTriMeshDataID g)
    void dGeomTriMeshDataBuildSingle1 (dTriMeshDataID g, void* Vertices,
                                int VertexStride, int VertexCount,
                                void* Indices, int IndexCount,
                                int TriStride, void* Normals)
    
    void dGeomTriMeshDataBuildSimple(dTriMeshDataID g,
                                 dReal* Vertices, int VertexCount,
                                 int* Indices, int IndexCount)

    dGeomID dCreateTriMesh (dSpaceID space, dTriMeshDataID Data,
                            void* Callback,
                            void* ArrayCallback,
                            void* RayCallback)   

    void dGeomTriMeshSetData (dGeomID g, dTriMeshDataID Data)
    
    void dGeomTriMeshClearTCCache (dGeomID g)

    void dGeomTriMeshGetTriangle (dGeomID g, int Index, dVector3 *v0,
                                  dVector3 *v1, dVector3 *v2)

    int dGeomTriMeshGetTriangleCount (dGeomID g)

    void dGeomTriMeshGetPoint (dGeomID g, int Index, dReal u, dReal v,
                               dVector3 Out)

    void dGeomTriMeshEnableTC(dGeomID g, int geomClass, int enable)
    int dGeomTriMeshIsTCEnabled(dGeomID g, int geomClass)

    # Heightfield
    dHeightfieldDataID dGeomHeightfieldDataCreate()
    void dGeomHeightfieldDataDestroy(dHeightfieldDataID g)
    void dGeomHeightfieldDataBuildCallback(dHeightfieldDataID d,
                                           void* pUserData,
                                           dHeightfieldGetHeight* pCallback,
                                           dReal width, dReal depth,
                                           int widthSamples, int depthSamples,
                                           dReal scale, dReal offset,
                                           dReal thickness, int bWrap)
    dGeomID dCreateHeightfield (dSpaceID space, dHeightfieldDataID data,
                                 int bPlaceable)

