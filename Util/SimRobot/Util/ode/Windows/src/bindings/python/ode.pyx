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

from ode cimport *


paramLoStop        = 0
paramHiStop        = 1
paramVel           = 2
paramFMax          = 3
paramFudgeFactor   = 4
paramBounce        = 5
paramCFM           = 6
paramStopERP       = 7
paramStopCFM       = 8
paramSuspensionERP = 9
paramSuspensionCFM = 10

ParamLoStop        = 0
ParamHiStop        = 1
ParamVel           = 2
ParamFMax          = 3
ParamFudgeFactor   = 4
ParamBounce        = 5
ParamCFM           = 6
ParamStopERP       = 7
ParamStopCFM       = 8
ParamSuspensionERP = 9
ParamSuspensionCFM = 10

ParamLoStop2        = 256 + 0
ParamHiStop2        = 256 + 1
ParamVel2           = 256 + 2
ParamFMax2          = 256 + 3
ParamFudgeFactor2   = 256 + 4
ParamBounce2        = 256 + 5
ParamCFM2           = 256 + 6
ParamStopERP2       = 256 + 7
ParamStopCFM2       = 256 + 8
ParamSuspensionERP2 = 256 + 9
ParamSuspensionCFM2 = 256 + 10

ParamLoStop3        = 512 + 0
ParamHiStop3        = 512 + 1
ParamVel3           = 512 + 2
ParamFMax3          = 512 + 3
ParamFudgeFactor3   = 512 + 4
ParamBounce3        = 512 + 5
ParamCFM3           = 512 + 6
ParamStopERP3       = 512 + 7
ParamStopCFM3       = 512 + 8
ParamSuspensionERP3 = 512 + 9
ParamSuspensionCFM3 = 512 + 10

ParamGroup = 256

ContactMu2    = 0x001
ContactFDir1    = 0x002
ContactBounce    = 0x004
ContactSoftERP    = 0x008
ContactSoftCFM    = 0x010
ContactMotion1    = 0x020
ContactMotion2    = 0x040
ContactSlip1    = 0x080
ContactSlip2    = 0x100

ContactApprox0 = 0x0000
ContactApprox1_1    = 0x1000
ContactApprox1_2    = 0x2000
ContactApprox1    = 0x3000

AMotorUser = dAMotorUser
AMotorEuler = dAMotorEuler

Infinity = dInfinity


import weakref
_geom_c2py_lut = weakref.WeakValueDictionary()


cdef class Mass:
    """Mass parameters of a rigid body.

    This class stores mass parameters of a rigid body which can be
    accessed through the following attributes:

     - mass: The total mass of the body (float)
     - c:    The center of gravity position in body frame (3-tuple of floats)
     - I:    The 3x3 inertia tensor in body frame (3-tuple of 3-tuples)

    This class wraps the dMass structure from the C API.

    @ivar mass: The total mass of the body
    @ivar c: The center of gravity position in body frame (cx, cy, cz)
    @ivar I: The 3x3 inertia tensor in body frame ((I11, I12, I13), (I12, I22, I23), (I13, I23, I33))
    @type mass: float
    @type c: 3-tuple of floats
    @type I: 3-tuple of 3-tuples of floats
    """
    cdef dMass _mass

    def __cinit__(self):
        dMassSetZero(&self._mass)

    def setZero(self):
        """setZero()

        Set all the mass parameters to zero."""
        dMassSetZero(&self._mass)

    def setParameters(self, mass, cgx, cgy, cgz, I11, I22, I33, I12, I13, I23):
        """setParameters(mass, cgx, cgy, cgz, I11, I22, I33, I12, I13, I23)

        Set the mass parameters to the given values.

        @param mass: Total mass of the body.
        @param cgx: Center of gravity position in the body frame (x component).
        @param cgy: Center of gravity position in the body frame (y component).
        @param cgz: Center of gravity position in the body frame (z component).
        @param I11: Inertia tensor
        @param I22: Inertia tensor
        @param I33: Inertia tensor
        @param I12: Inertia tensor
        @param I13: Inertia tensor
        @param I23: Inertia tensor
        @type mass: float
        @type cgx: float
        @type cgy: float
        @type cgz: float
        @type I11: float
        @type I22: float
        @type I33: float
        @type I12: float
        @type I13: float
        @type I23: float
        """
        dMassSetParameters(&self._mass, mass, cgx, cgy, cgz,
                           I11, I22, I33, I12, I13, I23)

    def setSphere(self, density, radius):
        """setSphere(density, radius)
        
        Set the mass parameters to represent a sphere of the given radius
        and density, with the center of mass at (0,0,0) relative to the body.

        @param density: The density of the sphere
        @param radius: The radius of the sphere
        @type density: float
        @type radius: float
        """
        dMassSetSphere(&self._mass, density, radius)

    def setSphereTotal(self, total_mass, radius):
        """setSphereTotal(total_mass, radius)
        
        Set the mass parameters to represent a sphere of the given radius
        and mass, with the center of mass at (0,0,0) relative to the body.

        @param total_mass: The total mass of the sphere
        @param radius: The radius of the sphere
        @type total_mass: float
        @type radius: float
        """
        dMassSetSphereTotal(&self._mass, total_mass, radius)

    def setCapsule(self, density, direction, radius, length):
        """setCapsule(density, direction, radius, length)

        Set the mass parameters to represent a capsule of the given parameters
        and density, with the center of mass at (0,0,0) relative to the body.
        The radius of the cylinder (and the spherical cap) is radius. The length
        of the cylinder (not counting the spherical cap) is length. The
        cylinder's long axis is oriented along the body's x, y or z axis
        according to the value of direction (1=x, 2=y, 3=z). The first function
        accepts the density of the object, the second accepts its total mass.

        @param density: The density of the capsule
        @param direction: The direction of the capsule's cylinder (1=x axis, 2=y axis, 3=z axis)
        @param radius: The radius of the capsule's cylinder
        @param length: The length of the capsule's cylinder (without the caps)
        @type density: float
        @type direction: int
        @type radius: float
        @type length: float
        """
        dMassSetCapsule(&self._mass, density, direction, radius, length)

    def setCapsuleTotal(self, total_mass, direction, radius, length):
        """setCapsuleTotal(total_mass, direction, radius, length)

        Set the mass parameters to represent a capsule of the given parameters
        and mass, with the center of mass at (0,0,0) relative to the body. The
        radius of the cylinder (and the spherical cap) is radius. The length of
        the cylinder (not counting the spherical cap) is length. The cylinder's
        long axis is oriented along the body's x, y or z axis according to the
        value of direction (1=x, 2=y, 3=z). The first function accepts the
        density of the object, the second accepts its total mass.

        @param total_mass: The total mass of the capsule
        @param direction: The direction of the capsule's cylinder (1=x axis, 2=y axis, 3=z axis)
        @param radius: The radius of the capsule's cylinder
        @param length: The length of the capsule's cylinder (without the caps)
        @type total_mass: float
        @type direction: int
        @type radius: float
        @type length: float
        """
        dMassSetCapsuleTotal(&self._mass, total_mass, direction,
                             radius, length)

    def setCylinder(self, density, direction, r, h):
        """setCylinder(density, direction, r, h)
        
        Set the mass parameters to represent a flat-ended cylinder of
        the given parameters and density, with the center of mass at
        (0,0,0) relative to the body. The radius of the cylinder is r.
        The length of the cylinder is h. The cylinder's long axis is
        oriented along the body's x, y or z axis according to the value
        of direction (1=x, 2=y, 3=z).

        @param density: The density of the cylinder
        @param direction: The direction of the cylinder (1=x axis, 2=y axis, 3=z axis)
        @param r: The radius of the cylinder
        @param h: The length of the cylinder
        @type density: float
        @type direction: int
        @type r: float
        @type h: float
        """
        dMassSetCylinder(&self._mass, density, direction, r, h)

    def setCylinderTotal(self, total_mass, direction, r, h):
        """setCylinderTotal(total_mass, direction, r, h)
        
        Set the mass parameters to represent a flat-ended cylinder of
        the given parameters and mass, with the center of mass at
        (0,0,0) relative to the body. The radius of the cylinder is r.
        The length of the cylinder is h. The cylinder's long axis is
        oriented along the body's x, y or z axis according to the value
        of direction (1=x, 2=y, 3=z).

        @param total_mass: The total mass of the cylinder
        @param direction: The direction of the cylinder (1=x axis, 2=y axis, 3=z axis)
        @param r: The radius of the cylinder
        @param h: The length of the cylinder
        @type total_mass: float
        @type direction: int
        @type r: float
        @type h: float
        """
        dMassSetCylinderTotal(&self._mass, total_mass, direction, r, h)

    def setBox(self, density, lx, ly, lz):
        """setBox(density, lx, ly, lz)

        Set the mass parameters to represent a box of the given
        dimensions and density, with the center of mass at (0,0,0)
        relative to the body. The side lengths of the box along the x,
        y and z axes are lx, ly and lz.

        @param density: The density of the box
        @param lx: The length along the x axis
        @param ly: The length along the y axis
        @param lz: The length along the z axis
        @type density: float
        @type lx: float
        @type ly: float
        @type lz: float
        """
        dMassSetBox(&self._mass, density, lx, ly, lz)

    def setBoxTotal(self, total_mass, lx, ly, lz):
        """setBoxTotal(total_mass, lx, ly, lz)

        Set the mass parameters to represent a box of the given
        dimensions and mass, with the center of mass at (0,0,0)
        relative to the body. The side lengths of the box along the x,
        y and z axes are lx, ly and lz.

        @param total_mass: The total mass of the box
        @param lx: The length along the x axis
        @param ly: The length along the y axis
        @param lz: The length along the z axis
        @type total_mass: float
        @type lx: float
        @type ly: float
        @type lz: float
        """
        dMassSetBoxTotal(&self._mass, total_mass, lx, ly, lz)

    def adjust(self, newmass):
        """adjust(newmass)

        Adjust the total mass. Given mass parameters for some object,
        adjust them so the total mass is now newmass. This is useful
        when using the setXyz() methods to set the mass parameters for
        certain objects - they take the object density, not the total
        mass.

        @param newmass: The new total mass
        @type newmass: float
        """
        dMassAdjust(&self._mass, newmass)

    def translate(self, t):
        """translate(t)

        Adjust mass parameters. Given mass parameters for some object,
        adjust them to represent the object displaced by (x,y,z)
        relative to the body frame.

        @param t: Translation vector (x, y, z)
        @type t: 3-tuple of floats
        """
        dMassTranslate(&self._mass, t[0], t[1], t[2])

#    def rotate(self, R):
#        """
#        Given mass parameters for some object, adjust them to
#        represent the object rotated by R relative to the body frame.
#        """
#        pass

    def add(self, Mass b):
        """add(b)

        Add the mass b to the mass object. Masses can also be added using
        the + operator.

        @param b: The mass to add to this mass
        @type b: Mass
        """
        dMassAdd(&self._mass, &b._mass)

    def __getattr__(self, name):
        if name == "mass":
            return self._mass.mass
        elif name == "c":
            return self._mass.c[0], self._mass.c[1], self._mass.c[2]
        elif name == "I":
            return ((self._mass.I[0], self._mass.I[1], self._mass.I[2]),
                    (self._mass.I[4], self._mass.I[5], self._mass.I[6]),
                    (self._mass.I[8], self._mass.I[9], self._mass.I[10]))
        else:
            raise AttributeError("Mass object has no attribute '%s'" % name)

    def __setattr__(self, name, value):
        if name == "mass":
            self.adjust(value)
        elif name == "c":
            raise AttributeError("Use the setParameter() method to change c")
        elif name == "I":
            raise AttributeError("Use the setParameter() method to change I")
        else:
            raise AttributeError("Mass object has no attribute '%s" % name)

    def __add__(self, Mass b):
        self.add(b)
        return self

    def __str__(self):
        m = str(self._mass.mass)
        sc0 = str(self._mass.c[0])
        sc1 = str(self._mass.c[1])
        sc2 = str(self._mass.c[2])
        I11 = str(self._mass.I[0])
        I22 = str(self._mass.I[5])
        I33 = str(self._mass.I[10])
        I12 = str(self._mass.I[1])
        I13 = str(self._mass.I[2])
        I23 = str(self._mass.I[6])
        return ("Mass=%s\n"
                "Cg=(%s, %s, %s)\n"
                "I11=%s I22=%s I33=%s\n"
                "I12=%s I13=%s I23=%s" %
                (m, sc0, sc1, sc2, I11, I22, I33, I12, I13, I23))
#        return ("Mass=%s / "
#                "Cg=(%s, %s, %s) / "
#                "I11=%s I22=%s I33=%s "
#                "I12=%s I13=%s I23=%s" %
#                (m, sc0, sc1, sc2, I11, I22, I33, I12, I13, I23))


cdef class Contact:
    """This class represents a contact between two bodies in one point.

    A Contact object stores all the input parameters for a ContactJoint.
    This class wraps the ODE dContact structure which has 3 components::

     struct dContact {
       dSurfaceParameters surface;
       dContactGeom geom;
       dVector3 fdir1;
     };

    This wrapper class provides methods to get and set the items of those
    structures.
    """
    
    cdef dContact _contact

    def __cinit__(self):
        self._contact.surface.mode = ContactBounce
        self._contact.surface.mu = dInfinity

        self._contact.surface.bounce = 0.1

    # getMode
    def getMode(self):
        """getMode() -> flags

        Return the contact flags.
        """
        return self._contact.surface.mode

    # setMode
    def setMode(self, flags):
        """setMode(flags)

        Set the contact flags. The argument m is a combination of the
        ContactXyz flags (ContactMu2, ContactBounce, ...).
 
        @param flags: Contact flags
        @type flags: int
        """
        self._contact.surface.mode = flags

    # getMu
    def getMu(self):
        """getMu() -> float

        Return the Coulomb friction coefficient.
        """
        return self._contact.surface.mu

    # setMu
    def setMu(self, mu):
        """setMu(mu)

        Set the Coulomb friction coefficient.

        @param mu: Coulomb friction coefficient (0..Infinity)
        @type mu: float
        """
        self._contact.surface.mu = mu

    # getMu2
    def getMu2(self):
        """getMu2() -> float

        Return the optional Coulomb friction coefficient for direction 2.
        """
        return self._contact.surface.mu2

    # setMu2
    def setMu2(self, mu):
        """setMu2(mu)

        Set the optional Coulomb friction coefficient for direction 2.

        @param mu: Coulomb friction coefficient (0..Infinity)
        @type mu: float
        """
        self._contact.surface.mu2 = mu

    # getBounce
    def getBounce(self):
        """getBounce() -> float

        Return the restitution parameter.
        """
        return self._contact.surface.bounce

    # setBounce
    def setBounce(self, b):
        """setBounce(b)

        @param b: Restitution parameter (0..1)
        @type b: float
        """
        self._contact.surface.bounce = b

    # getBounceVel
    def getBounceVel(self):
        """getBounceVel() -> float

        Return the minimum incoming velocity necessary for bounce.
        """
        return self._contact.surface.bounce_vel

    # setBounceVel
    def setBounceVel(self, bv):
        """setBounceVel(bv)

        Set the minimum incoming velocity necessary for bounce. Incoming
        velocities below this will effectively have a bounce parameter of 0.

        @param bv: Velocity
        @type bv: float
        """
        self._contact.surface.bounce_vel = bv

    # getSoftERP
    def getSoftERP(self):
        """getSoftERP() -> float

        Return the contact normal "softness" parameter.
        """
        return self._contact.surface.soft_erp

    # setSoftERP
    def setSoftERP(self, erp):
        """setSoftERP(erp)

        Set the contact normal "softness" parameter.

        @param erp: Softness parameter
        @type erp: float
        """
        self._contact.surface.soft_erp = erp

    # getSoftCFM
    def getSoftCFM(self):
        """getSoftCFM() -> float

        Return the contact normal "softness" parameter.
        """
        return self._contact.surface.soft_cfm

    # setSoftCFM
    def setSoftCFM(self, cfm):
        """setSoftCFM(cfm)

        Set the contact normal "softness" parameter.

        @param cfm: Softness parameter
        @type cfm: float
        """
        self._contact.surface.soft_cfm = cfm

    # getMotion1
    def getMotion1(self):
        """getMotion1() -> float

        Get the surface velocity in friction direction 1.
        """
        return self._contact.surface.motion1

    # setMotion1
    def setMotion1(self, m):
        """setMotion1(m)

        Set the surface velocity in friction direction 1.

        @param m: Surface velocity
        @type m: float
        """
        self._contact.surface.motion1 = m

    # getMotion2
    def getMotion2(self):
        """getMotion2() -> float

        Get the surface velocity in friction direction 2.
        """
        return self._contact.surface.motion2

    # setMotion2
    def setMotion2(self, m):
        """setMotion2(m)

        Set the surface velocity in friction direction 2.

        @param m: Surface velocity
        @type m: float
        """
        self._contact.surface.motion2 = m

    # getSlip1
    def getSlip1(self):
        """getSlip1() -> float

        Get the coefficient of force-dependent-slip (FDS) for friction
        direction 1.
        """
        return self._contact.surface.slip1

    # setSlip1
    def setSlip1(self, s):
        """setSlip1(s)

        Set the coefficient of force-dependent-slip (FDS) for friction
        direction 1.

        @param s: FDS coefficient
        @type s: float
        """
        self._contact.surface.slip1 = s

    # getSlip2
    def getSlip2(self):
        """getSlip2() -> float

        Get the coefficient of force-dependent-slip (FDS) for friction
        direction 2.
        """
        return self._contact.surface.slip2

    # setSlip2
    def setSlip2(self, s):
        """setSlip2(s)

        Set the coefficient of force-dependent-slip (FDS) for friction
        direction 1.

        @param s: FDS coefficient
        @type s: float
        """
        self._contact.surface.slip2 = s

    # getFDir1
    def getFDir1(self):
        """getFDir1() -> (x, y, z)

        Get the "first friction direction" vector that defines a direction
        along which frictional force is applied.
        """
        return (self._contact.fdir1[0],
                self._contact.fdir1[1],
                self._contact.fdir1[2])

    # setFDir1
    def setFDir1(self, fdir):
        """setFDir1(fdir)

        Set the "first friction direction" vector that defines a direction
        along which frictional force is applied. It must be of unit length
        and perpendicular to the contact normal (so it is typically
        tangential to the contact surface).

        @param fdir: Friction direction
        @type fdir: 3-sequence of floats
        """
        self._contact.fdir1[0] = fdir[0]
        self._contact.fdir1[1] = fdir[1]
        self._contact.fdir1[2] = fdir[2]

    # getContactGeomParams
    def getContactGeomParams(self):
        """getContactGeomParams() -> (pos, normal, depth, geom1, geom2)

        Get the ContactGeom structure of the contact.

        The return value is a tuple (pos, normal, depth, geom1, geom2)
        where pos and normal are 3-tuples of floats and depth is a single
        float. geom1 and geom2 are the Geom objects of the geoms in contact.
        """
        cdef long id1, id2

        pos = (self._contact.geom.pos[0],
               self._contact.geom.pos[1],
               self._contact.geom.pos[2])
        normal = (self._contact.geom.normal[0],
                  self._contact.geom.normal[1],
                  self._contact.geom.normal[2])
        depth = self._contact.geom.depth

        id1 = <long>self._contact.geom.g1
        id2 = <long>self._contact.geom.g2
        g1 = _geom_c2py_lut[id1]
        g2 = _geom_c2py_lut[id2]
        return pos, normal, depth, g1, g2

    # setContactGeomParams
    def setContactGeomParams(self, pos, normal, depth, g1=None, g2=None):
        """setContactGeomParams(pos, normal, depth, geom1=None, geom2=None)
        
        Set the ContactGeom structure of the contact.

        @param pos:  Contact position, in global coordinates
        @type pos: 3-sequence of floats
        @param normal: Unit length normal vector
        @type normal: 3-sequence of floats
        @param depth: Depth to which the two bodies inter-penetrate
        @type depth: float
        @param geom1: Geometry object 1 that collided
        @type geom1: Geom
        @param geom2: Geometry object 2 that collided
        @type geom2: Geom
        """

        cdef long id

        self._contact.geom.pos[0] = pos[0]
        self._contact.geom.pos[1] = pos[1]
        self._contact.geom.pos[2] = pos[2]
        self._contact.geom.normal[0] = normal[0]
        self._contact.geom.normal[1] = normal[1]
        self._contact.geom.normal[2] = normal[2]
        self._contact.geom.depth = depth
        if g1 != None:
            id = g1._id()
            self._contact.geom.g1 = <dGeomID>id
        else:
            self._contact.geom.g1 = <dGeomID>0
            
        if g2 != None:
            id = g2._id()
            self._contact.geom.g2 = <dGeomID>id
        else:
            self._contact.geom.g2 = <dGeomID>0


# World
cdef class World:
    """Dynamics world.
    
    The world object is a container for rigid bodies and joints.
    
    
    Constructor::
    
      World()
    """

    cdef dWorldID wid

    def __cinit__(self):
        self.wid = dWorldCreate()

    def __dealloc__(self):
        if self.wid != NULL:
            dWorldDestroy(self.wid)

    # setGravity
    def setGravity(self, gravity):
        """setGravity(gravity)

        Set the world's global gravity vector.

        @param gravity: Gravity vector
        @type gravity: 3-sequence of floats
        """
        dWorldSetGravity(self.wid, gravity[0], gravity[1], gravity[2])

    # getGravity
    def getGravity(self):
        """getGravity() -> 3-tuple

        Return the world's global gravity vector as a 3-tuple of floats.
        """
        cdef dVector3 g
        dWorldGetGravity(self.wid, g)
        return g[0], g[1], g[2]

    # setERP
    def setERP(self, erp):
        """setERP(erp)

        Set the global ERP value, that controls how much error
        correction is performed in each time step. Typical values are
        in the range 0.1-0.8. The default is 0.2.

        @param erp: Global ERP value
        @type erp: float
        """
        dWorldSetERP(self.wid, erp)

    # getERP
    def getERP(self):
        """getERP() -> float

        Get the global ERP value, that controls how much error
        correction is performed in each time step. Typical values are
        in the range 0.1-0.8. The default is 0.2.
        """
        return dWorldGetERP(self.wid)

    # setCFM
    def setCFM(self, cfm):
        """setCFM(cfm)

        Set the global CFM (constraint force mixing) value. Typical
        values are in the range 10E-9 - 1. The default is 10E-5 if
        single precision is being used, or 10E-10 if double precision
        is being used.

        @param cfm: Constraint force mixing value
        @type cfm: float
        """
        dWorldSetCFM(self.wid, cfm)

    # getCFM
    def getCFM(self):
        """getCFM() -> float

        Get the global CFM (constraint force mixing) value. Typical
        values are in the range 10E-9 - 1. The default is 10E-5 if
        single precision is being used, or 10E-10 if double precision
        is being used.
        """
        return dWorldGetCFM(self.wid)

    # step
    def step(self, stepsize):
        """step(stepsize)

        Step the world. This uses a "big matrix" method that takes
        time on the order of O(m3) and memory on the order of O(m2), where m
        is the total number of constraint rows.

        For large systems this will use a lot of memory and can be
        very slow, but this is currently the most accurate method.

        @param stepsize: Time step
        @type stepsize: float
        """

        dWorldStep(self.wid, stepsize)

    # quickStep
    def quickStep(self, stepsize):
        """quickStep(stepsize)
        
        Step the world. This uses an iterative method that takes time
        on the order of O(m*N) and memory on the order of O(m), where m is
        the total number of constraint rows and N is the number of
        iterations.

        For large systems this is a lot faster than dWorldStep, but it
        is less accurate.

        @param stepsize: Time step
        @type stepsize: float
        """
        dWorldQuickStep(self.wid, stepsize)

    # setQuickStepNumIterations
    def setQuickStepNumIterations(self, num):
        """setQuickStepNumIterations(num)
        
        Set the number of iterations that the QuickStep method
        performs per step. More iterations will give a more accurate
        solution, but will take longer to compute. The default is 20
        iterations.

        @param num: Number of iterations
        @type num: int
        """
        
        dWorldSetQuickStepNumIterations(self.wid, num)

    # getQuickStepNumIterations
    def getQuickStepNumIterations(self):
        """getQuickStepNumIterations() -> int
        
        Get the number of iterations that the QuickStep method
        performs per step. More iterations will give a more accurate
        solution, but will take longer to compute. The default is 20
        iterations.
        """
        return dWorldGetQuickStepNumIterations(self.wid)

    # setQuickStepNumIterations
    def setContactMaxCorrectingVel(self, vel):
        """setContactMaxCorrectingVel(vel)

        Set the maximum correcting velocity that contacts are allowed
        to generate. The default value is infinity (i.e. no
        limit). Reducing this value can help prevent "popping" of
        deeply embedded objects.

        @param vel: Maximum correcting velocity
        @type vel: float
        """
        dWorldSetContactMaxCorrectingVel(self.wid, vel)

    # getQuickStepNumIterations
    def getContactMaxCorrectingVel(self):
        """getContactMaxCorrectingVel() -> float

        Get the maximum correcting velocity that contacts are allowed
        to generate. The default value is infinity (i.e. no
        limit). Reducing this value can help prevent "popping" of
        deeply embedded objects.

        """
        return dWorldGetContactMaxCorrectingVel(self.wid)

    # setContactSurfaceLayer
    def setContactSurfaceLayer(self, depth):
        """setContactSurfaceLayer(depth)

        Set the depth of the surface layer around all geometry
        objects. Contacts are allowed to sink into the surface layer
        up to the given depth before coming to rest. The default value
        is zero. Increasing this to some small value (e.g. 0.001) can
        help prevent jittering problems due to contacts being
        repeatedly made and broken.

        @param depth: Surface layer depth
        @type depth: float
        """
        dWorldSetContactSurfaceLayer(self.wid, depth)

    # getContactSurfaceLayer
    def getContactSurfaceLayer(self):
        """getContactSurfaceLayer()

        Get the depth of the surface layer around all geometry
        objects. Contacts are allowed to sink into the surface layer
        up to the given depth before coming to rest. The default value
        is zero. Increasing this to some small value (e.g. 0.001) can
        help prevent jittering problems due to contacts being
        repeatedly made and broken.
        """
        return dWorldGetContactSurfaceLayer(self.wid)

    # setAutoDisableFlag
    def setAutoDisableFlag(self, flag):
        """setAutoDisableFlag(flag)
        
        Set the default auto-disable flag for newly created bodies.

        @param flag: True = Do auto disable
        @type flag: bool
        """
        dWorldSetAutoDisableFlag(self.wid, flag)
        
    # getAutoDisableFlag
    def getAutoDisableFlag(self):
        """getAutoDisableFlag() -> bool
        
        Get the default auto-disable flag for newly created bodies.
        """
        return dWorldGetAutoDisableFlag(self.wid)

    # setAutoDisableLinearThreshold
    def setAutoDisableLinearThreshold(self, threshold):
        """setAutoDisableLinearThreshold(threshold)
        
        Set the default auto-disable linear threshold for newly created
        bodies.

        @param threshold: Linear threshold
        @type threshold: float
        """
        dWorldSetAutoDisableLinearThreshold(self.wid, threshold)

    # getAutoDisableLinearThreshold
    def getAutoDisableLinearThreshold(self):
        """getAutoDisableLinearThreshold() -> float
        
        Get the default auto-disable linear threshold for newly created
        bodies.
        """
        return dWorldGetAutoDisableLinearThreshold(self.wid)

    # setAutoDisableAngularThreshold
    def setAutoDisableAngularThreshold(self, threshold):
        """setAutoDisableAngularThreshold(threshold)
        
        Set the default auto-disable angular threshold for newly created
        bodies.

        @param threshold: Angular threshold
        @type threshold: float
        """
        dWorldSetAutoDisableAngularThreshold(self.wid, threshold)

    # getAutoDisableAngularThreshold
    def getAutoDisableAngularThreshold(self):
        """getAutoDisableAngularThreshold() -> float
        
        Get the default auto-disable angular threshold for newly created
        bodies.
        """
        return dWorldGetAutoDisableAngularThreshold(self.wid)
    
    # setAutoDisableSteps
    def setAutoDisableSteps(self, steps):
        """setAutoDisableSteps(steps)
        
        Set the default auto-disable steps for newly created bodies.

        @param steps: Auto disable steps
        @type steps: int
        """
        dWorldSetAutoDisableSteps(self.wid, steps)

    # getAutoDisableSteps
    def getAutoDisableSteps(self):
        """getAutoDisableSteps() -> int
        
        Get the default auto-disable steps for newly created bodies.
        """
        return dWorldGetAutoDisableSteps(self.wid)

    # setAutoDisableTime
    def setAutoDisableTime(self, time):
        """setAutoDisableTime(time)
        
        Set the default auto-disable time for newly created bodies.

        @param time: Auto disable time
        @type time: float
        """
        dWorldSetAutoDisableTime(self.wid, time)

    # getAutoDisableTime
    def getAutoDisableTime(self):
        """getAutoDisableTime() -> float
        
        Get the default auto-disable time for newly created bodies.
        """
        return dWorldGetAutoDisableTime(self.wid)

    # setLinearDamping
    def setLinearDamping(self, scale):
        """setLinearDamping(scale)

        Set the world's linear damping scale.
                @param scale The linear damping scale that is to be applied to bodies.
                Default is 0 (no damping). Should be in the interval [0, 1].
        @type scale: float
        """
        dWorldSetLinearDamping(self.wid, scale)

    # getLinearDamping
    def getLinearDamping(self):
        """getLinearDamping() -> float

        Get the world's linear damping scale.
        """
        return dWorldGetLinearDamping(self.wid)

    # setAngularDamping
    def setAngularDamping(self, scale):
        """setAngularDamping(scale)

        Set the world's angular damping scale.
                @param scale The angular damping scale that is to be applied to bodies.
                Default is 0 (no damping). Should be in the interval [0, 1].
        @type scale: float
        """
        dWorldSetAngularDamping(self.wid, scale)

    # getAngularDamping
    def getAngularDamping(self):
        """getAngularDamping() -> float

        Get the world's angular damping scale.
        """
        return dWorldGetAngularDamping(self.wid)

    # impulseToForce
    def impulseToForce(self, stepsize, impulse):
        """impulseToForce(stepsize, impulse) -> 3-tuple

        If you want to apply a linear or angular impulse to a rigid
        body, instead of a force or a torque, then you can use this
        function to convert the desired impulse into a force/torque
        vector before calling the dBodyAdd... function.

        @param stepsize: Time step
        @param impulse: Impulse vector
        @type stepsize: float
        @type impulse: 3-tuple of floats
        """
        cdef dVector3 force
        dWorldImpulseToForce(self.wid, stepsize,
                             impulse[0], impulse[1], impulse[2], force)
        return force[0], force[1], force[2]

    # createBody
#    def createBody(self):
#        return Body(self)

    # createBallJoint
#    def createBallJoint(self, jointgroup=None):
#        return BallJoint(self, jointgroup)

    # createHingeJoint
#    def createHingeJoint(self, jointgroup=None):
#        return HingeJoint(self, jointgroup)

    # createHinge2Joint
#    def createHinge2Joint(self, jointgroup=None):
#        return Hinge2Joint(self, jointgroup)

    # createSliderJoint
#    def createSliderJoint(self, jointgroup=None):
#        return SliderJoint(self, jointgroup)

    # createFixedJoint
#    def createFixedJoint(self, jointgroup=None):
#        return FixedJoint(self, jointgroup)

    # createContactJoint
#    def createContactJoint(self, jointgroup, contact):
#        return ContactJoint(self, jointgroup, contact)


# Body
cdef class Body:
    """The rigid body class encapsulating the ODE body.

    This class represents a rigid body that has a location and orientation
    in space and that stores the mass properties of an object.

    When creating a Body object you have to pass the world it belongs to
    as argument to the constructor::

      >>> import ode
      >>> w = ode.World()
      >>> b = ode.Body(w)
    """

    cdef dBodyID bid
    # A reference to the world so that the world won't be destroyed while
    # there are still joints using it.
    cdef object world
    
    # A dictionary with user attributes
    # (set via __getattr__ and __setattr__)
    cdef object userattribs

    def __cinit__(self, World world not None):
        self.bid = dBodyCreate(world.wid)

    def __init__(self, World world not None):
        """Constructor.

        @param world: The world in which the body should be created.
        @type world: World
        """
        self.world = world
        self.userattribs = {}

    def __dealloc__(self):
        if self.bid != NULL:
            dBodyDestroy(self.bid)

    def __getattr__(self, name):
        try:
            return self.userattribs[name]
        except:
            raise AttributeError("Body object has no attribute '%s'" % name)
            
    def __setattr__(self, name, value):
        self.userattribs[name] = value

    def __delattr__(self, name):
        try:
            del self.userattribs[name]
        except:
            raise AttributeError("Body object has no attribute '%s'" % name)

    # setPosition
    def setPosition(self, pos):
        """setPosition(pos)

        Set the position of the body.

        @param pos: The new position
        @type pos: 3-sequence of floats
        """
        dBodySetPosition(self.bid, pos[0], pos[1], pos[2])

    # getPosition
    def getPosition(self):
        """getPosition() -> 3-tuple

        Return the current position of the body.
        """
        cdef dReal* p
        # The "const" in the original return value is cast away
        p = <dReal*>dBodyGetPosition(self.bid)
        return p[0], p[1], p[2]

    # setRotation
    def setRotation(self, R):
        """setRotation(R)

        Set the orientation of the body. The rotation matrix must be
        given as a sequence of 9 floats which are the elements of the
        matrix in row-major order.

        @param R: Rotation matrix
        @type R: 9-sequence of floats
        """
        cdef dMatrix3 m
        m[0] = R[0]
        m[1] = R[1]
        m[2] = R[2]
        m[3] = 0
        m[4] = R[3]
        m[5] = R[4]
        m[6] = R[5]
        m[7] = 0
        m[8] = R[6]
        m[9] = R[7]
        m[10] = R[8]
        m[11] = 0
        dBodySetRotation(self.bid, m)

    # getRotation
    def getRotation(self):
        """getRotation() -> 9-tuple

        Return the current rotation matrix as a tuple of 9 floats (row-major
        order).
        """
        cdef dReal* m
        # The "const" in the original return value is cast away
        m = <dReal*>dBodyGetRotation(self.bid)
        return m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10]

    # getQuaternion
    def getQuaternion(self):
        """getQuaternion() -> 4-tuple

        Return the current rotation as a quaternion. The return value
        is a list of 4 floats.
        """
        cdef dReal* q
        q = <dReal*>dBodyGetQuaternion(self.bid)
        return q[0], q[1], q[2], q[3]

    # setQuaternion
    def setQuaternion(self, q):
        """setQuaternion(q)

        Set the orientation of the body. The quaternion must be given as a
        sequence of 4 floats.

        @param q: Quaternion
        @type q: 4-sequence of floats
        """
        cdef dQuaternion w
        w[0] = q[0]
        w[1] = q[1]
        w[2] = q[2]
        w[3] = q[3]
        dBodySetQuaternion(self.bid, w)

    # setLinearVel
    def setLinearVel(self, vel):
        """setLinearVel(vel)

        Set the linear velocity of the body.

        @param vel: New velocity
        @type vel: 3-sequence of floats
        """
        dBodySetLinearVel(self.bid, vel[0], vel[1], vel[2])

    # getLinearVel
    def getLinearVel(self):
        """getLinearVel() -> 3-tuple

        Get the current linear velocity of the body.
        """
        cdef dReal* p
        # The "const" in the original return value is cast away
        p = <dReal*>dBodyGetLinearVel(self.bid)
        return p[0], p[1], p[2]

    # setAngularVel
    def setAngularVel(self, vel):
        """setAngularVel(vel)

        Set the angular velocity of the body.

        @param vel: New angular velocity
        @type vel: 3-sequence of floats
        """
        dBodySetAngularVel(self.bid, vel[0], vel[1], vel[2])

    # getAngularVel
    def getAngularVel(self):
        """getAngularVel() -> 3-tuple

        Get the current angular velocity of the body.
        """
        cdef dReal* p
        # The "const" in the original return value is cast away
        p = <dReal*>dBodyGetAngularVel(self.bid)
        return p[0], p[1], p[2]
    
    # setMass
    def setMass(self, Mass mass):
        """setMass(mass)

        Set the mass properties of the body. The argument mass must be
        an instance of a Mass object.

        @param mass: Mass properties
        @type mass: Mass
        """
        dBodySetMass(self.bid, &mass._mass)

    # getMass
    def getMass(self):
        """getMass() -> mass

        Return the mass properties as a Mass object.
        """
        cdef Mass m
        m = Mass()
        dBodyGetMass(self.bid, &m._mass)
        return m

    # addForce
    def addForce(self, f):
        """addForce(f)

        Add an external force f given in absolute coordinates. The force
        is applied at the center of mass.

        @param f: Force
        @type f: 3-sequence of floats
        """
        dBodyAddForce(self.bid, f[0], f[1], f[2])

    # addTorque
    def addTorque(self, t):
        """addTorque(t)

        Add an external torque t given in absolute coordinates.

        @param t: Torque
        @type t: 3-sequence of floats
        """
        dBodyAddTorque(self.bid, t[0], t[1], t[2])

    # addRelForce
    def addRelForce(self, f):
        """addRelForce(f)

        Add an external force f given in relative coordinates
        (relative to the body's own frame of reference). The force
        is applied at the center of mass.

        @param f: Force
        @type f: 3-sequence of floats
        """
        dBodyAddRelForce(self.bid, f[0], f[1], f[2])

    # addRelTorque
    def addRelTorque(self, t):
        """addRelTorque(t)

        Add an external torque t given in relative coordinates
        (relative to the body's own frame of reference).

        @param t: Torque
        @type t: 3-sequence of floats
        """
        dBodyAddRelTorque(self.bid, t[0], t[1], t[2])

    # addForceAtPos
    def addForceAtPos(self, f, p):
        """addForceAtPos(f, p)

        Add an external force f at position p. Both arguments must be
        given in absolute coordinates.

        @param f: Force
        @param p: Position
        @type f: 3-sequence of floats
        @type p: 3-sequence of floats
        """
        dBodyAddForceAtPos(self.bid, f[0], f[1], f[2], p[0], p[1], p[2])

    # addForceAtRelPos
    def addForceAtRelPos(self, f, p):
        """addForceAtRelPos(f, p)

        Add an external force f at position p. f is given in absolute
        coordinates and p in absolute coordinates.

        @param f: Force
        @param p: Position
        @type f: 3-sequence of floats
        @type p: 3-sequence of floats
        """
        dBodyAddForceAtRelPos(self.bid, f[0], f[1], f[2], p[0], p[1], p[2])

    # addRelForceAtPos
    def addRelForceAtPos(self, f, p):
        """addRelForceAtPos(f, p)

        Add an external force f at position p. f is given in relative
        coordinates and p in relative coordinates.

        @param f: Force
        @param p: Position
        @type f: 3-sequence of floats
        @type p: 3-sequence of floats
        """
        dBodyAddRelForceAtPos(self.bid, f[0], f[1], f[2], p[0], p[1], p[2])

    # addRelForceAtRelPos
    def addRelForceAtRelPos(self, f, p):
        """addRelForceAtRelPos(f, p)

        Add an external force f at position p. Both arguments must be
        given in relative coordinates.

        @param f: Force
        @param p: Position
        @type f: 3-sequence of floats
        @type p: 3-sequence of floats
        """
        dBodyAddRelForceAtRelPos(self.bid, f[0], f[1], f[2], p[0], p[1], p[2])

    # getForce
    def getForce(self):
        """getForce() -> 3-tuple

        Return the current accumulated force.
        """
        cdef dReal* f
        # The "const" in the original return value is cast away
        f = <dReal*>dBodyGetForce(self.bid)
        return f[0], f[1], f[2]

    # getTorque
    def getTorque(self):
        """getTorque() -> 3-tuple

        Return the current accumulated torque.
        """
        cdef dReal* f
        # The "const" in the original return value is cast away
        f = <dReal*>dBodyGetTorque(self.bid)
        return f[0], f[1], f[2]

    # setForce
    def setForce(self, f):
        """setForce(f)

        Set the body force accumulation vector.

        @param f: Force
        @type f: 3-tuple of floats
        """
        dBodySetForce(self.bid, f[0], f[1], f[2])

    # setTorque
    def setTorque(self, t):
        """setTorque(t)

        Set the body torque accumulation vector.

        @param t: Torque
        @type t: 3-tuple of floats
        """
        dBodySetTorque(self.bid, t[0], t[1], t[2])

    # getRelPointPos
    def getRelPointPos(self, p):
        """getRelPointPos(p) -> 3-tuple

        Utility function that takes a point p on a body and returns
        that point's position in global coordinates. The point p
        must be given in body relative coordinates.

        @param p: Body point (local coordinates)
        @type p: 3-sequence of floats
        """

        cdef dVector3 res
        dBodyGetRelPointPos(self.bid, p[0], p[1], p[2], res)
        return res[0], res[1], res[2]

    # getRelPointVel
    def getRelPointVel(self, p):
        """getRelPointVel(p) -> 3-tuple

        Utility function that takes a point p on a body and returns
        that point's velocity in global coordinates. The point p
        must be given in body relative coordinates.

        @param p: Body point (local coordinates)
        @type p: 3-sequence of floats
        """
        cdef dVector3 res
        dBodyGetRelPointVel(self.bid, p[0], p[1], p[2], res)
        return res[0], res[1], res[2]

    # getPointVel
    def getPointVel(self, p):
        """getPointVel(p) -> 3-tuple

        Utility function that takes a point p on a body and returns
        that point's velocity in global coordinates. The point p
        must be given in global coordinates.

        @param p: Body point (global coordinates)
        @type p: 3-sequence of floats
        """
        cdef dVector3 res
        dBodyGetPointVel(self.bid, p[0], p[1], p[2], res)
        return res[0], res[1], res[2]

    # getPosRelPoint
    def getPosRelPoint(self, p):
        """getPosRelPoint(p) -> 3-tuple

        This is the inverse of getRelPointPos(). It takes a point p in
        global coordinates and returns the point's position in
        body-relative coordinates.

        @param p: Body point (global coordinates)
        @type p: 3-sequence of floats
        """
        cdef dVector3 res
        dBodyGetPosRelPoint(self.bid, p[0], p[1], p[2], res)
        return res[0], res[1], res[2]

    # vectorToWorld
    def vectorToWorld(self, v):
        """vectorToWorld(v) -> 3-tuple

        Given a vector v expressed in the body coordinate system, rotate
        it to the world coordinate system.

        @param v: Vector in body coordinate system
        @type v: 3-sequence of floats
        """
        cdef dVector3 res
        dBodyVectorToWorld(self.bid, v[0], v[1], v[2], res)
        return res[0], res[1], res[2]

    # vectorFromWorld
    def vectorFromWorld(self, v):
        """vectorFromWorld(v) -> 3-tuple

        Given a vector v expressed in the world coordinate system, rotate
        it to the body coordinate system.

        @param v: Vector in world coordinate system
        @type v: 3-sequence of floats
        """
        cdef dVector3 res
        dBodyVectorFromWorld(self.bid, v[0], v[1], v[2], res)
        return res[0], res[1], res[2]

    # Enable
    def enable(self):
        """enable()

        Manually enable a body.
        """
        dBodyEnable(self.bid)
        
    # Disable
    def disable(self):
        """disable()

        Manually disable a body. Note that a disabled body that is connected
        through a joint to an enabled body will be automatically re-enabled
        at the next simulation step.
        """
        dBodyDisable(self.bid)
        
    # isEnabled
    def isEnabled(self):
        """isEnabled() -> bool

        Check if a body is currently enabled.
        """
        return dBodyIsEnabled(self.bid)
        
    # setFiniteRotationMode
    def setFiniteRotationMode(self, mode):
        """setFiniteRotationMode(mode)

        This function controls the way a body's orientation is updated at
        each time step. The mode argument can be:
        
         - 0: An "infinitesimal" orientation update is used. This is
           fast to compute, but it can occasionally cause inaccuracies
           for bodies that are rotating at high speed, especially when
           those bodies are joined to other bodies. This is the default
           for every new body that is created.
        
         - 1: A "finite" orientation update is used. This is more
           costly to compute, but will be more accurate for high speed
           rotations. Note however that high speed rotations can result
           in many types of error in a simulation, and this mode will
           only fix one of those sources of error.

        @param mode: Rotation mode (0/1)
        @type mode: int
        """
        dBodySetFiniteRotationMode(self.bid, mode)
        
    # getFiniteRotationMode
    def getFiniteRotationMode(self):
        """getFiniteRotationMode() -> mode (0/1)

        Return the current finite rotation mode of a body (0 or 1).
        See setFiniteRotationMode().
        """
        return dBodyGetFiniteRotationMode(self.bid)

    # setFiniteRotationAxis
    def setFiniteRotationAxis(self, a):
        """setFiniteRotationAxis(a)

        Set the finite rotation axis of the body.  This axis only has a
        meaning when the finite rotation mode is set
        (see setFiniteRotationMode()).
        
        @param a: Axis
        @type a: 3-sequence of floats
        """
        dBodySetFiniteRotationAxis(self.bid, a[0], a[1], a[2])

    # getFiniteRotationAxis
    def getFiniteRotationAxis(self):
        """getFiniteRotationAxis() -> 3-tuple

        Return the current finite rotation axis of the body.
        """
        cdef dVector3 p
        # The "const" in the original return value is cast away
        dBodyGetFiniteRotationAxis(self.bid, p)
        return p[0], p[1], p[2]
        
    # getNumJoints
    def getNumJoints(self):
        """getNumJoints() -> int

        Return the number of joints that are attached to this body.
        """
        return dBodyGetNumJoints(self.bid)

    # setGravityMode
    def setGravityMode(self, mode):
        """setGravityMode(mode)

        Set whether the body is influenced by the world's gravity
        or not. If mode is True it is, otherwise it isn't.
        Newly created bodies are always influenced by the world's gravity.

        @param mode: Gravity mode
        @type mode: bool
        """
        dBodySetGravityMode(self.bid, mode)
        
    # getGravityMode
    def getGravityMode(self):
        """getGravityMode() -> bool

        Return True if the body is influenced by the world's gravity.
        """
        return dBodyGetGravityMode(self.bid)

    def setDynamic(self):
        """setDynamic()

        Set a body to the (default) "dynamic" state, instead of "kinematic".
        See setKinematic() for more information.
        """
        dBodySetDynamic(self.bid)

    def setKinematic(self):
        """setKinematic()

        Set the kinematic state of the body (change it into a kinematic body)

        Kinematic bodies behave as if they had infinite mass. This means they don't react
        to any force (gravity, constraints or user-supplied); they simply follow 
        velocity to reach the next position. [from ODE wiki]

        """
        dBodySetKinematic(self.bid)

    def isKinematic(self):
        """isKinematic() -> bool

        Return True if the body is kinematic (not influenced by other forces).

        Kinematic bodies behave as if they had infinite mass. This means they don't react
        to any force (gravity, constraints or user-supplied); they simply follow
        velocity to reach the next position. [from ODE wiki]

        """
        return dBodyIsKinematic(self.bid)

    def setMaxAngularSpeed(self, max_speed):
        """setMaxAngularSpeed(max_speed)

        You can also limit the maximum angular speed. In contrast to the damping
        functions, the angular velocity is affected before the body is moved.
        This means that it will introduce errors in joints that are forcing the
        body to rotate too fast. Some bodies have naturally high angular
        velocities (like cars' wheels), so you may want to give them a very high
        (like the default, dInfinity) limit.

        """
        dBodySetMaxAngularSpeed(self.bid, max_speed)


# JointGroup
cdef class JointGroup:
    """Joint group.

    Constructor::
    
      JointGroup()
    """

    # JointGroup ID
    cdef dJointGroupID gid
    # A list of Python joints that were added to the group
    cdef object jointlist

    def __cinit__(self):
        self.gid = dJointGroupCreate(0)

    def __init__(self):
        self.jointlist = []

    def __dealloc__(self):
        if self.gid != NULL:
            for j in self.jointlist:
                j._destroyed()
            dJointGroupDestroy(self.gid)

    # empty
    def empty(self):
        """empty()

        Destroy all joints in the group.
        """
        dJointGroupEmpty(self.gid)
        for j in self.jointlist:
            j._destroyed()
        self.jointlist = []

    def _addjoint(self, j):
        """_addjoint(j)

        Add a joint to the group.  This is an internal method that is
        called by the joints.  The group has to know the Python
        wrappers because it has to notify them when the group is
        emptied (so that the ODE joints won't get destroyed
        twice). The notification is done by calling _destroyed() on
        the Python joints.

        @param j: The joint to add
        @type j: Joint
        """
        self.jointlist.append(j)


######################################################################

# Joint
cdef class Joint:
    """Base class for all joint classes."""

    # Joint id as returned by dJointCreateXxx()
    cdef dJointID jid
    # A reference to the world so that the world won't be destroyed while
    # there are still joints using it.
    cdef object world
    # The feedback buffer
    cdef dJointFeedback* feedback

    cdef object body1
    cdef object body2

    # A dictionary with user attributes
    # (set via __getattr__ and __setattr__)
    cdef object userattribs

    def __cinit__(self, *a, **kw):
        self.jid = NULL
        self.world = None
        self.feedback = NULL
        self.body1 = None
        self.body2 = None
        self.userattribs = {}

    def __init__(self, *a, **kw):
        raise NotImplementedError("Joint base class can't be used directly")

    def __dealloc__(self):
        self.setFeedback(False)
        if self.jid != NULL:
            dJointDestroy(self.jid)

    def __getattr__(self, name):
        try:
            return self.userattribs[name]
        except:
            raise AttributeError("Joint object has no attribute '%s'" % name)
            
    def __setattr__(self, name, value):
        self.userattribs[name] = value

    def __delattr__(self, name):
        try:
            del self.userattribs[name]
        except:
            raise AttributeError("Joint object has no attribute '%s'" % name)

    # _destroyed
    def _destroyed(self):
        """Notify the joint object about an external destruction of the ODE joint.

        This method has to be called when the underlying ODE object
        was destroyed by someone else (e.g. by a joint group). The Python
        wrapper will then refrain from destroying it again.
        """
        self.jid = NULL

    # enable
    def enable(self):
        """enable()

        Enable the joint. Disabled joints are completely ignored during the
        simulation. Disabled joints don't lose the already computed information
        like anchors and axes.
        """
        dJointEnable(self.jid)

    # disable
    def disable(self):
        """disable()

        Disable the joint. Disabled joints are completely ignored during the
        simulation. Disabled joints don't lose the already computed information
        like anchors and axes.
        """
        dJointDisable(self.jid)

    # isEnabled
    def isEnabled(self):
        """isEnabled() -> bool

        Determine whether the joint is enabled. Disabled joints are completely
        ignored during the simulation. Disabled joints don't lose the already
        computed information like anchors and axes.
        """
        return dJointIsEnabled(self.jid)

    # attach
    def attach(self, Body body1, Body body2):
        """attach(body1, body2)

        Attach the joint to some new bodies. A body can be attached
        to the environment by passing None as second body.
        
        @param body1: First body
        @param body2: Second body
        @type body1: Body
        @type body2: Body
        """
        cdef dBodyID id1, id2

        if body1 == None:
            id1 = NULL
        else:
            id1 = body1.bid
            
        if body2 == None:
            id2 = NULL
        else:
            id2 = body2.bid

        self.body1 = body1
        self.body2 = body2
        dJointAttach(self.jid, id1, id2)

    # getBody
    def getBody(self, index):
        """getBody(index) -> Body

        Return the bodies that this joint connects. If index is 0 the
        "first" body will be returned, corresponding to the body1
        argument of the attach() method. If index is 1 the "second" body
        will be returned, corresponding to the body2 argument of the
        attach() method.

        @param index: Bodx index (0 or 1).
        @type index: int
        """
        
        if index == 0:
            return self.body1
        elif index == 1:
            return self.body2
        else:
            raise IndexError()

    # setFeedback
    def setFeedback(self, flag=1):
        """setFeedback(flag=True)

        Create a feedback buffer. If flag is True then a buffer is
        allocated and the forces/torques applied by the joint can
        be read using the getFeedback() method. If flag is False the
        buffer is released.

        @param flag: Specifies whether a buffer should be created or released
        @type flag: bool
        """
        
        if flag:
            # Was there already a buffer allocated? then we're finished
            if self.feedback != NULL:
                return
            # Allocate a buffer and pass it to ODE
            self.feedback = <dJointFeedback*>malloc(sizeof(dJointFeedback))
            if self.feedback == NULL:
                raise MemoryError("can't allocate feedback buffer")
            dJointSetFeedback(self.jid, self.feedback)
        else:
            if self.feedback != NULL:
                # Free a previously allocated buffer
                dJointSetFeedback(self.jid, NULL)
                free(self.feedback)
                self.feedback = NULL
        
    # getFeedback
    def getFeedback(self):
        """getFeedback() -> (force1, torque1, force2, torque2)

        Get the forces/torques applied by the joint. If feedback is
        activated (i.e. setFeedback(True) was called) then this method
        returns a tuple (force1, torque1, force2, torque2) with the
        forces and torques applied to body 1 and body 2.  The
        forces/torques are given as 3-tuples.

        If feedback is deactivated then the method always returns None.
        """
        cdef dJointFeedback* fb
        
        fb = dJointGetFeedback(self.jid)
        if fb == NULL:
            return None
           
        f1 = (fb.f1[0], fb.f1[1], fb.f1[2])
        t1 = (fb.t1[0], fb.t1[1], fb.t1[2])
        f2 = (fb.f2[0], fb.f2[1], fb.f2[2])
        t2 = (fb.t2[0], fb.t2[1], fb.t2[2])
        return f1, t1, f2, t2

######################################################################


# BallJoint
cdef class BallJoint(Joint):
    """Ball joint.

    Constructor::
    
      BallJoint(world, jointgroup=None)
    """

    def __cinit__(self, World world not None, jointgroup=None):
        cdef JointGroup jg
        cdef dJointGroupID jgid

        jgid = NULL
        if jointgroup != None:
            jg = jointgroup
            jgid = jg.gid
        self.jid = dJointCreateBall(world.wid, jgid)

    def __init__(self, World world not None, jointgroup=None):
        self.world = world
        if jointgroup != None:
            jointgroup._addjoint(self)
            
    # setAnchor
    def setAnchor(self, pos):
        """setAnchor(pos)

        Set the joint anchor point which must be specified in world
        coordinates.

        @param pos: Anchor position
        @type pos: 3-sequence of floats
        """
        dJointSetBallAnchor(self.jid, pos[0], pos[1], pos[2])
    
    # getAnchor
    def getAnchor(self):
        """getAnchor() -> 3-tuple of floats

        Get the joint anchor point, in world coordinates.  This
        returns the point on body 1.  If the joint is perfectly
        satisfied, this will be the same as the point on body 2.
        """
        
        cdef dVector3 p
        dJointGetBallAnchor(self.jid, p)
        return p[0], p[1], p[2]

    # getAnchor2
    def getAnchor2(self):
        """getAnchor2() -> 3-tuple of floats

        Get the joint anchor point, in world coordinates.  This
        returns the point on body 2. If the joint is perfectly
        satisfied, this will be the same as the point on body 1.
        """

        cdef dVector3 p
        dJointGetBallAnchor2(self.jid, p)
        return p[0], p[1], p[2]

    # setParam
    def setParam(self, param, value):
        pass

    # getParam
    def getParam(self, param):
        return 0.0
        
    
# HingeJoint
cdef class HingeJoint(Joint):
    """Hinge joint.

    Constructor::
    
      HingeJoint(world, jointgroup=None)
    """

    def __cinit__(self, World world not None, jointgroup=None):
        cdef JointGroup jg
        cdef dJointGroupID jgid
        
        jgid = NULL
        if jointgroup != None:
            jg = jointgroup
            jgid = jg.gid
        self.jid = dJointCreateHinge(world.wid, jgid)
        
    def __init__(self, World world not None, jointgroup=None):
        self.world = world
        if jointgroup != None:
            jointgroup._addjoint(self)

    # setAnchor
    def setAnchor(self, pos):
        """setAnchor(pos)

        Set the hinge anchor which must be given in world coordinates.

        @param pos: Anchor position
        @type pos: 3-sequence of floats
        """
        dJointSetHingeAnchor(self.jid, pos[0], pos[1], pos[2])
    
    # getAnchor
    def getAnchor(self):
        """getAnchor() -> 3-tuple of floats

        Get the joint anchor point, in world coordinates. This returns
        the point on body 1. If the joint is perfectly satisfied, this
        will be the same as the point on body 2.
        """
        cdef dVector3 p
        dJointGetHingeAnchor(self.jid, p)
        return p[0], p[1], p[2]

    # getAnchor2
    def getAnchor2(self):
        """getAnchor2() -> 3-tuple of floats

        Get the joint anchor point, in world coordinates. This returns
        the point on body 2. If the joint is perfectly satisfied, this
        will be the same as the point on body 1.
        """
        cdef dVector3 p
        dJointGetHingeAnchor2(self.jid, p)
        return p[0], p[1], p[2]

    # setAxis
    def setAxis(self, axis):
        """setAxis(axis)

        Set the hinge axis.

        @param axis: Hinge axis
        @type axis: 3-sequence of floats
        """
        dJointSetHingeAxis(self.jid, axis[0], axis[1], axis[2])
    
    # getAxis
    def getAxis(self):
        """getAxis() -> 3-tuple of floats

        Get the hinge axis.
        """
        cdef dVector3 a
        dJointGetHingeAxis(self.jid, a)
        return a[0], a[1], a[2]

    # getAngle
    def getAngle(self):
        """getAngle() -> float

        Get the hinge angle. The angle is measured between the two
        bodies, or between the body and the static environment. The
        angle will be between -pi..pi.

        When the hinge anchor or axis is set, the current position of
        the attached bodies is examined and that position will be the
        zero angle.
        """
        
        return dJointGetHingeAngle(self.jid)

    # getAngleRate
    def getAngleRate(self):
        """getAngleRate() -> float

        Get the time derivative of the angle.
        """
        return dJointGetHingeAngleRate(self.jid)

    # addTorque
    def addTorque(self, torque):
        """addTorque(torque)

        Applies the torque about the hinge axis.

        @param torque: Torque magnitude
        @type torque: float
        """
        dJointAddHingeTorque(self.jid, torque)

    # setParam
    def setParam(self, param, value):
        """setParam(param, value)

        Set limit/motor parameters for the joint.

        param is one of ParamLoStop, ParamHiStop, ParamVel, ParamFMax,
        ParamFudgeFactor, ParamBounce, ParamCFM, ParamStopERP, ParamStopCFM,
        ParamSuspensionERP, ParamSuspensionCFM.

        These parameter names can be optionally followed by a digit (2
        or 3) to indicate the second or third set of parameters.

        @param param: Selects the parameter to set
        @param value: Parameter value
        @type param: int
        @type value: float
        """
        
        dJointSetHingeParam(self.jid, param, value)

    # getParam
    def getParam(self, param):
        """getParam(param) -> float

        Get limit/motor parameters for the joint.

        param is one of ParamLoStop, ParamHiStop, ParamVel, ParamFMax,
        ParamFudgeFactor, ParamBounce, ParamCFM, ParamStopERP, ParamStopCFM,
        ParamSuspensionERP, ParamSuspensionCFM.

        These parameter names can be optionally followed by a digit (2
        or 3) to indicate the second or third set of parameters.

        @param param: Selects the parameter to read
        @type param: int
        """
        return dJointGetHingeParam(self.jid, param)
        
        
# SliderJoint
cdef class SliderJoint(Joint):
    """Slider joint.
    
    Constructor::
    
      SlideJoint(world, jointgroup=None)
    """

    def __cinit__(self, World world not None, jointgroup=None):
        cdef JointGroup jg
        cdef dJointGroupID jgid

        jgid = NULL
        if jointgroup != None:
            jg = jointgroup
            jgid = jg.gid
        self.jid = dJointCreateSlider(world.wid, jgid)

    def __init__(self, World world not None, jointgroup=None):
        self.world = world
        if jointgroup != None:
            jointgroup._addjoint(self)
          
    # setAxis
    def setAxis(self, axis):
        """setAxis(axis)

        Set the slider axis parameter.

        @param axis: Slider axis
        @type axis: 3-sequence of floats
        """
        dJointSetSliderAxis(self.jid, axis[0], axis[1], axis[2])
    
    # getAxis
    def getAxis(self):
        """getAxis() -> 3-tuple of floats

        Get the slider axis parameter.
        """
        cdef dVector3 a
        dJointGetSliderAxis(self.jid, a)
        return a[0], a[1], a[2]

    # getPosition
    def getPosition(self):
        """getPosition() -> float

        Get the slider linear position (i.e. the slider's "extension").

        When the axis is set, the current position of the attached
        bodies is examined and that position will be the zero
        position.
        """
        
        return dJointGetSliderPosition(self.jid)

    # getPositionRate
    def getPositionRate(self):
        """getPositionRate() -> float

        Get the time derivative of the position.
        """
        return dJointGetSliderPositionRate(self.jid)

    # addForce
    def addForce(self, force):
        """addForce(force)

        Applies the given force in the slider's direction.

        @param force: Force magnitude
        @type force: float
        """
        dJointAddSliderForce(self.jid, force)

    # setParam
    def setParam(self, param, value):
        dJointSetSliderParam(self.jid, param, value)

    # getParam
    def getParam(self, param):
        return dJointGetSliderParam(self.jid, param)
        
    
# UniversalJoint
cdef class UniversalJoint(Joint):
    """Universal joint.

    Constructor::
    
      UniversalJoint(world, jointgroup=None)
    """

    def __cinit__(self, World world not None, jointgroup=None):
        cdef JointGroup jg
        cdef dJointGroupID jgid

        jgid = NULL
        if jointgroup != None:
            jg = jointgroup
            jgid = jg.gid
        self.jid = dJointCreateUniversal(world.wid, jgid)

    def __init__(self, World world not None, jointgroup=None):
        self.world = world
        if jointgroup != None:
            jointgroup._addjoint(self)

    # setAnchor
    def setAnchor(self, pos):
        """setAnchor(pos)

        Set the universal anchor.

        @param pos: Anchor position
        @type pos: 3-sequence of floats
        """
        dJointSetUniversalAnchor(self.jid, pos[0], pos[1], pos[2])
    
    # getAnchor
    def getAnchor(self):
        """getAnchor() -> 3-tuple of floats

        Get the joint anchor point, in world coordinates. This returns
        the point on body 1. If the joint is perfectly satisfied, this
        will be the same as the point on body 2.
        """
        
        cdef dVector3 p
        dJointGetUniversalAnchor(self.jid, p)
        return p[0], p[1], p[2]

    # getAnchor2
    def getAnchor2(self):
        """getAnchor2() -> 3-tuple of floats

        Get the joint anchor point, in world coordinates. This returns
        the point on body 2. If the joint is perfectly satisfied, this
        will be the same as the point on body 1.
        """
        
        cdef dVector3 p
        dJointGetUniversalAnchor2(self.jid, p)
        return p[0], p[1], p[2]

    # setAxis1
    def setAxis1(self, axis):
        """setAxis1(axis)

        Set the first universal axis. Axis 1 and axis 2 should be
        perpendicular to each other.

        @param axis: Joint axis
        @type axis: 3-sequence of floats
        """
        dJointSetUniversalAxis1(self.jid, axis[0], axis[1], axis[2])
    
    # getAxis1
    def getAxis1(self):
        """getAxis1() -> 3-tuple of floats

        Get the first univeral axis.
        """
        cdef dVector3 a
        dJointGetUniversalAxis1(self.jid, a)
        return a[0], a[1], a[2]

    # setAxis2
    def setAxis2(self, axis):
        """setAxis2(axis)

        Set the second universal axis. Axis 1 and axis 2 should be
        perpendicular to each other.

        @param axis: Joint axis
        @type axis: 3-sequence of floats
        """
        dJointSetUniversalAxis2(self.jid, axis[0], axis[1], axis[2])
    
    # getAxis2
    def getAxis2(self):
        """getAxis2() -> 3-tuple of floats

        Get the second univeral axis.
        """
        cdef dVector3 a
        dJointGetUniversalAxis2(self.jid, a)
        return a[0], a[1], a[2]

    # addTorques
    def addTorques(self, torque1, torque2):
        """addTorques(torque1, torque2)

        Applies torque1 about axis 1, and torque2 about axis 2.

        @param torque1: Torque 1 magnitude
        @param torque2: Torque 2 magnitude
        @type torque1: float
        @type torque2: float
        """
        dJointAddUniversalTorques(self.jid, torque1, torque2)

    def getAngle1(self):
        return dJointGetUniversalAngle1(self.jid)

    def getAngle2(self):
        return dJointGetUniversalAngle2(self.jid)
    
    def getAngle1Rate(self):
        return dJointGetUniversalAngle1Rate(self.jid)

    def getAngle2Rate(self):
        return dJointGetUniversalAngle2Rate(self.jid)

    # setParam
    def setParam(self, param, value):
        dJointSetUniversalParam(self.jid, param, value)

    # getParam
    def getParam(self, param):
        return dJointGetUniversalParam(self.jid, param)

    
# Hinge2Joint
cdef class Hinge2Joint(Joint):
    """Hinge2 joint.

    Constructor::
    
      Hinge2Joint(world, jointgroup=None)
    """

    def __cinit__(self, World world not None, jointgroup=None):
        cdef JointGroup jg
        cdef dJointGroupID jgid

        jgid = NULL
        if jointgroup != None:
            jg = jointgroup
            jgid = jg.gid
        self.jid = dJointCreateHinge2(world.wid, jgid)

    def __init__(self, World world, jointgroup=None):
        self.world = world
        if jointgroup != None:
            jointgroup._addjoint(self)

    # setAnchor
    def setAnchor(self, pos):
        """setAnchor(pos)

        Set the hinge-2 anchor.

        @param pos: Anchor position
        @type pos: 3-sequence of floats
        """
        dJointSetHinge2Anchor(self.jid, pos[0], pos[1], pos[2])
    
    # getAnchor
    def getAnchor(self):
        """getAnchor() -> 3-tuple of floats

        Get the joint anchor point, in world coordinates. This returns
        the point on body 1. If the joint is perfectly satisfied, this
        will be the same as the point on body 2.
        """
        
        cdef dVector3 p
        dJointGetHinge2Anchor(self.jid, p)
        return p[0], p[1], p[2]

    # getAnchor2
    def getAnchor2(self):
        """getAnchor2() -> 3-tuple of floats

        Get the joint anchor point, in world coordinates. This returns
        the point on body 2. If the joint is perfectly satisfied, this
        will be the same as the point on body 1.
        """
        
        cdef dVector3 p
        dJointGetHinge2Anchor2(self.jid, p)
        return p[0], p[1], p[2]

    # setAxis1
    def setAxis1(self, axis):
        """setAxis1(axis)

        Set the first hinge-2 axis. Axis 1 and axis 2 must not lie
        along the same line.

        @param axis: Joint axis
        @type axis: 3-sequence of floats
        """
        
        dJointSetHinge2Axis1(self.jid, axis[0], axis[1], axis[2])
    
    # getAxis1
    def getAxis1(self):
        """getAxis1() -> 3-tuple of floats

        Get the first hinge-2 axis.
        """
        cdef dVector3 a
        dJointGetHinge2Axis1(self.jid, a)
        return a[0], a[1], a[2]

    # setAxis2
    def setAxis2(self, axis):
        """setAxis2(axis)

        Set the second hinge-2 axis. Axis 1 and axis 2 must not lie
        along the same line.

        @param axis: Joint axis
        @type axis: 3-sequence of floats
        """
        dJointSetHinge2Axis2(self.jid, axis[0], axis[1], axis[2])
    
    # getAxis2
    def getAxis2(self):
        """getAxis2() -> 3-tuple of floats

        Get the second hinge-2 axis.
        """
        cdef dVector3 a
        dJointGetHinge2Axis2(self.jid, a)
        return a[0], a[1], a[2]

    # getAngle
    def getAngle1(self):
        """getAngle1() -> float

        Get the first hinge-2 angle (around axis 1).

        When the anchor or axis is set, the current position of the
        attached bodies is examined and that position will be the zero
        angle.
        """
        return dJointGetHinge2Angle1(self.jid)

    # getAngle1Rate
    def getAngle1Rate(self):
        """getAngle1Rate() -> float

        Get the time derivative of the first hinge-2 angle.
        """
        return dJointGetHinge2Angle1Rate(self.jid)

    # getAngle2Rate
    def getAngle2Rate(self):
        """getAngle2Rate() -> float

        Get the time derivative of the second hinge-2 angle.
        """
        return dJointGetHinge2Angle2Rate(self.jid)

    # addTorques
    def addTorques(self, torque1, torque2):
        """addTorques(torque1, torque2)

        Applies torque1 about axis 1, and torque2 about axis 2.

        @param torque1: Torque 1 magnitude
        @param torque2: Torque 2 magnitude
        @type torque1: float
        @type torque2: float
        """
        dJointAddHinge2Torques(self.jid, torque1, torque2)

    # setParam
    def setParam(self, param, value):
        dJointSetHinge2Param(self.jid, param, value)

    # getParam
    def getParam(self, param):
        return dJointGetHinge2Param(self.jid, param)

    
# FixedJoint
cdef class FixedJoint(Joint):
    """Fixed joint.

    Constructor::
    
      FixedJoint(world, jointgroup=None)
    """

    def __cinit__(self, World world not None, jointgroup=None):
        cdef JointGroup jg
        cdef dJointGroupID jgid

        jgid = NULL
        if jointgroup != None:
            jg = jointgroup
            jgid = jg.gid
        self.jid = dJointCreateFixed(world.wid, jgid)

    def __init__(self, World world not None, jointgroup=None):
        self.world = world
        if jointgroup != None:
            jointgroup._addjoint(self)

    # setFixed
    def setFixed(self):
        """setFixed()

        Call this on the fixed joint after it has been attached to
        remember the current desired relative offset and desired
        relative rotation between the bodies.
        """
        dJointSetFixed(self.jid)

        
# ContactJoint
cdef class ContactJoint(Joint):
    """Contact joint.

    Constructor::
    
      ContactJoint(world, jointgroup, contact)
    """

    def __cinit__(self, World world not None, jointgroup, Contact contact):
        cdef JointGroup jg
        cdef dJointGroupID jgid
        jgid = NULL
        if jointgroup != None:
            jg = jointgroup
            jgid = jg.gid
        self.jid = dJointCreateContact(world.wid, jgid, &contact._contact)

    def __init__(self, World world not None, jointgroup, Contact contact):
        self.world = world
        if jointgroup != None:
            jointgroup._addjoint(self)

# AMotor
cdef class AMotor(Joint):
    """AMotor joint.
    
    Constructor::
    
      AMotor(world, jointgroup=None)
    """

    def __cinit__(self, World world not None, jointgroup=None):
        cdef JointGroup jg
        cdef dJointGroupID jgid

        jgid = NULL
        if jointgroup != None:
            jg = jointgroup
            jgid = jg.gid
        self.jid = dJointCreateAMotor(world.wid, jgid)

    def __init__(self, World world not None, jointgroup=None):
        self.world = world
        if jointgroup != None:
            jointgroup._addjoint(self)
            
    # setMode
    def setMode(self, mode):
        """setMode(mode)

        Set the angular motor mode.  mode must be either AMotorUser or
        AMotorEuler.

        @param mode: Angular motor mode
        @type mode: int
        """
        dJointSetAMotorMode(self.jid, mode)

    # getMode
    def getMode(self):
        """getMode()

        Return the angular motor mode (AMotorUser or AMotorEuler).
        """
        return dJointGetAMotorMode(self.jid)

    # setNumAxes
    def setNumAxes(self, int num):
        """setNumAxes(num)

        Set the number of angular axes that will be controlled by the AMotor.
        num may be in the range from 0 to 3.

        @param num: Number of axes (0-3)
        @type num: int
        """
        dJointSetAMotorNumAxes(self.jid, num)

    # getNumAxes
    def getNumAxes(self):
        """getNumAxes() -> int

        Get the number of angular axes that are controlled by the AMotor.
        """
        return dJointGetAMotorNumAxes(self.jid)

    # setAxis
    def setAxis(self, int anum, int rel, axis):
        """setAxis(anum, rel, axis)

        Set an AMotor axis.

        The anum argument selects the axis to change (0,1 or 2).
        Each axis can have one of three "relative orientation" modes,
        selected by rel:
        
        0: The axis is anchored to the global frame.
        1: The axis is anchored to the first body.
        2: The axis is anchored to the second body.

        The axis vector is always specified in global coordinates
        regardless of the setting of rel.

        @param anum: Axis number
        @param rel: Relative orientation mode
        @param axis: Axis
        @type anum: int
        @type rel: int
        @type axis: 3-sequence of floats
        """
        dJointSetAMotorAxis(self.jid, anum, rel, axis[0], axis[1], axis[2])

    # getAxis
    def getAxis(self, int anum):
        """getAxis(anum)

        Get an AMotor axis.

        @param anum: Axis index (0-2)
        @type anum: int
        """
        cdef dVector3 a
        dJointGetAMotorAxis(self.jid, anum, a)
        return a[0], a[1], a[2]

    # getAxisRel
    def getAxisRel(self, int anum):
        """getAxisRel(anum) -> int

        Get the relative mode of an axis.

        @param anum: Axis index (0-2)
        @type anum: int
        """
        return dJointGetAMotorAxisRel(self.jid, anum)

    # setAngle
    def setAngle(self, int anum, angle):
        """setAngle(anum, angle)

        Tell the AMotor what the current angle is along axis anum.

        @param anum: Axis index
        @param angle: Angle
        @type anum: int
        @type angle: float
        """
        dJointSetAMotorAngle(self.jid, anum, angle)

    # getAngle
    def getAngle(self, int anum):
        """getAngle(anum) -> float

        Return the current angle for axis anum.

        @param anum: Axis index
        @type anum: int
        """
        return dJointGetAMotorAngle(self.jid, anum)

    # getAngleRate
    def getAngleRate(self, int anum):
        """getAngleRate(anum) -> float

        Return the current angle rate for axis anum.

        @param anum: Axis index
        @type anum: int
        """
        return dJointGetAMotorAngleRate(self.jid, anum)

    # addTorques
    def addTorques(self, torque0, torque1, torque2):
        """addTorques(torque0, torque1, torque2)

        Applies torques about the AMotor's axes.

        @param torque0: Torque 0 magnitude
        @param torque1: Torque 1 magnitude
        @param torque2: Torque 2 magnitude
        @type torque0: float
        @type torque1: float
        @type torque2: float
        """
        dJointAddAMotorTorques(self.jid, torque0, torque1, torque2)

    # setParam
    def setParam(self, param, value):
        dJointSetAMotorParam(self.jid, param, value)

    # getParam
    def getParam(self, param):
        return dJointGetAMotorParam(self.jid, param)


# LMotor
cdef class LMotor(Joint):
    """LMotor joint.
    
    Constructor::
    
      LMotor(world, jointgroup=None)
    """

    def __cinit__(self, World world not None, jointgroup=None):
        cdef JointGroup jg
        cdef dJointGroupID jgid

        jgid = NULL
        if jointgroup != None:
            jg = jointgroup
            jgid = jg.gid
        self.jid = dJointCreateLMotor(world.wid, jgid)

    def __init__(self, World world not None, jointgroup=None):
        self.world = world
        if jointgroup != None:
            jointgroup._addjoint(self)
            
    # setNumAxes
    def setNumAxes(self, int num):
        """setNumAxes(num)

        Set the number of angular axes that will be controlled by the LMotor.
        num may be in the range from 0 to 3.

        @param num: Number of axes (0-3)
        @type num: int
        """
        dJointSetLMotorNumAxes(self.jid, num)

    # getNumAxes
    def getNumAxes(self):
        """getNumAxes() -> int

        Get the number of angular axes that are controlled by the LMotor.
        """
        return dJointGetLMotorNumAxes(self.jid)

    # setAxis
    def setAxis(self, int anum, int rel, axis):
        """setAxis(anum, rel, axis)

        Set an LMotor axis.

        The anum argument selects the axis to change (0,1 or 2).
        Each axis can have one of three "relative orientation" modes,
        selected by rel:

        0: The axis is anchored to the global frame.
        1: The axis is anchored to the first body.
        2: The axis is anchored to the second body.

        @param anum: Axis number
        @param rel: Relative orientation mode
        @param axis: Axis
        @type anum: int
        @type rel: int
        @type axis: 3-sequence of floats
        """
        dJointSetLMotorAxis(self.jid, anum, rel, axis[0], axis[1], axis[2])

    # getAxis
    def getAxis(self, int anum):
        """getAxis(anum)

        Get an LMotor axis.

        @param anum: Axis index (0-2)
        @type anum: int
        """
        cdef dVector3 a
        dJointGetLMotorAxis(self.jid, anum, a)
        return a[0], a[1], a[2]

    # setParam
    def setParam(self, param, value):
        dJointSetLMotorParam(self.jid, param, value)

    # getParam
    def getParam(self, param):
        return dJointGetLMotorParam(self.jid, param)


# Plane2DJoint
cdef class Plane2DJoint(Joint):
    """Plane-2D Joint.

    Constructor::
    
      Plane2DJoint(world, jointgroup=None)
    """

    def __cinit__(self, World world not None, jointgroup=None):
        cdef JointGroup jg
        cdef dJointGroupID jgid

        jgid = NULL
        if jointgroup != None:
            jg = jointgroup
            jgid = jg.gid
        self.jid = dJointCreatePlane2D(world.wid, jgid)

    def __init__(self, World world not None, jointgroup=None):
        self.world = world
        if jointgroup != None:
            jointgroup._addjoint(self)
            
    def setXParam(self, param, value):
        dJointSetPlane2DXParam(self.jid, param, value)
        
    def setYParam(self, param, value):
        dJointSetPlane2DYParam(self.jid, param, value)
        
    def setAngleParam(self, param, value):
        dJointSetPlane2DAngleParam(self.jid, param, value)


# Geom base class
cdef class GeomObject:
    """This is the abstract base class for all geom objects.
    """
    
    # The id of the geom object as returned by dCreateXxxx()
    cdef dGeomID gid
    # The space in which the geom was placed (or None). This reference
    # is kept so that the space won't be destroyed while there are still
    # geoms around that might use it.
    cdef object space
    # The body that the geom was attached to (or None).
    cdef object body

    # A dictionary with user defined attributes
    cdef object attribs

    cdef object __weakref__

    def __cinit__(self, *a, **kw):
        self.gid = NULL
        self.space = None
        self.body = None
        self.attribs = {}

    def __init__(self, *a, **kw):
        raise NotImplementedError(
            "GeomObject base class can't be used directly")

    def __dealloc__(self):
        if self.gid != NULL:
            dGeomDestroy(self.gid)
            self.gid = NULL

    def __getattr__(self, name):
        if name in self.attribs:
            return self.attribs[name]
        else:
            raise AttributeError("geom has no attribute '%s'" % name)

    def __setattr__(self, name, val):
        self.attribs[name] = val

    def __delattr__(self, name):
        if name in self.attribs:
            del self.attribs[name]
        else:
            raise AttributeError("geom has no attribute '%s'" % name)

    def _id(self):
        """_id() -> int

        Return the internal id of the geom (dGeomID) as returned by
        the dCreateXyz() functions.

        This method has to be overwritten in derived methods.
        """
        raise NotImplementedError("Bug: The _id() method is not implemented")

    def placeable(self):
        """placeable() -> bool

        Returns True if the geom object is a placeable geom.

        This method has to be overwritten in derived methods.
        """
        return False

    def setBody(self, Body body):
        """setBody(body)

        Set the body associated with a placeable geom.

        @param body: The Body object or None.
        @type body: Body
        """

        if not self.placeable():
            raise ValueError(
                "Non-placeable geoms cannot have a body associated to them")
        
        if body == None:
            dGeomSetBody(self.gid, NULL)
        else:
            dGeomSetBody(self.gid, body.bid)
        self.body = body

    def getBody(self):
        """getBody() -> Body

        Get the body associated with this geom.
        """
        if not self.placeable():
            return environment
        
        return self.body

    def setPosition(self, pos):
        """setPosition(pos)

        Set the position of the geom. If the geom is attached to a body,
        the body's position will also be changed.

        @param pos: Position
        @type pos: 3-sequence of floats
        """
        if not self.placeable():
            raise ValueError("Cannot set a position on non-placeable geoms")
        dGeomSetPosition(self.gid, pos[0], pos[1], pos[2])

    def getPosition(self):
        """getPosition() -> 3-tuple

        Get the current position of the geom. If the geom is attached to
        a body the returned value is the body's position.
        """
        if not self.placeable():
            raise ValueError("Non-placeable geoms do not have a position")

        cdef dReal* p
        p = <dReal*>dGeomGetPosition(self.gid)
        return p[0], p[1], p[2]

    def setRotation(self, R):
        """setRotation(R)

        Set the orientation of the geom. If the geom is attached to a body,
        the body's orientation will also be changed.

        @param R: Rotation matrix
        @type R: 9-sequence of floats
        """
        if not self.placeable():
            raise ValueError("Cannot set a rotation on non-placeable geoms")

        cdef dMatrix3 m
        m[0] = R[0]
        m[1] = R[1]
        m[2] = R[2]
        m[3] = 0
        m[4] = R[3]
        m[5] = R[4]
        m[6] = R[5]
        m[7] = 0
        m[8] = R[6]
        m[9] = R[7]
        m[10] = R[8]
        m[11] = 0
        dGeomSetRotation(self.gid, m)

    def getRotation(self):
        """getRotation() -> 9-tuple

        Get the current orientation of the geom. If the geom is attached to
        a body the returned value is the body's orientation.
        """
        if not self.placeable():
            raise ValueError("Non-placeable geoms do not have a rotation")

        cdef dReal* m
        m = <dReal*>dGeomGetRotation(self.gid)
        return [m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10]]

    def getQuaternion(self):
        """getQuaternion() -> (w,x,y,z)

        Get the current orientation of the geom. If the geom is attached to
        a body the returned value is the body's orientation.
        """
        if not self.placeable():
            raise ValueError("Non-placeable geoms do not have an orientation")

        cdef dQuaternion q
        dGeomGetQuaternion(self.gid, q)
        return q[0], q[1], q[2], q[3]

    def setQuaternion(self, q):
        """setQuaternion(q)

        Set the orientation of the geom. If the geom is attached to a body,
        the body's orientation will also be changed.

        @param q: Quaternion (w,x,y,z)
        @type q: 4-sequence of floats
        """
        if not self.placeable():
            raise ValueError("Cannot set a quaternion on non-placeable geoms")

        cdef dQuaternion cq
        cq[0] = q[0]
        cq[1] = q[1]
        cq[2] = q[2]
        cq[3] = q[3]
        dGeomSetQuaternion(self.gid, cq)

    def setOffsetPosition(self, pos):
        """setOffsetPosition(pos)

        Set the offset position of the geom. The geom must be attached to a
        body.  If the geom did not have an offset, it is automatically created.
        This sets up an additional (local) transformation for the geom, since
        geoms attached to a body share their global position and rotation.

        @param pos: Position
        @type pos: 3-sequence of floats
        """
        if self.body == None:
            raise ValueError("Cannot set an offset position on a geom before "
                             "calling setBody")
        dGeomSetOffsetPosition(self.gid, pos[0], pos[1], pos[2])

    def getOffsetPosition(self):
        """getOffsetPosition() -> 3-tuple

        Get the offset position of the geom.
        """
        cdef dReal* p
        p = <dReal*>dGeomGetOffsetPosition(self.gid)
        return (p[0],p[1],p[2])

    def setOffsetRotation(self, R):
        """setOffsetRotation(R)

        Set the offset rotation of the geom. The geom must be attached to a
        body.  If the geom did not have an offset, it is automatically created.
        This sets up an additional (local) transformation for the geom, since
        geoms attached to a body share their global position and rotation.

        @param R: Rotation matrix
        @type R: 9-sequence of floats
        """
        if self.body == None:
            raise ValueError("Cannot set an offset rotation on a geom before "
                             "calling setBody")

        cdef dMatrix3 m
        m[0] = R[0]
        m[1] = R[1]
        m[2] = R[2]
        m[3] = 0
        m[4] = R[3]
        m[5] = R[4]
        m[6] = R[5]
        m[7] = 0
        m[8] = R[6]
        m[9] = R[7]
        m[10] = R[8]
        m[11] = 0
        dGeomSetOffsetRotation(self.gid, m)

    def getOffsetRotation(self):
        """getOffsetRotation() -> 9-tuple

        Get the offset rotation of the geom.
        """
        cdef dReal* m
        m = <dReal*>dGeomGetOffsetRotation(self.gid)
        return [m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10]]

    def clearOffset(self):
        """clearOffset()

        Disable the offset transform of the geom.
        """
        dGeomClearOffset(self.gid)

    def getAABB(self):
        """getAABB() -> 6-tuple

        Return an axis aligned bounding box that surrounds the geom.
        The return value is a 6-tuple (minx, maxx, miny, maxy, minz, maxz).
        """
        cdef dReal aabb[6]
        
        dGeomGetAABB(self.gid, aabb)
        return aabb[0], aabb[1], aabb[2], aabb[3], aabb[4], aabb[5]

    def isSpace(self):
        """isSpace() -> bool

        Return 1 if the given geom is a space, or 0 if not."""
        return dGeomIsSpace(self.gid)

    def getSpace(self):
        """getSpace() -> Space

        Return the space that the given geometry is contained in,
        or return None if it is not contained in any space."""
        return self.space

    def setCollideBits(self, bits):
        """setCollideBits(bits)

        Set the "collide" bitfields for this geom.

        @param bits: Collide bit field
        @type bits: int/long
        """
        dGeomSetCollideBits(self.gid, long(bits))
        
    def setCategoryBits(self, bits):
        """setCategoryBits(bits)

        Set the "category" bitfields for this geom.

        @param bits: Category bit field
        @type bits: int/long
        """
        dGeomSetCategoryBits(self.gid, long(bits))

    def getCollideBits(self):
        """getCollideBits() -> long

        Return the "collide" bitfields for this geom.
        """
        return dGeomGetCollideBits(self.gid)

    def getCategoryBits(self):
        """getCategoryBits() -> long

        Return the "category" bitfields for this geom.
        """
        return dGeomGetCategoryBits(self.gid)
    
    def enable(self):
        """enable()

        Enable the geom."""
        dGeomEnable(self.gid)

    def disable(self):
        """disable()

        Disable the geom."""
        dGeomDisable(self.gid)

    def isEnabled(self):
        """isEnabled() -> bool

        Return True if the geom is enabled."""
        return dGeomIsEnabled(self.gid)


# _SpaceIterator
class _SpaceIterator:
    """Iterates over the geoms inside a Space.
    """

    def __init__(self, space):
        self.space = space
        self.idx = 0
        
    def __iter__(self):
        return self

    def next(self):
        if self.idx >= self.space.getNumGeoms():
            raise StopIteration
        else:
            res = self.space.getGeom(self.idx)
            self.idx = self.idx + 1
            return res


# SpaceBase
cdef class SpaceBase(GeomObject):
    """Space class (container for geometry objects).

    A Space object is a container for geometry objects which are used
    to do collision detection.
    The space does high level collision culling, which means that it
    can identify which pairs of geometry objects are potentially
    touching.

    This Space class can be used for both, a SimpleSpace and a HashSpace
    (see ODE documentation).

     >>> space = Space(type=0)   # Create a SimpleSpace
     >>> space = Space(type=1)   # Create a HashSpace
    """

    # The id of the space. Actually this is a copy of the value in self.gid
    # (as the Space is derived from GeomObject) which can be used without
    # casting whenever a *space* id is required.
    cdef dSpaceID sid
    
    # Dictionary with Geomobjects. Key is the ID (geom._id()) and the value
    # is the geom object (Python wrapper). This is used in collide_callback()
#    cdef object geom_dict

    def __cinit__(self, *a, **kw):
        pass

    def __init__(self, *a, **kw):
        raise NotImplementedError("The SpaceBase class can't be used directly")

    def __dealloc__(self):
        if self.gid != NULL:
            dSpaceDestroy(self.sid)
            self.sid = NULL
            self.gid = NULL

#    def _addgeom(self, geom):
#        """Insert the geom object into the dictionary (internal method).
#
#        This method has to called in the constructor of a geom object.
#        """
#        self.geom_dict[geom._id()]=geom

#    def _id2geom(self, id):
#        """Get the Python wrapper that corresponds to an ID.
#
#        The ID is the integer value, as returned by geom._id().
#        If the ID is unknown then None is returned.
#        """
#        if id in self.geom_dict:
#            return self.geom_dict[id]
#        else:
#            return None
       
    def _id(self):
        cdef long id
        id = <long>self.sid
        return id

    def __len__(self):
        return self.getNumGeoms()

    def __iter__(self):
        return _SpaceIterator(self)

    def add(self, GeomObject geom):
        """add(geom)

        Add a geom to a space. This does nothing if the geom is
        already in the space.

        @param geom: Geom object to add
        @type geom: GeomObject
        """
        
        dSpaceAdd(self.sid, geom.gid)

    def remove(self, GeomObject geom):
        """remove(geom)

        Remove a geom from a space.

        @param geom: Geom object to remove
        @type geom: GeomObject
        """
        dSpaceRemove(self.sid, geom.gid)

    def query(self, GeomObject geom):
        """query(geom) -> bool

        Return True if the given geom is in the space.

        @param geom: Geom object to check
        @type geom: GeomObject
        """
        return dSpaceQuery(self.sid, geom.gid)

    def getNumGeoms(self):
        """getNumGeoms() -> int

        Return the number of geoms contained within the space.
        """
        return dSpaceGetNumGeoms(self.sid)

    def getGeom(self, int idx):
        """getGeom(idx) -> GeomObject

        Return the geom with the given index contained within the space.

        @param idx: Geom index (0,1,...,getNumGeoms()-1)
        @type idx: int
        """
        cdef dGeomID gid

        # Check the index
        if idx < 0 or idx >= dSpaceGetNumGeoms(self.sid):
            raise IndexError("geom index out of range")

        gid = dSpaceGetGeom(self.sid, idx)
        if <long>gid not in _geom_c2py_lut:
            raise RuntimeError(
                "geom id cannot be translated to a Python object")

        return _geom_c2py_lut[<long>gid]

    def collide(self, arg, callback):
        """collide(arg, callback)

        Call a callback function one or more times, for all
        potentially intersecting objects in the space. The callback
        function takes 3 arguments:

        def NearCallback(arg, geom1, geom2):

        The arg parameter is just passed on to the callback function.
        Its meaning is user defined. The geom1 and geom2 arguments are
        the geometry objects that may be near each other. The callback
        function can call the function collide() (not the Space
        method) on geom1 and geom2, perhaps first determining
        whether to collide them at all based on other information.

        @param arg: A user argument that is passed to the callback function
        @param callback: Callback function
        @type callback: callable
        """
        
        cdef void* data
        cdef object tup
        tup = (callback, arg)
        data = <void*>tup
        dSpaceCollide(self.sid, data, collide_callback)


# Callback function for the dSpaceCollide() call in the Space.collide() method
# The data parameter is a tuple (Python-Callback, Arguments).
# The function calls a Python callback function with 3 arguments:
# def callback(UserArg, Geom1, Geom2)
# Geom1 and Geom2 are instances of GeomXyz classes.
cdef void collide_callback(void* data, dGeomID o1, dGeomID o2):
    cdef object tup
#    cdef Space space
    cdef long id1, id2

#    if (dGeomGetBody(o1)==dGeomGetBody(o2)):
#        return
    
    tup = <object>data
    callback, arg = tup
    id1 = <long>o1
    id2 = <long>o2
    g1 = _geom_c2py_lut[id1]
    g2 = _geom_c2py_lut[id2]
    callback(arg, g1, g2)


# SimpleSpace
cdef class SimpleSpace(SpaceBase):
    """Simple space.

    This does not do any collision culling - it simply checks every
    possible pair of geoms for intersection, and reports the pairs
    whose AABBs overlap. The time required to do intersection testing
    for n objects is O(n**2). This should not be used for large numbers
    of objects, but it can be the preferred algorithm for a small
    number of objects. This is also useful for debugging potential
    problems with the collision system.
    """

    def __cinit__(self, space=None):
        cdef SpaceBase sp
        cdef dSpaceID parentid

        parentid = NULL
        if space != None:
            sp = space
            parentid = sp.sid
        
        self.sid = dSimpleSpaceCreate(parentid)

        # Copy the ID
        self.gid = <dGeomID>self.sid

        dSpaceSetCleanup(self.sid, 0)
        _geom_c2py_lut[<long>self.sid] = self

    def __init__(self, space=None):
        pass

# HashSpace
cdef class HashSpace(SpaceBase):
    """Multi-resolution hash table space.

    This uses an internal data structure that records how each geom
    overlaps cells in one of several three dimensional grids. Each
    grid has cubical cells of side lengths 2**i, where i is an integer
    that ranges from a minimum to a maximum value. The time required
    to do intersection testing for n objects is O(n) (as long as those
    objects are not clustered together too closely), as each object
    can be quickly paired with the objects around it.
    """

    def __cinit__(self, space=None):
        cdef SpaceBase sp
        cdef dSpaceID parentid

        parentid = NULL
        if space != None:
            sp = space
            parentid = sp.sid
        
        self.sid = dHashSpaceCreate(parentid)

        # Copy the ID
        self.gid = <dGeomID>self.sid

        dSpaceSetCleanup(self.sid, 0)
        _geom_c2py_lut[<long>self.sid] = self

    def __init__(self, space=None):
        pass
    
    def setLevels(self, int minlevel, int maxlevel):
        """setLevels(minlevel, maxlevel)

        Sets the size of the smallest and largest cell used in the
        hash table. The actual size will be 2^minlevel and 2^maxlevel
        respectively.
        """
        
        if minlevel > maxlevel:
            raise ValueError(
                "minlevel (%d) must be less than or equal to maxlevel (%d)" %
                (minlevel, maxlevel))
            
        dHashSpaceSetLevels(self.sid, minlevel, maxlevel)

    def getLevels(self):
        """getLevels() -> (minlevel, maxlevel)

        Gets the size of the smallest and largest cell used in the
        hash table. The actual size is 2^minlevel and 2^maxlevel
        respectively.
        """
        
        cdef int minlevel
        cdef int maxlevel
        dHashSpaceGetLevels(self.sid, &minlevel, &maxlevel)
        return minlevel, maxlevel


# QuadTreeSpace
cdef class QuadTreeSpace(SpaceBase):
    """Quadtree space.

    This uses a pre-allocated hierarchical grid-based AABB tree to
    quickly cull collision checks. It's exceptionally quick for large
    amounts of objects in landscape-shaped worlds. The amount of
    memory used is 4**depth * 32 bytes.

    Currently getGeom() is not implemented for the quadtree space.
    """

    def __cinit__(self, center, extents, depth, space=None):
        cdef SpaceBase sp
        cdef dSpaceID parentid
        cdef dVector3 c
        cdef dVector3 e

        parentid = NULL
        if space != None:
            sp = space
            parentid = sp.sid

        c[0] = center[0]
        c[1] = center[1]
        c[2] = center[2]
        e[0] = extents[0]
        e[1] = extents[1]
        e[2] = extents[2]
        self.sid = dQuadTreeSpaceCreate(parentid, c, e, depth)

        # Copy the ID
        self.gid = <dGeomID>self.sid

        dSpaceSetCleanup(self.sid, 0)
        _geom_c2py_lut[<long>self.sid] = self

    def __init__(self, center, extents, depth, space=None):
        pass


def Space(space_type=0):
    """Space factory function.

    Depending on the type argument this function either returns a
    SimpleSpace (space_type=0) or a HashSpace (space_type=1).

    This function is provided to remain compatible with previous
    versions of PyODE where there was only one Space class.
    
     >>> space = Space(space_type=0)   # Create a SimpleSpace
     >>> space = Space(space_type=1)   # Create a HashSpace
    """
    if space_type == 0:
        return SimpleSpace()
    elif space_type == 1:
        return HashSpace()
    else:
        raise ValueError("Unknown space type (%d)" % space_type)


# GeomSphere
cdef class GeomSphere(GeomObject):
    """Sphere geometry.

    This class represents a sphere centered at the origin.

    Constructor::
    
      GeomSphere(space=None, radius=1.0)
    """

    def __cinit__(self, space=None, radius=1.0):
        cdef SpaceBase sp
        cdef dSpaceID sid

        sid = NULL
        if space != None:
            sp = space
            sid = sp.sid
        self.gid = dCreateSphere(sid, radius)
#        if space != None:
#            space._addgeom(self)

        _geom_c2py_lut[<long>self.gid] = self

    def __init__(self, space=None, radius=1.0):
        self.space = space
        self.body = None

    def placeable(self):
        return True

    def _id(self):
        cdef long id
        id = <long>self.gid
        return id

    def setRadius(self, radius):
        """setRadius(radius)

        Set the radius of the sphere.

        @param radius: New radius
        @type radius: float
        """
        dGeomSphereSetRadius(self.gid, radius)

    def getRadius(self):
        """getRadius() -> float

        Return the radius of the sphere.
        """
        return dGeomSphereGetRadius(self.gid)

    def pointDepth(self, p):
        """pointDepth(p) -> float

        Return the depth of the point p in the sphere. Points inside
        the geom will have positive depth, points outside it will have
        negative depth, and points on the surface will have zero
        depth.

        @param p: Point
        @type p: 3-sequence of floats
        """
        return dGeomSpherePointDepth(self.gid, p[0], p[1], p[2])

                
# GeomBox
cdef class GeomBox(GeomObject):
    """Box geometry.

    This class represents a box centered at the origin.

    Constructor::
    
      GeomBox(space=None, lengths=(1.0, 1.0, 1.0))
    """

    def __cinit__(self, space=None, lengths=(1.0, 1.0, 1.0)):
        cdef SpaceBase sp
        cdef dSpaceID sid
        
        sid = NULL
        if space != None:
            sp = space
            sid = sp.sid
        self.gid = dCreateBox(sid, lengths[0], lengths[1], lengths[2])
#        if space != None:
#            space._addgeom(self)

        _geom_c2py_lut[<long>self.gid] = self

    def __init__(self, space=None, lengths=(1.0, 1.0, 1.0)):
        self.space = space
        self.body = None

    def placeable(self):
        return True

    def _id(self):
        cdef long id
        id = <long>self.gid
        return id

    def setLengths(self, lengths):
        dGeomBoxSetLengths(self.gid, lengths[0], lengths[1], lengths[2])

    def getLengths(self):
        cdef dVector3 res
        dGeomBoxGetLengths(self.gid, res)
        return res[0], res[1], res[2]

    def pointDepth(self, p):
        """pointDepth(p) -> float

        Return the depth of the point p in the box. Points inside the
        geom will have positive depth, points outside it will have
        negative depth, and points on the surface will have zero
        depth.

        @param p: Point
        @type p: 3-sequence of floats
        """
        return dGeomBoxPointDepth(self.gid, p[0], p[1], p[2])


# GeomPlane
cdef class GeomPlane(GeomObject):
    """Plane geometry.

    This class represents an infinite plane. The plane equation is:
    n.x*x + n.y*y + n.z*z = dist

    This object can't be attached to a body.
    If you call getBody() on this object it always returns ode.environment.

    Constructor::
    
      GeomPlane(space=None, normal=(0,0,1), dist=0)

    """

    def __cinit__(self, space=None, normal=(0, 0, 1), dist=0):
        cdef SpaceBase sp
        cdef dSpaceID sid
        
        sid = NULL
        if space != None:
            sp = space
            sid = sp.sid
        self.gid = dCreatePlane(sid, normal[0], normal[1], normal[2], dist)
#        if space != None:
#            space._addgeom(self)

        _geom_c2py_lut[<long>self.gid] = self

    def __init__(self, space=None, normal=(0, 0, 1), dist=0):
        self.space = space

    def _id(self):
        cdef long id
        id = <long>self.gid
        return id

    def setParams(self, normal, dist):
        dGeomPlaneSetParams(self.gid, normal[0], normal[1], normal[2], dist)

    def getParams(self):
        cdef dVector4 res
        dGeomPlaneGetParams(self.gid, res)
        return ((res[0], res[1], res[2]), res[3])

    def pointDepth(self, p):
        """pointDepth(p) -> float

        Return the depth of the point p in the plane. Points inside the
        geom will have positive depth, points outside it will have
        negative depth, and points on the surface will have zero
        depth.

        @param p: Point
        @type p: 3-sequence of floats
        """
        return dGeomPlanePointDepth(self.gid, p[0], p[1], p[2])


# GeomCapsule
cdef class GeomCapsule(GeomObject):
    """Capped cylinder geometry.

    This class represents a capped cylinder aligned along the local Z axis
    and centered at the origin.

    Constructor::
    
      GeomCapsule(space=None, radius=0.5, length=1.0)

    The length parameter does not include the caps.
    """

    def __cinit__(self, space=None, radius=0.5, length=1.0):
        cdef SpaceBase sp
        cdef dSpaceID sid
        
        sid = NULL
        if space != None:
            sp = space
            sid = sp.sid
        self.gid = dCreateCapsule(sid, radius, length)
#        if space != None:
#            space._addgeom(self)

        _geom_c2py_lut[<long>self.gid] = self

    def __init__(self, space=None, radius=0.5, length=1.0):
        self.space = space
        self.body = None

    def placeable(self):
        return True

    def _id(self):
        cdef long id
        id = <long>self.gid
        return id

    def setParams(self, radius, length):
        dGeomCapsuleSetParams(self.gid, radius, length)

    def getParams(self):
        cdef dReal radius, length
        dGeomCapsuleGetParams(self.gid, &radius, &length)
        return radius, length

    def pointDepth(self, p):
        """pointDepth(p) -> float

        Return the depth of the point p in the cylinder. Points inside the
        geom will have positive depth, points outside it will have
        negative depth, and points on the surface will have zero
        depth.

        @param p: Point
        @type p: 3-sequence of floats
        """
        return dGeomCapsulePointDepth(self.gid, p[0], p[1], p[2])

GeomCCylinder = GeomCapsule # backwards compatibility


# GeomCylinder
cdef class GeomCylinder(GeomObject):
    """Plain cylinder geometry.

    This class represents an uncapped cylinder aligned along the local Z axis
    and centered at the origin.

    Constructor::
    
      GeomCylinder(space=None, radius=0.5, length=1.0)
    """

    def __cinit__(self, space=None, radius=0.5, length=1.0):
        cdef SpaceBase sp
        cdef dSpaceID sid
        
        sid = NULL
        if space != None:
            sp = space
            sid = sp.sid
        self.gid = dCreateCylinder(sid, radius, length)
#        if space != None:
#            space._addgeom(self)

        _geom_c2py_lut[<long>self.gid] = self

    def __init__(self, space=None, radius=0.5, length=1.0):
        self.space = space
        self.body = None

    def placeable(self):
        return True

    def _id(self):
        cdef long id
        id = <long>self.gid
        return id

    def setParams(self, radius, length):
        dGeomCylinderSetParams(self.gid, radius, length)

    def getParams(self):
        cdef dReal radius, length
        dGeomCylinderGetParams(self.gid, &radius, &length)
        return radius, length

    ## dGeomCylinderPointDepth not implemented upstream in ODE 0.7


# GeomRay
cdef class GeomRay(GeomObject):
    """Ray object.

    A ray is different from all the other geom classes in that it does
    not represent a solid object. It is an infinitely thin line that
    starts from the geom's position and extends in the direction of
    the geom's local Z-axis.

    Constructor::
    
      GeomRay(space=None, rlen=1.0)
    
    """

    def __cinit__(self, space=None, rlen=1.0):
        cdef SpaceBase sp
        cdef dSpaceID sid
        
        sid = NULL
        if space != None:
            sp = space
            sid = sp.sid
        self.gid = dCreateRay(sid, rlen)
#        if space != None:
#            space._addgeom(self)

        _geom_c2py_lut[<long>self.gid] = self

    def __init__(self, space=None, rlen=1.0):
        self.space = space
        self.body = None

    def _id(self):
        cdef long id
        id = <long>self.gid
        return id

    def placeable(self):
        return True

    def setLength(self, rlen):
        '''setLength(rlen)

        Set length of the ray.

        @param rlen: length of the ray
        @type rlen: float'''
        dGeomRaySetLength(self.gid, rlen)

    def getLength(self):
        '''getLength() -> length

        Get the length of the ray.

        @returns: length of the ray (float)'''
        return dGeomRayGetLength(self.gid)

    def set(self, p, u):
        '''set(p, u)

        Set the position and rotation of a ray.
        
        @param p: position
        @type p: 3-sequence of floats
        @param u: rotation
        @type u: 3-sequence of floats'''
        dGeomRaySet(self.gid, p[0], p[1], p[2], u[0], u[1], u[2])

    def get(self):
        '''get() -> ((p[0], p[1], p[2]), (u[0], u[1], u[2]))

        Return the position and rotation as a pair of
        tuples.

        @returns: position and rotation'''
        cdef dVector3 start
        cdef dVector3 dir
        dGeomRayGet(self.gid, start, dir)
        return (start[0], start[1], start[2]), (dir[0], dir[1], dir[2])


# GeomTransform
cdef class GeomTransform(GeomObject):
    """GeomTransform.

    A geometry transform "T" is a geom that encapsulates another geom
    "E", allowing E to be positioned and rotated arbitrarily with
    respect to its point of reference.

    Constructor::
    
      GeomTransform(space=None)
    """

    cdef object geom

    def __cinit__(self, space=None):
        cdef SpaceBase sp
        cdef dSpaceID sid
        
        sid = NULL
        if space != None:
            sp = space
            sid = sp.sid
        self.gid = dCreateGeomTransform(sid)
        # Set cleanup mode to 0 as a contained geom will be deleted
        # by its Python wrapper class
        dGeomTransformSetCleanup(self.gid, 0)
#        if space != None:
#            space._addgeom(self)

        _geom_c2py_lut[<long>self.gid] = self

    def __init__(self, space=None):
        self.space = space
        self.body = None
        self.geom = None

        self.attribs = {}

    def placeable(self):
        return True

    def _id(self):
        cdef long id
        id = <long>self.gid
        return id

    def setGeom(self, GeomObject geom not None):
        """setGeom(geom)

        Set the geom that the geometry transform encapsulates.
        A ValueError exception is thrown if a) the geom is not placeable,
        b) the geom was already inserted into a space or c) the geom is
        already associated with a body.

        @param geom: Geom object to encapsulate
        @type geom: GeomObject
        """
        cdef long id

        if not geom.placeable():
            raise ValueError(
                "Only placeable geoms can be encapsulated by a GeomTransform")
        if dGeomGetSpace(geom.gid) != <dSpaceID>0:
            raise ValueError(
                "The encapsulated geom was already inserted into a space")
        if dGeomGetBody(geom.gid) != <dBodyID>0:
            raise ValueError(
                "The encapsulated geom is already associated with a body")
        
        id = geom._id()
        dGeomTransformSetGeom(self.gid, <dGeomID>id)
        self.geom = geom

    def getGeom(self):
        """getGeom() -> GeomObject

        Get the geom that the geometry transform encapsulates.
        """
        return self.geom

    def setInfo(self, int mode):
        """setInfo(mode)

        Set the "information" mode of the geometry transform.

        With mode 0, when a transform object is collided with another
        object, the geom field of the ContactGeom structure is set to the
        geom that is encapsulated by the transform object.

        With mode 1, the geom field of the ContactGeom structure is set
        to the transform object itself.

        @param mode: Information mode (0 or 1)
        @type mode: int
        """
        if mode < 0 or mode > 1:
            raise ValueError(
                "Invalid information mode (%d). Must be either 0 or 1." % mode)
        dGeomTransformSetInfo(self.gid, mode)

    def getInfo(self):
        """getInfo() -> int

        Get the "information" mode of the geometry transform (0 or 1).

        With mode 0, when a transform object is collided with another
        object, the geom field of the ContactGeom structure is set to the
        geom that is encapsulated by the transform object.

        With mode 1, the geom field of the ContactGeom structure is set
        to the transform object itself.
        """
        return dGeomTransformGetInfo(self.gid)

######################################################################


cdef class TriMeshData:
    """This class stores the mesh data.
    """

    cdef dTriMeshDataID tmdid
    cdef dReal* vertex_buffer
    cdef int* face_buffer

    def __cinit__(self):
        self.tmdid = dGeomTriMeshDataCreate()
        self.vertex_buffer = NULL
        self.face_buffer = NULL

    def __dealloc__(self):
        if self.tmdid != NULL:
            dGeomTriMeshDataDestroy(self.tmdid)
        if self.vertex_buffer != NULL:
            free(self.vertex_buffer)
        if self.face_buffer != NULL:
            free(self.face_buffer)
    
    def build(self, verts, faces):
        """build(verts, faces)

        @param verts: Vertices
        @type verts: Sequence of 3-sequences of floats
        @param faces: Face definitions (three indices per face)
        @type faces: Sequence of 3-sequences of ints
        """
        cdef int numverts
        cdef int numfaces
        cdef dReal* vp
        cdef int* fp
        cdef int a, b, c
        
        numverts = len(verts)
        numfaces = len(faces)
        # Allocate the vertex and face buffer
        self.vertex_buffer = <dReal*>malloc(numverts * 4 * sizeof(dReal))
        self.face_buffer = <int*>malloc(numfaces * 3 * sizeof(int))

        # Fill the vertex buffer
        vp = self.vertex_buffer
        for v in verts:
            vp[0] = v[0]
            vp[1] = v[1]
            vp[2] = v[2]
            vp[3] = 0
            vp = vp + 4

        # Fill the face buffer
        fp = self.face_buffer
        for f in faces:
            a = f[0]
            b = f[1]
            c = f[2]
            if (a < 0 or b < 0 or c < 0 or a >= numverts or b >= numverts or c >= numverts):
                raise ValueError("Vertex index out of range")
            fp[0] = a
            fp[1] = b
            fp[2] = c
            fp = fp + 3

        # Pass the data to ODE
        dGeomTriMeshDataBuildSimple(self.tmdid, self.vertex_buffer, numverts,
                                    self.face_buffer, numfaces * 3)

######################################################################


# GeomTriMesh
cdef class GeomTriMesh(GeomObject):
    """TriMesh object.

    To construct the trimesh geom you need a TriMeshData object that
    stores the actual mesh. This object has to be passed as first
    argument to the constructor.

    Constructor::
    
      GeomTriMesh(data, space=None)
    """

    # Keep a reference to the data
    cdef TriMeshData data

    def __cinit__(self, TriMeshData data not None, space=None):
        cdef SpaceBase sp
        cdef dSpaceID sid

        self.data = data

        sid = NULL
        if space != None:
            sp = space
            sid = sp.sid
        self.gid = dCreateTriMesh(sid, data.tmdid, NULL, NULL, NULL)

        _geom_c2py_lut[<long>self.gid] = self

    def __init__(self, TriMeshData data not None, space=None):
        self.space = space
        self.body = None

    def placeable(self):
        return True

    def _id(self):
        cdef long id
        id = <long>self.gid
        return id

    def clearTCCache(self):
        """clearTCCache()

        Clears the internal temporal coherence caches.
        """
        dGeomTriMeshClearTCCache(self.gid)

    def getTriangle(self, int idx):
        """getTriangle(idx) -> (v0, v1, v2)

        @param idx: Triangle index
        @type idx: int
        """

        cdef dVector3 v0, v1, v2
        cdef dVector3* vp0
        cdef dVector3* vp1
        cdef dVector3* vp2

        vp0 = <dVector3*>v0
        vp1 = <dVector3*>v1
        vp2 = <dVector3*>v2

        dGeomTriMeshGetTriangle(self.gid, idx, vp0, vp1, vp2)
        return ((v0[0], v0[1], v0[2]),
                (v1[0], v1[1], v1[2]),
                (v2[0], v2[1], v2[2]))
        
    def getTriangleCount(self):
        """getTriangleCount() -> n

        Returns the number of triangles in the TriMesh."""

        return dGeomTriMeshGetTriangleCount(self.gid)

######################################################################


def collide(geom1, geom2):
    """collide(geom1, geom2) -> contacts

    Generate contact information for two objects.

    Given two geometry objects that potentially touch (geom1 and geom2),
    generate contact information for them. Internally, this just calls
    the correct class-specific collision functions for geom1 and geom2.

    [flags specifies how contacts should be generated if the objects
    touch. Currently the lower 16 bits of flags specifies the maximum
    number of contact points to generate. If this number is zero, this
    function just pretends that it is one - in other words you can not
    ask for zero contacts. All other bits in flags must be zero. In
    the future the other bits may be used to select other contact
    generation strategies.]

    If the objects touch, this returns a list of Contact objects,
    otherwise it returns an empty list.

    @param geom1: First Geom
    @type geom1: GeomObject
    @param geom2: Second Geom
    @type geom2: GeomObject
    @returns: Returns a list of Contact objects.
    """
    
    cdef dContactGeom c[150]
    cdef long id1
    cdef long id2
    cdef int i, n
    cdef Contact cont

    id1 = geom1._id()
    id2 = geom2._id()

    n = dCollide(<dGeomID>id1, <dGeomID>id2, 150, c, sizeof(dContactGeom))
    res = []
    i = 0
    while i < n:
        cont = Contact()
        cont._contact.geom = c[i]
        res.append(cont)
        i = i + 1

    return res


def collide2(geom1, geom2, arg, callback):
    """collide2(geom1, geom2, arg, callback)
    
    Calls the callback for all potentially intersecting pairs that contain
    one geom from geom1 and one geom from geom2.

    @param geom1: First Geom
    @type geom1: GeomObject
    @param geom2: Second Geom
    @type geom2: GeomObject
    @param arg: A user argument that is passed to the callback function
    @param callback: Callback function
    @type callback: callable
    """
    cdef void* data
    cdef object tup
    cdef long id1
    cdef long id2

    id1 = geom1._id()
    id2 = geom2._id()
    
    tup = (callback, arg)
    data = <void*>tup
    # collide_callback is defined in space.pyx
    dSpaceCollide2(<dGeomID>id1, <dGeomID>id2, data, collide_callback)


def areConnected(Body body1, Body body2):
    """areConnected(body1, body2) -> bool

    Return True if the two bodies are connected together by a joint,
    otherwise return False.

    @param body1: First body
    @type body1: Body
    @param body2: Second body
    @type body2: Body
    @returns: True if the bodies are connected
    """

    if body1 is environment:
        return False
    if body2 is environment:
        return False

    return bool(dAreConnected(<dBodyID> body1.bid, <dBodyID> body2.bid))


def CloseODE():
    """CloseODE()

    Deallocate some extra memory used by ODE that can not be deallocated
    using the normal destroy functions.
    """
    dCloseODE()


def InitODE():
    '''InitODE()

    Initialize some ODE internals. This will be called for you when you
    "import ode", but you should call this again if you CloseODE().'''
    dInitODE()


#environment = Body(None)
environment = None
InitODE()
