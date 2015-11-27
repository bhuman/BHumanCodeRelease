#!/usr/bin/env python

# http://pyode.sourceforge.net/tutorials/tutorial1.html

# pyODE example 1: Getting started

# modified by Gideon Klompje (removed literals and using
# 'ode.Mass.setSphereTotal' instead of 'ode.Mass.setSphere')

import ode

# Simulation constants
GRAVITY = (0, -9.81, 0)

SPHERE_RADIUS = 0.05
SPHERE_MASS = 1.0
SPHERE_START_POS = (0, 2, 0)
SPHERE_FORCE = (0, 200, 0) # Initial force to apply to the sphere

TIME_STEP = 0.04
TIME_STOP = 2.0 # When to stop the simulation

# Create a world object
world = ode.World()
world.setGravity(GRAVITY)

# Create a spherical body inside the world
body = ode.Body(world)
mass = ode.Mass()
mass.setSphereTotal(SPHERE_MASS, SPHERE_RADIUS)
body.setMass(mass)

body.setPosition(SPHERE_START_POS)
body.addForce(SPHERE_FORCE)

# Do the simulation...
if __name__ == "__main__":
    total_time = 0.0
    while total_time < TIME_STOP:
        # output the body's position and velocity
        x, y, z = body.getPosition()
        u, v, w = body.getLinearVel()
        print "%1.2fsec: pos=(%6.3f, %6.3f, %6.3f)  vel=(%6.3f, %6.3f, %6.3f)" % \
            (total_time, x, y, z, u, v, w)

        # advance the simulation
        world.step(TIME_STEP)
        total_time += TIME_STEP

