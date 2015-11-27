#!/usr/bin/env python

# http://pyode.sourceforge.net/tutorials/tutorial2.html

# pyODE example 2: Connecting bodies with joints

# modified by Gideon Klompje (removed literals and using
# 'ode.Mass.setSphereTotal' instead of 'ode.Mass.setSphere')


import ode
import pygame

from pygame.locals import QUIT, KEYDOWN

# Constants
WINDOW_RESOLUTION = (640, 480)

DRAW_SCALE = WINDOW_RESOLUTION[0] / 5
"""Factor to multiply physical coordinates by to obtain screen size in pixels"""

DRAW_OFFSET = (WINDOW_RESOLUTION[0] / 2, 50)
"""Screen coordinates (in pixels) that map to the physical origin (0, 0, 0)"""

BACKGROUND_COLOR = (255, 255, 255)

GRAVITY = (0, -9.81, 0)

SPHERE1_POSITION = (1, 0, 0)
SPHERE1_MASS = 1
SPHERE1_RADIUS = 0.15
SPHERE1_COLOR = (55, 0, 200)

SPHERE2_POSITION = (2, 0, 0)
SPHERE2_MASS = 1
SPHERE2_RADIUS = 0.15
SPHERE2_COLOR = (55, 0, 200)

JOINT1_ANCHOR = (0, 0, 0)
JOINT1_COLOR = (200, 0, 55)
JOINT1_WIDTH = 2
"""Width of the line (in pixels) representing the joint"""

JOINT2_ANCHOR = SPHERE1_POSITION
JOINT2_COLOR = (200, 0, 55)
JOINT2_WIDTH = 2
"""Width of the line (in pixels) representing the joint"""

TIME_STEP = 0.04

# Utility functions
def coord(x, y, integer=False):
    """
    Convert world coordinates to pixel coordinates.  Setting 'integer' to
    True will return integer coordinates.
    """
    xs = (DRAW_OFFSET[0] + DRAW_SCALE*x)
    ys = (DRAW_OFFSET[1] - DRAW_SCALE*y)

    if integer:
        return int(round(xs)), int(round(ys))
    else:
        return xs, ys

# Initialize pygame
pygame.init()

# Open a display
screen = pygame.display.set_mode(WINDOW_RESOLUTION)

# Create a world object
world = ode.World()
world.setGravity(GRAVITY)

# Create two bodies
body1 = ode.Body(world)
M = ode.Mass()
M.setSphereTotal(SPHERE1_MASS, SPHERE1_RADIUS)
body1.setMass(M)
body1.setPosition(SPHERE1_POSITION)

body2 = ode.Body(world)
M = ode.Mass()
M.setSphereTotal(SPHERE2_MASS, SPHERE2_RADIUS)
body2.setMass(M)
body2.setPosition(SPHERE2_POSITION)

# Connect body1 with the static environment
j1 = ode.BallJoint(world)
j1.attach(body1, ode.environment)
j1.setAnchor(JOINT1_ANCHOR)

# Connect body2 with body1
j2 = ode.BallJoint(world)
j2.attach(body1, body2)
j2.setAnchor(JOINT2_ANCHOR)

# Simulation loop...
if __name__ == "__main__":
    fps = 1.0 / TIME_STEP
    clk = pygame.time.Clock()

    sph1_rad = int(DRAW_SCALE * SPHERE1_RADIUS)
    sph2_rad = int(DRAW_SCALE * SPHERE2_RADIUS)

    loopFlag = True
    while loopFlag:
        for e in pygame.event.get():
            if e.type==QUIT:
                loopFlag=False
            if e.type==KEYDOWN:
                loopFlag=False

        # Clear the screen
        screen.fill(BACKGROUND_COLOR)

        # Draw the two bodies and the lines representing the joints
        x1, y1, z1 = body1.getPosition()
        x2, y2, z2 = body2.getPosition()
        xj1, yj1, zj1 = j1.getAnchor()
        xj2, yj2, zj2 = j2.getAnchor()

        pygame.draw.line(screen, JOINT1_COLOR, coord(xj1, yj1), coord(x1, y1), JOINT1_WIDTH)
        pygame.draw.line(screen, JOINT2_COLOR, coord(xj2, yj2), coord(x2, y2), JOINT2_WIDTH)
        pygame.draw.circle(screen, SPHERE1_COLOR, coord(x1, y1, integer=True), sph1_rad, 0)
        pygame.draw.circle(screen, SPHERE2_COLOR, coord(x2, y2, integer=True), sph2_rad, 0)

        pygame.display.flip()

        # Next simulation step
        world.step(TIME_STEP)

        # Try to keep the specified framerate    
        clk.tick(fps)

