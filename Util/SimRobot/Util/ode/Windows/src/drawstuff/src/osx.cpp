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

// Platform-specific code for Mac OS X using Carbon+AGL
//
// Created using x11.cpp and the window-initialization -routines from GLFW
// as reference.
// Not thoroughly tested and is certain to contain deficiencies and bugs

#include <ode/odeconfig.h>
#include "config.h"
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#ifdef HAVE_SYS_TIME_H
#include <sys/time.h>
#endif

#include <drawstuff/drawstuff.h>
#include <drawstuff/version.h>
#include "internal.h"

#include <OpenGL/gl.h>
#include <GLUT/glut.h>

// Global variables

static bool paused = false;			// 1 if in `pause' mode
static bool singlestep = false;		// 1 if single step key pressed
static bool writeframes = false;	// 1 if frame files to be written

static int					   	windowWidth = -1;
static int					   	windowHeight = -1;
static int 						mouseButtonMode = 0;	
static bool						mouseWithOption = false;	// Set if dragging the mouse with alt pressed
static bool						mouseWithControl = false;	// Set if dragging the mouse with ctrl pressed

static dsFunctions*			   	functions = NULL;
static int                   	windowReference;
static int                      frame = 1;
static int                      prev_x = 0;
static int                      prev_y = 0;

//***************************************************************************
// error handling for unix

static void printMessage (const char *msg1, const char *msg2, va_list ap)
{
  fflush (stderr);
  fflush (stdout);
  fprintf (stderr,"\n%s: ",msg1);
  vfprintf (stderr,msg2,ap);
  fprintf (stderr,"\n");
  fflush (stderr);
}

extern "C" void dsError (const char *msg, ...)
{
  va_list ap;
  va_start (ap,msg);
  printMessage ("Error",msg,ap);
  va_end (ap);
  exit (1);
}


extern "C" void dsDebug (const char *msg, ...)
{
  va_list ap;
  va_start (ap,msg);
  printMessage ("INTERNAL ERROR",msg,ap);
  va_end (ap);
  // *((char *)0) = 0;	 ... commit SEGVicide ?
  abort();
}

extern "C" void dsPrint (const char *msg, ...)
{
  va_list ap;
  va_start (ap,msg);
  vprintf (msg,ap);
  va_end (ap);
}

static void captureFrame( int num ){

  	fprintf( stderr,"\rcapturing frame %04d", num );
	unsigned char buffer[windowWidth*windowHeight][3];
	glReadPixels( 0, 0, windowWidth, windowHeight, GL_RGB, GL_UNSIGNED_BYTE, &buffer );
	char s[100];
	sprintf (s,"frame%04d.ppm",num);
	FILE *f = fopen (s,"wb");
	if( !f ){
		dsError( "can't open \"%s\" for writing", s );
	}
	fprintf( f,"P6\n%d %d\n255\n", windowWidth, windowHeight );
	for( int y=windowHeight-1; y>-1; y-- ){
		fwrite( buffer[y*windowWidth], 3*windowWidth, 1, f );
	}
	fclose (f);	
}

extern "C" void dsStop()
{
}

extern "C" double dsElapsedTime()
{
#if HAVE_GETTIMEOFDAY
  static double prev=0.0;
  timeval tv ;

  gettimeofday(&tv, 0);
  double curr = tv.tv_sec + (double) tv.tv_usec / 1000000.0 ;
  if (!prev)
    prev=curr;
  double retval = curr-prev;
  prev=curr;
  if (retval>1.0) retval=1.0;
  if (retval<dEpsilon) retval=dEpsilon;
  return retval;
#else
  return 0.01666; // Assume 60 fps
#endif
}

int osxGetModifierMask()
{
    return glutGetModifiers() & ~GLUT_ACTIVE_SHIFT;
}

void osxKeyEventHandler( unsigned char key, int x, int y )
{
    unsigned char uppercase;
    if (key >= 'a' && key <= 'z')
        uppercase = key - ('a' - 'A');
    else
        uppercase = key;
    
    int modifierMask = osxGetModifierMask();    
    if (modifierMask == 0)
    {
        if( key >= ' ' && key <= 126 && functions -> command )
            functions -> command( key );
    }
    else if (modifierMask & GLUT_ACTIVE_CTRL)
    {
        // ctrl+key was pressed
        uppercase += 'A' - 1;
        switch(uppercase ){
            case 'T':
                dsSetTextures( !dsGetTextures() );
                break;
            case 'S':
                dsSetShadows( !dsGetShadows() );
                break;
            case 'X':
                exit(0);
                break;
            case 'P':
                paused = !paused;
                singlestep = false;
                break;
            case 'O':
                if( paused ){
                    singlestep = true;
                }
                break;
            case 'V': {
                float xyz[3],hpr[3];
                dsGetViewpoint( xyz,hpr );
                printf( "Viewpoint = (%.4f,%.4f,%.4f,%.4f,%.4f,%.4f)\n", xyz[0], xyz[1], xyz[2], hpr[0], hpr[1], hpr[2] );
                break;
            }
            case 'W':						
                writeframes = !writeframes;
                if( writeframes ){
                    printf( "Now writing frames to PPM files\n" );
                }						 
                break;
        }
    }
}

void osxMouseEventHandler(int button, int state, int x, int y)
{
    prev_x = x;
    prev_y = y;
	bool buttonDown = false;
    switch( state ){
        case GLUT_DOWN:
			buttonDown = true;
        case GLUT_UP:
            if( button == GLUT_LEFT_BUTTON ){
                int modifierMask = osxGetModifierMask();
                if( modifierMask & GLUT_ACTIVE_CTRL ){
                    // Ctrl+button == right
                    button = GLUT_RIGHT_BUTTON;
                    mouseWithControl = true;
                }	
                else if( modifierMask & GLUT_ACTIVE_ALT ){
                    // Alt+button == left+right
                    mouseButtonMode = 5;
                    mouseWithOption = true;
                    return;
                }
            }
            if( buttonDown ){
                if( button == GLUT_LEFT_BUTTON ) mouseButtonMode |= 1;		// Left
                if( button == GLUT_MIDDLE_BUTTON ) mouseButtonMode |= 2;	// Middle				
                if( button == GLUT_RIGHT_BUTTON ) mouseButtonMode |= 4;	    // Right
            }
            else{
                if( button == GLUT_LEFT_BUTTON ) mouseButtonMode &= (~1);	// Left
                if( button == GLUT_MIDDLE_BUTTON ) mouseButtonMode &= (~2);	// Middle									
                if( button == GLUT_RIGHT_BUTTON ) mouseButtonMode &= (~4);  // Right
            }		
            return;
    }
}

void osxMotionEventHandler(int x, int y)
{
    dsMotion( mouseButtonMode, x - prev_x, y - prev_y );
    prev_x = x;
    prev_y = y;
}

void osxWindowReshapeEventHandler(int width, int height)
{
    windowWidth = width;
    windowHeight = height;
}

static void osxCreateMainWindow( int width, int height )
{
    int argc = 1;
    char* argv[2];
    argv[0] = (char*)"";
    argv[1] = NULL;
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(width, height);

    windowReference = glutCreateWindow("ODE - Drawstuff");
	windowWidth = width;
	windowHeight = height;
}

void osxRedisplayEventHandler()
{
    dsDrawFrame( windowWidth, windowHeight, functions, paused && !singlestep );
    singlestep = false;
    glutSwapBuffers();
    
    // capture frames if necessary
    if( !paused && writeframes ){
        captureFrame( frame );
        frame++;
    }
}

void osxTimerEventHandler(int);

void osxInstallTimerHandler()
{
    glutTimerFunc(1000/60, osxTimerEventHandler, 0);
}

void osxTimerEventHandler(int)
{
    glutPostRedisplay();
    osxInstallTimerHandler();
}

int  osxInstallEventHandlers()
{
    glutKeyboardFunc(osxKeyEventHandler);
    glutMouseFunc(osxMouseEventHandler);
    glutMotionFunc(osxMotionEventHandler);
    glutDisplayFunc(osxRedisplayEventHandler);
    glutReshapeFunc(osxWindowReshapeEventHandler);
    osxInstallTimerHandler();    
    return GL_TRUE;
}

extern void dsPlatformSimLoop( int givenWindowWidth, int givenWindowHeight, dsFunctions *fn, int givenPause ){
	
	functions = fn;
	
	paused = givenPause;
	
	osxCreateMainWindow( givenWindowWidth, givenWindowHeight );
	osxInstallEventHandlers();
	
	dsStartGraphics( windowWidth, windowHeight, fn );
	
	static bool firsttime=true;
	if( firsttime )
	{
		fprintf
		(
		 stderr,
		 "\n"
		 "Simulation test environment v%d.%02d\n"
		 "   Ctrl-P : pause / unpause (or say `-pause' on command line).\n"
		 "   Ctrl-O : single step when paused.\n"
		 "   Ctrl-T : toggle textures (or say `-notex' on command line).\n"
		 "   Ctrl-S : toggle shadows (or say `-noshadow' on command line).\n"
		 "   Ctrl-V : print current viewpoint coordinates (x,y,z,h,p,r).\n"
		 "   Ctrl-W : write frames to ppm files: frame/frameNNN.ppm\n"
		 "   Ctrl-X : exit.\n"
		 "\n"
		 "Change the camera position by clicking + dragging in the window.\n"
		 "   Left button - pan and tilt.\n"
		 "   Right button (or Ctrl + button) - forward and sideways.\n"
		 "   Left + Right button (or middle button, or Alt + button) - sideways and up.\n"
		 "\n",DS_VERSION >> 8,DS_VERSION & 0xff
		 );
		firsttime = false;
	}
	
	if( fn -> start ) fn->start();
	
    glutMainLoop();
}
