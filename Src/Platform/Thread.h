/**
* @file Platform/Thread.h
*
* Inclusion of platform dependent definitions for thread usage.
*
* @author <a href="mailto:ingsie@informatik.uni-bremen.de">Ingo Sieverdingbeck</a>
*/

#pragma once

#ifdef TARGET_ROBOT

#include "Linux/Thread.h"
#define THREAD_INCLUDED

#endif

#if defined(TARGET_SIM) || defined(TARGET_TOOL)

#ifdef WINDOWS
#include "Windows/Thread.h"
#define THREAD_INCLUDED
#endif

#ifdef LINUX
#include "Linux/Thread.h"
#define THREAD_INCLUDED
#endif

#ifdef OSX
#include "linux/Thread.h"
#define THREAD_INCLUDED
#endif

#endif

#ifndef THREAD_INCLUDED
#error Unknown platform
#endif
