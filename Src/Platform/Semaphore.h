/**
* @file Platform/Semaphore.h
*
* Inclusion of platform dependent definitions for semaphore usage.
*
* @author Colin Graf
*/

#pragma once

#ifdef LINUX
#include "Linux/Semaphore.h"
#define SEMAPHORE_INCLUDED
#endif

#ifdef MACOSX
#include "MacOS/Semaphore.h"
#define SEMAPHORE_INCLUDED
#endif

#ifdef WIN32
#include "Win32/Semaphore.h"
#define SEMAPHORE_INCLUDED
#endif

#ifndef SEMAPHORE_INCLUDED
#error Unknown platform
#endif
