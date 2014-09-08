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

#ifdef OSX
#include "OSX/Semaphore.h"
#define SEMAPHORE_INCLUDED
#endif

#ifdef WINDOWS
#include "Windows/Semaphore.h"
#define SEMAPHORE_INCLUDED
#endif

#ifndef SEMAPHORE_INCLUDED
#error Unknown platform
#endif
