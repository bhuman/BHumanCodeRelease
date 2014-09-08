/**
* @file Controller/Platform/Directory.h
* Inclusion of platform dependend implementation of a class for accessing directories.
*
* @author Colin Graf
*/

#pragma once
#ifdef WINDOWS
#include "Windows/Directory.h"

#elif defined LINUX || defined OSX
#include "Linux/Directory.h"

#else
#error "Unknown platform"
#endif
