/**
* @file Controller/Platform/Directory.h
* Inclusion of platform dependend implementation of a class for accessing directories.
*
* @author Colin Graf
*/

#pragma once
#ifdef WIN32
#include "Win32/Directory.h"

#elif defined LINUX || defined MACOSX
#include "Linux/Directory.h"

#else
#error "Unknown platform"
#endif
