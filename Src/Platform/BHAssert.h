/**
* @file Platform/BHAssert.h
*
* Inclusion of platform dependent definitions for low level debugging.
*
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once

#ifdef TARGET_ROBOT

#include "Linux/BHAssert.h"
#define BHASSERT_INCLUDED

#endif

#if defined(TARGET_SIM) || defined(TARGET_TOOL)

#ifdef WIN32
#include "Win32/BHAssert.h"
#define BHASSERT_INCLUDED
#endif

#ifdef LINUX
#include "Linux/BHAssert.h"
#define BHASSERT_INCLUDED
#endif

#ifdef MACOSX
#include "Linux/BHAssert.h"
#define BHASSERT_INCLUDED
#endif

#endif

#ifndef BHASSERT_INCLUDED
#error "Unknown platform or target"
#endif
