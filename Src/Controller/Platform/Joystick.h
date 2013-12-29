/**
* @file Controller/Platform/Joystick.h
* Inclusion of platform dependend joystick interface implementation.
* @author Colin Graf
*/

#pragma once

#ifdef WIN32
#include "Win32/Joystick.h"
#define Joystick_H
#endif

#ifdef LINUX
#include "Linux/Joystick.h"
#define Joystick_H
#endif

#ifdef MACOSX
#include "MacOS/Joystick.h"
#define Joystick_H
#endif

#ifndef Joystick_H
#error "Unknown platform"
#endif
