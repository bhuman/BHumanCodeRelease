/**
* @file  Platform/SystemCall.h
*
* Inclusion of platform dependent definitions of system calls.
*
* @author <a href="mailto:martin@martin-loetzsch.de">Martin Lötzsch</a>
*/

#pragma once

#ifdef TARGET_ROBOT
#include "Linux/SystemCall.h"
#define SYSTEMCALL_INCLUDED
#endif

#if defined(TARGET_SIM) || defined(TARGET_TOOL)
#include "SimRobotQt/SystemCall.h"
#define SYSTEMCALL_INCLUDED
#endif

#ifndef SYSTEMCALL_INCLUDED
#error "Unknown platform or target"
#endif
