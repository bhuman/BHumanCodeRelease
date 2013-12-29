/**
* @file Platform/DebugHandler.h
*
* Inclusion of platform dependent code for debug communication.
*
* @author <a href="mailto:robocup@m-wachter.de">Michael Wachter</a>
*/

#pragma once

#ifdef TARGET_ROBOT

#include "Linux/DebugHandler.h"
#define EXTERNAL_DEBUGGING \
  MessageQueue theDebugReceiver; \
  MessageQueue theDebugSender; \
  DebugHandler debugHandler;

#define INIT_EXTERNAL_DEBUGGING \
  Process(theDebugReceiver, theDebugSender), \
  debugHandler(theDebugReceiver, theDebugSender, MAX_PACKAGE_SEND_SIZE, 0)

#define DO_EXTERNAL_DEBUGGING(s) \
  debugHandler.communicate(s)

#endif

#ifdef TARGET_SIM

#define EXTERNAL_DEBUGGING DEBUGGING
#define INIT_EXTERNAL_DEBUGGING INIT_DEBUGGING
#define DO_EXTERNAL_DEBUGGING(s) \
  if(s) \
    theDebugSender.send()

#endif

#ifndef EXTERNAL_DEBUGGING
#error "Unknown platform or target"
#endif
