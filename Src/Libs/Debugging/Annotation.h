/**
 * @file Annotation.h
 * @author Andreas Stolpmann
 */

#pragma once

#include "Debugging/AnnotationManager.h"
#include "Debugging/Debugging.h"

/**
 * A macro for sending annotation messages.
 *
 * @param name The part of the system where this annotation originates.
 * @param message A message streamable as text.
 *
 * Examples:
 * <pre>
 * ANNOTATION("Behavior", "Kickin'!");
 * ANNOTATION("TacticProvider", "Changed Tactic from " << oldTactic << " to " << newTactic << ".");
 * </pre>
 */
#define ANNOTATION(name, message) \
  do \
  { \
    OutTextMemory _text; \
    _text << name << message; \
    Global::getAnnotationManager().add().write(_text.data(), _text.size()); \
  } \
  while(false)
