/**
 * @file Annotation.h
 * @author Andreas Stolpmann
 */

#pragma once

#include "AnnotationManager.h"
#include "Debugging.h"

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
    Global::getAnnotationManager().addAnnotation(); \
    Global::getAnnotationManager().getOut().out.text << name << message; \
    Global::getAnnotationManager().getOut().out.finishMessage(idAnnotation); \
  } \
  while(false)
