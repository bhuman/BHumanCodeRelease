/**
 * @file RoleCheck.h
 * The file declares macros to assist a fast check-function declaration.
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Tools/Debugging/Annotation.h"
#include "Tools/Module/Blackboard.h"
#include "Representations/Infrastructure/FrameInfo.h"

/**
 * A macro for creating the check body with an intance of FrameInfo (variablen name: theFrameInfo)
 *
 * code will be only evaluated if FrameInfo exists
 */
#define CHECK(code) void verify() const \
{ \
  if(Blackboard::getInstance().exists("FrameInfo")) \
  { \
    const FrameInfo& theFrameInfo = static_cast<const FrameInfo&>(Blackboard::getInstance()["FrameInfo"]); \
    code \
  } \
};\

/**
 * A macro for creating helper variables for ANNOTATE_OSCILLATION
 *
 * @param type, the type of var from ANNOTATE_OSCILLATION
 * @param inital, an inital value of var from ANNOTATE_OSCILLATION
 */
#define CHECK_OSCILLATION_VARS(type, initial) const struct CheckOscillationVarHolder \
{ \
  CheckOscillationVarHolder() : oscillationTimeStamp(0) COMMA oscillationLastVal(initial) {} \
  unsigned oscillationTimeStamp; \
  type oscillationLastVal; \
} checkOscillationVars;

/**
 * A macro for annotate the an oscillation of a variavle
 *
 * @param var, the variable to check on oscillation
 * @param name, the ANNOTATION name
 * @param allowedMinTimeDif, the minimal time which has to be gone to say it is not oscillating
 */
#define ANNOTATE_OSCILLATION(var, name, allowedMinTimeDif) \
{  \
    if(checkOscillationVars.oscillationLastVal != var) \
    { \
      if(theFrameInfo.time && theFrameInfo.getTimeSince(checkOscillationVars.oscillationTimeStamp) <= allowedMinTimeDif) \
        ANNOTATION(name, "CHECK FAILED of ANNOTATE_OSCILLATION: time difference was: " << theFrameInfo.getTimeSince(checkOscillationVars.oscillationTimeStamp) << " allowed: " << allowedMinTimeDif); \
      const_cast<CheckOscillationVarHolder&>(checkOscillationVars).oscillationTimeStamp = theFrameInfo.time; \
      const_cast<CheckOscillationVarHolder&>(checkOscillationVars).oscillationLastVal = var; \
    } \
}

/**
 * A marco for creating an oscillation check within the check-function of a STREAMABLE
 *
 * @param var, the variable to check on oscillation
 * @param type, the type of var
 * @param inital, an inital value of the var-type
 * @param name, the ANNOTATION name
 * @param allowedMinTimeDif, the minimal time which has to be gone to say it is not oscillating
 */
#define CHECK_OSCILLATION(var, type, inital, name, allowedMinTimeDif) CHECK_OSCILLATION_VARS(type, inital) CHECK(ANNOTATE_OSCILLATION(var, name, allowedMinTimeDif))
