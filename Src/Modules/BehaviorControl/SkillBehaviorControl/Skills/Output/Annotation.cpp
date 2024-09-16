/**
 * @file Annotation.cpp
 *
 * This file implements the Annotation skill.
 *
 * @author Arne Hasselbring
 */

#include "SkillBehaviorControl.h"
#include "Debugging/Annotation.h"

option((SkillBehaviorControl) Annotation,
       args((const std::string&) annotation),
       vars((std::string)("") lastAnnotationSent))
{
  initial_state(execute)
  {
    transition
    {
      if(annotation == lastAnnotationSent)
        goto done;
    }
    action
    {
      ANNOTATION("Behavior", annotation);
      lastAnnotationSent = annotation;
    }
  }

  target_state(done)
  {
    transition
    {
      if(annotation != lastAnnotationSent)
        goto execute;
    }
  }
}
