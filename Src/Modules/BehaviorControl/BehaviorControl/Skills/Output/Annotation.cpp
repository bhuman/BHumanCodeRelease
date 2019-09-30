/**
 * @file Annotation.cpp
 *
 * This file implements the implementation of the Annotation skill.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Tools/Debugging/Annotation.h"
#include <string>

SKILL_IMPLEMENTATION(AnnotationImpl,
{,
  IMPLEMENTS(Annotation),
});

class AnnotationImpl : public AnnotationImplBase
{
  void execute(const Annotation& p) override
  {
    if(p.annotation != lastAnnotationSent)
    {
      ANNOTATION("Behavior", p.annotation);
      lastAnnotationSent = p.annotation;
    }
  }

  void reset(const Annotation&) override
  {
    lastAnnotationSent.clear();
  }

  std::string lastAnnotationSent; /**< The last annotation sent. */
};

MAKE_SKILL_IMPLEMENTATION(AnnotationImpl);
