/**
 * @file SkillStubs.h
 *
 * This file declares all skills that are used by the current behavior. This file is intended
 * to be included inside of a class body. "Skills.h" must have been already included
 * (outside the class). It is assumed that a declaration "SkillRegistry theSkillRegistry"
 * was made before this file is included.
 *
 * @author Thomas Röfer
 */

#include "SkillInterfaces.h"

#ifndef SKILL_INTERFACE
#error "'Skills.h' must have already been included!"
#else
#undef SKILL_INTERFACE
#define SKILL_INTERFACE(name, ...) Skills::name##Skill& the##name##Skill = *theSkillRegistry.getSkill<Skills::name##Skill>(#name)
#include "SkillInterfaces.h"
#endif
