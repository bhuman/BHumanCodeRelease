/**
 * @file Tools/ProcessFramework/ProcessFramework.cpp
 *
 * This file implements classes corresponding to the process framework.
 *
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "ProcessFramework.h"

ProcessCreatorBase* ProcessCreatorBase::first = nullptr;

void PlatformProcess::setPriority(int priority)
{
  this->priority = priority;
  if(processBase)
    processBase->setPriority(priority);
}
