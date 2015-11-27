/**
 * @file Tools/Enum.cpp
 * Implements a function that converts a single comma-separated string
 * of enum names into single entries that can be accessed in
 * constant time.
 * @author Thomas Röfer
 */

#include "Enum.h"
#include "Platform/BHAssert.h"
#include "Platform/Thread.h"
#include <cstring>

static SyncObject _syncObject;

static char* trim(char* pBegin, char* pEnd)
{
  while(*pBegin == ' ')
    ++pBegin;
  while(pEnd > pBegin && pEnd[-1] == ' ')
    --pEnd;
  *pEnd = 0;
  return pBegin;
}

void enumInit(char* enums, const char** names, int numOfEnums)
{
  SYNC;
  char* pEnd = enums - 1;
  int index = 0;
  bool end;
  do
  {
    char* pBegin = pEnd + 1;
    pEnd = strchr(pBegin, ',');
    end = !pEnd;
    if(end)
      pEnd = pBegin + strlen(pBegin);
    char* name = trim(pBegin, pEnd);
    char* p = strchr(name, '=');
    if(p)
    {
      ASSERT(index && !strcmp(trim(p + 1, pEnd), names[index - 1]));
      name = trim(name, p);
      --index;
    }
    ASSERT(index < numOfEnums);
    names[index++] = name;
  } while(!end);
}
