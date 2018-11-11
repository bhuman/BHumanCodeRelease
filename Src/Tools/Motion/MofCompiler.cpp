/**
 * @file MofCompiler.cpp
 *
 * This file implements all functions required to compile the motion net for special actions.
 *
 * @author Uwe Düffert
 * @author Martin Lötzsch
 */

#include "MofCompiler.h"

#include "Platform/File.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/StiffnessData.h"

#include <fstream>
#include <cstdarg>
#include <cstring>
#include <dirent.h>
#include <string>

#if defined LINUX || defined _CYGWIN_ || defined MACOS
#define _strdup strdup
#define _vsnprintf vsnprintf
#endif

MofCompiler::~MofCompiler()
{
  for(int i = 0; i < numOfLines; ++i)
    free(line_data[i]);

  for(int i = 0; i < numOfFiles; ++i)
    free(file_name[i]);

  for(int i = 0; i < numOfLabels; ++i)
  {
    free(label_name[i]);
    free(label_motion[i]);
  }
}

int MofCompiler::myprintf(const char* format, ...)
{
  va_list args;
  va_start(args, format);
  size_t size = strlen(printBuffer);
  return _vsnprintf(printBuffer + size, printBufferSize - size, format, args);
}

#define printf myprintf

bool MofCompiler::generateMotionNet(std::vector<float>& motionData)
{
  for(int i = 0; i < SpecialActionRequest::numOfSpecialActionIDs; ++i)
    motionData.push_back(static_cast<float>(jumpTable[i]));

  motionData.push_back(static_cast<float>(numOfLines));

  int labelnum = 0;
  int filenum = 0;
  for(int i = 0; i < numOfLines; ++i)
  {
    if(file_startindex[filenum] == i)
      ++filenum;
    if(label_number[labelnum] == i)
      ++labelnum;
    if(!strncmp(line_data[i], "transition", 10))
    {
      char request[512];
      char viamotion[512];
      char vialabel[512];
      sscanf(line_data[i] + 11, "%s %s %s", request, viamotion, vialabel);
      //check if label exists (request was already checked)

      bool found = false;
      int j;
      for(j = 0; j < numOfLabels; ++j)
        if(!strcmp(label_motion[j], viamotion) && !strcmp(label_name[j], vialabel))
        {
          found = true;
          break;
        }

      if(!found)
        for(int k = numOfFiles - 1; k >= 0; --k)
          if(file_startindex[k] <= i)
          {
            printf("%s(%i) : error: jump label unknown\n", file_name[k], line_number[i]);
            return false;
          }

      if(strcmp(request, "allMotions"))
      {
        motionData.push_back(1);
        motionData.push_back(label_number[j]);
        motionData.push_back(SpecialActionRequest::getSpecialActionFromName(request));
      }
      else
      {
        motionData.push_back(2);
        motionData.push_back(static_cast<float>(label_number[j]));
      }
    }
    else if(!strncmp(line_data[i], "stiffness", 9))
    {
      motionData.push_back(4);
      char* p = line_data[i] + 10;
      while(*p)
      {
        motionData.push_back(std::stof(p));

        while(*p && *p != ' ')
          ++p;
        if(*p)
          ++p;
      }
    }
    else
    {
      motionData.push_back(3);

      char* p = line_data[i];
      while(*p)
      {
        motionData.push_back(std::stof(p));

        while(*p && *p != ' ')
          ++p;
        if(*p)
          ++p;
      }
    }

    if(line_motionID[i] >= 0)
    {
      motionData.push_back(line_motionID[i]);
    }
    else
    {
      motionData.push_back(-1);
    }
  }

  return true;
}

bool MofCompiler::parseMofs()
{
  numOfFiles = 0;
  dirent* ff = 0;
  DIR* fd;

  char ffname[1024];
  sprintf(ffname, "%s/Config/mof", File::getBHDir());
  bool thereAreMore;
  if((fd = opendir(ffname)))
  {
    do
    {
      ff = readdir(fd);
      thereAreMore = ff != nullptr;
    }
    while(thereAreMore && (strlen(ff->d_name) <= 4 || strcmp(ff->d_name + strlen(ff->d_name) - 4, ".mof") != 0));
  }
  else
  {
    thereAreMore = false;
  }
  while(thereAreMore)
  {
    if(strcmp(ff->d_name, "extern.mof"))
    {
      char name[512];
      sprintf(name, "%s/Config/mof/%s", File::getBHDir(), ff->d_name);
      FILE* f = fopen(name, "r");
      if(f == nullptr)
      {
        printf("error opening %s. Aborting.\n", name);
        return false;
      }
      else
      {
        file_name[numOfFiles] = _strdup(name);
        file_startindex[numOfFiles++] = static_cast<short>(numOfLines);
        bool thisMofHasLabels = false;
        strcpy(motion, ff->d_name);
        if(!strcmp(motion + strlen(motion) - 4, ".mof"))
          motion[strlen(motion) - 4] = 0;
        actMotionID = TypeRegistry::getEnumValue(typeid(SpecialActionRequest::SpecialActionID).name(), std::string(motion));

        char s[128000];
        size_t siz = fread(s, 1, 128000, f);
        fclose(f);
        if(siz > 0)
        {
          s[siz] = 0;
          char* t = &s[0];
          int line = 1;
          while(*t)
          {
            char* u = strchr(t, '\n');
            if(u >= t)
            {
              *u = 0;

              char sval[Joints::numOfJoints + 3][256]; // joints + interpolate + duration + bad argument
              int c = sscanf(t, "%s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s", // (numOfJoints + 3) * %s
                             sval[0], sval[1], sval[2], sval[3], sval[4], sval[5], sval[6], sval[7],
                             sval[8], sval[9], sval[10], sval[11], sval[12], sval[13], sval[14], sval[15],
                             sval[16], sval[17], sval[18], sval[19], sval[20], sval[21], sval[22], sval[23],
                             sval[24], sval[25], sval[26], sval[27], sval[28]); // sval[0]..sval[numOfJoints + 2]
              if(c == -1 || sval[0][0] == '"' || !strncmp(sval[0], "//", 2))
              {
                //skip comments and empty lines by doing nothing
              }
              else if(!strcmp(sval[0], "motion_id"))
              {
                //if there is a motion_id we use it, otherwise filename
                if(!strcmp(sval[1], "=") && c == 3)
                {
                  strcpy(motion, sval[2]);
                  actMotionID = TypeRegistry::getEnumValue(typeid(SpecialActionRequest::SpecialActionID).name(), std::string(motion));
                }
                else
                {
                  printf("%s(%i) : error: motion_id format\n", name, line);
                  return false;
                }
              }
              else if(!strcmp(sval[0], "transition"))
              {
                bool found = false;
                if(!thisMofHasLabels)
                {
                  printf("%s(%i) : error: this line is unreachable, because there was no label before\n", name, line);
                  return false;
                }
                else if(!strcmp(sval[1], "allMotions"))
                  found = true;
                else
                  found = TypeRegistry::getEnumValue(typeid(SpecialActionRequest::SpecialActionID).name(), std::string(sval[1])) >= 0;

                if(!found)
                {
                  printf("%s(%i) : error: request for transition unknown\n", name, line);
                  return false;
                }

                if(c == 4)
                {
                  line_motionID[numOfLines] = static_cast<short>(actMotionID);
                  line_data[numOfLines] = _strdup(t);
                  line_number[numOfLines++] = static_cast<short>(line);
                }
                else
                {
                  printf("%s(%i) : error: transition format\n", name, line);
                  return false;
                }
              }
              else if(!strcmp(sval[0], "label"))
              {
                if(c == 2)
                {
                  label_motion[numOfLabels] = _strdup(motion);
                  label_name[numOfLabels] = _strdup(sval[1]);
                  label_number[numOfLabels++] = static_cast<short>(numOfLines);
                  thisMofHasLabels = true;
                }
                else
                {
                  printf("%s(%i) : error: label format\n", name, line);
                  return false;
                }
              }
              else if(!strcmp(sval[0], "stiffness"))
              {
                if(!thisMofHasLabels)
                {
                  printf("%s(%i) : error: this line is unreachable, because there was no label before\n", name, line);
                  return false;
                }
                if(c == Joints::numOfJoints + 2)
                {
                  char temp[1024];
                  temp[0] = 0;
                  strcat(temp, "stiffness");
                  int val = 0;

                  for(int j = 1; j < Joints::numOfJoints + 1; j++)
                  {
                    if(!strcmp(sval[j], "*"))
                      sprintf(sval[j], "%i", StiffnessData::useDefault);

                    if((!strcmp(sval[j], "-1") || sscanf(sval[j], "%i", &val) == 1) && val >= 0 && val <= 100)
                    {
                      strcat(temp, " ");
                      strcat(temp, sval[j]);
                    }
                    else
                    {
                      printf("%s(%i) : error: stiffness data format\n", name, line);
                      return false;
                    }
                  }
                  strcat(temp, " ");
                  strcat(temp, sval[Joints::numOfJoints + 1]);

                  line_motionID[numOfLines] = static_cast<short>(actMotionID);
                  line_data[numOfLines] = _strdup(temp);
                  line_number[numOfLines++] = static_cast<short>(line);
                }
                else
                {
                  printf("%s(%i) : error: stiffness format\n", name, line);
                  return false;
                }
              }
              else if(c == Joints::numOfJoints + 2)
              {
                char temp[1024];
                temp[0] = 0;
                int val = 0;
                if(!thisMofHasLabels)
                {
                  printf("%s(%i) : error: this line is unreachable, because there was no label before\n", name, line);
                  return false;
                }

                for(int j = 0; j < Joints::numOfJoints; ++j)
                {
                  if(!strcmp(sval[j], "*"))
                    sprintf(sval[j], "%i", JointAngles::ignore);
                  if(!strcmp(sval[j], "-"))
                    sprintf(sval[j], "%i", JointAngles::off);

                  if((!strcmp(sval[j], "10000") || !strcmp(sval[j], "20000") || sscanf(sval[j], "%i", &val) == 1) && val >= -210 && val <= 210)
                  {
                    if(j != 0)
                      strcat(temp, " ");
                    strcat(temp, sval[j]);
                  }
                  else
                  {
                    printf("%s(%i) : error: joint data format\n", name, line);
                    return false;
                  }
                }
                if(sscanf(sval[Joints::numOfJoints], "%i", &val) == 1 && (val >= 0 || val <= 3))
                {
                  strcat(temp, " ");
                  strcat(temp, sval[Joints::numOfJoints]);
                }
                else
                {
                  printf("%s(%i) : error: interpolation data format\n", name, line);
                  return false;
                }
                if(sscanf(sval[Joints::numOfJoints + 1], "%i", &val) == 1 && val > 0)
                {
                  strcat(temp, " ");
                  strcat(temp, sval[Joints::numOfJoints + 1]);
                }
                else
                {
                  printf("%s(%i) : error: time data format\n", name, line);
                  return false;
                }
                line_motionID[numOfLines] = static_cast<short>(actMotionID);
                line_data[numOfLines] = _strdup(temp);
                line_number[numOfLines++] = static_cast<short>(line);
              }
              else
              {
                printf("%s(%i) : error: illegal number of arguments\n", name, line);
                return false;
              }
              ++line;
              t = u + 1;
            }
            else
              t += strlen(t);
          }
        }
        else
        {
          printf("error reading from %s. Aborting.\n", name);
          return false;
        }
      }
    }
    if(fd)
    {
      do
      {
        ff = readdir(fd);
        thereAreMore = ff != nullptr;
      }
      while(thereAreMore && (strlen(ff->d_name) <= 4 || strcmp(ff->d_name + strlen(ff->d_name) - 4, ".mof") != 0));
    }
    else
    {
      thereAreMore = false;
    }
  }

  if(fd)
    closedir(fd);

  return true;
}

bool MofCompiler::parseExternMof()
{
  for(int i = 0; i < SpecialActionRequest::numOfSpecialActionIDs; ++i)
    jumpTable[i] = -1;

  char name[512];
  sprintf(name, "%s/Config/mof/extern.mof", File::getBHDir());
  FILE* f = fopen(name, "r");
  if(f == nullptr)
  {
    printf("error opening %s. Aborting.\n", name);
    return false;
  }
  else
  {
    char s[128000];
    size_t siz = fread(s, 1, 128000, f);
    fclose(f);
    if(siz > 0)
    {
      s[siz] = 0;
      char* t = &s[0];
      int line = 1;
      while(*t)
      {
        char* u = strchr(t, '\n');
        if(u >= t)
        {
          *u = 0;

          char sval[5][256];
          int c = sscanf(t, "%s %s %s %s %s", sval[0], sval[1], sval[2], sval[3], sval[4]);

          if(c == -1 || sval[0][0] == '"' || !strncmp(sval[0], "//", 2))
          {
            //skip comments and empty lines by doing nothing
          }
          else if(!strcmp(sval[0], "motion_id") || !strcmp(sval[0], "label"))
          {
            //skip labels and id
          }
          else if(!strcmp(sval[0], "transition") && c == 4)
          {
            bool found = false;
            if(!strcmp(sval[1], "extern"))
              continue;
            else
            {
              int i;
              for(i = 0; i < SpecialActionRequest::numOfSpecialActionIDs; ++i)
                if(!strcmp(TypeRegistry::getEnumName(SpecialActionRequest::SpecialActionID(i)), sval[1]))
                {
                  found = true;
                  break;
                }

              if(!found)
              {
                printf("%s(%i) : error: special action '%s' unknown (not in SpecialActionRequest.h)\n", name, line, sval[1]);
                return false;
              }

              for(int j = 0; j < numOfLabels; ++j)
                if(!strcmp(label_motion[j], sval[2]) && !strcmp(label_name[j], sval[3]))
                {
                  jumpTable[i] = label_number[j];
                  found = true;
                  break;
                }

              if(!found)
              {
                printf("%s(%i) : error: jump label '%s:%s' unknown (not in *.mof)\n", name, line, sval[2], sval[3]);
                return false;
              }
            }
          }
          else
          {
            printf("%s(%i) : error: unexpected data\n", name, line);
            return false;
          }
          ++line;
          t = u + 1;
        }
        else
          t += strlen(t);
      }
    }
    else
    {
      printf("error reading from %s. Aborting.\n", name);
      return false;
    }
  }

  for(int i = 0; i < SpecialActionRequest::numOfSpecialActionIDs; ++i)
    if(jumpTable[i] == -1)
    {
      printf("%s/Config/mof/extern.mof(1): error: no motion net entry defined for special action %s\n",
             File::getBHDir(), TypeRegistry::getEnumName(SpecialActionRequest::SpecialActionID(i)));
      return false;
    }
  return true;
}

bool MofCompiler::compileMofs(char* buffer, size_t size, std::vector<float>& motionData)
{
  printBuffer = buffer;
  printBufferSize = size;
  printBuffer[0] = 0;

  actMotionID = -1;

  numOfLabels = 0;
  numOfLines = 0;
  numOfFiles = 0;

  std::memset(label_number, 0, sizeof(label_number));
  std::memset(motion, 0, sizeof(motion));
  std::memset(line_number, 0, sizeof(line_number));
  std::memset(line_motionID, 0, sizeof(line_motionID));
  std::memset(file_startindex, 0, sizeof(file_startindex));

  label_motion[numOfLabels] = _strdup("extern");
  label_name[numOfLabels] = _strdup("start");
  label_number[numOfLabels++] = static_cast<short>(numOfLines);

  line_data[numOfLines] = _strdup("transition allMotions extern start");
  line_number[numOfLines++] = 1;

  if(!parseMofs() || !parseExternMof())
    return false;

  //check whether the last line of every mof is an unconditional jump
  for(int i = 1; i <= numOfFiles; ++i)
  {
    int ind = i < numOfFiles ? file_startindex[i] - 1 : numOfLines - 1;
    if(strncmp(line_data[ind], "transition allMotions", 21))
    {
      printf("%s(%i) : error: this mof does not end with a transition for allMotions\n", file_name[i - 1], line_number[ind]);
      return false;
    }
  }

  //check whether there are mofs without SpecialActionRequest::SpecialAction
  for(int i = 1; i < numOfLabels; ++i)
  {
    if(!strcmp(label_motion[i], label_motion[i - 1]))
      continue;
    bool found = false;
    for(int j = 0; j < SpecialActionRequest::numOfSpecialActionIDs; ++j)
      if(!strcmp(label_motion[i], TypeRegistry::getEnumName(SpecialActionRequest::SpecialActionID(j))))
      {
        found = true;
        break;
      }
    if(!found)
      for(int j = numOfFiles - 1; j >= 0; --j)
        if(file_startindex[j] <= label_number[i])
        {
          printf("%s(%i) : warning: there is no special action id for this mof\n", file_name[j], 1);
          break;
        }
  }

  /** @todo warning for motions without labels */

  if(!generateMotionNet(motionData))
    return false;

  return true;
}
