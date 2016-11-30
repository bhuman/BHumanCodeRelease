/**
 * @file Platform/Nao/NaoBody.cpp
 * Declaration of a class for accessing the body of the nao via NaoQi/libbhuman.
 * @author Colin Graf
 * @author jeff
 */

#include <sys/mman.h>
#include <fcntl.h>
#include <semaphore.h>
#include <unistd.h>
#include <cerrno>
#include <cstdio>

#include "NaoBody.h"
#include "Platform/BHAssert.h"
#include "Platform/Time.h"

#include "libbhuman/bhuman.h"

class NaoBodyAccess
{
public:
  int fd = -1; /**< The file descriptor for the shared memory block. */
  sem_t* sem = SEM_FAILED; /**< The semaphore used for synchronizing to the NaoQi DCM. */
  LBHData* lbhData = (LBHData*)MAP_FAILED; /**< The pointer to the mapped shared memory block. */

  ~NaoBodyAccess()
  {
    cleanup();
  }

  bool init()
  {
    if(lbhData != MAP_FAILED)
      return true;

    fd = shm_open(LBH_MEM_NAME, O_RDWR, S_IRUSR | S_IWUSR);
    if(fd == -1)
      return false;

    sem = sem_open(LBH_SEM_NAME, O_RDWR, S_IRUSR | S_IWUSR, 0);
    if(sem == SEM_FAILED)
    {
      close(fd);
      fd = -1;
      return false;
    }

    VERIFY((lbhData = (LBHData*)mmap(nullptr, sizeof(LBHData), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0)) != MAP_FAILED);
    lbhData->state = okState;
    return true;
  }

  void cleanup()
  {
    if(lbhData != MAP_FAILED)
    {
      munmap(lbhData, sizeof(LBHData));
      lbhData = (LBHData*)MAP_FAILED;
    }
    if(fd != -1)
    {
      close(fd);
      fd = -1;
    }
    if(sem != SEM_FAILED)
    {
      sem_close(sem);
      sem = SEM_FAILED;
    }
  }
} naoBodyAccess;

NaoBody::~NaoBody()
{
  if(fdCpuTemp)
    fclose(fdCpuTemp);
}

bool NaoBody::init()
{
  return naoBodyAccess.init();
}

void NaoBody::cleanup()
{
  naoBodyAccess.cleanup();
}

void NaoBody::setCrashed(int termSignal)
{
  ASSERT(naoBodyAccess.lbhData != (LBHData*)MAP_FAILED);
  naoBodyAccess.lbhData->state = BHState(termSignal);
}

bool NaoBody::wait()
{
  ASSERT(naoBodyAccess.lbhData != (LBHData*)MAP_FAILED);
  ASSERT(naoBodyAccess.sem != SEM_FAILED);
  do
  {
    if(sem_wait(naoBodyAccess.sem) == -1)
    {
      bool success = false;
      while(errno == 516)
      {
        if(sem_wait(naoBodyAccess.sem) == -1)
        {
          ASSERT(false);
          continue;
        }
        else
        {
          success = true;
          break;
        }
      }
      if(!success)
      {
        ASSERT(false);
        return false;
      }
    }
  }
  while(naoBodyAccess.lbhData->readingSensors == naoBodyAccess.lbhData->newestSensors);
  naoBodyAccess.lbhData->readingSensors = naoBodyAccess.lbhData->newestSensors;

  static bool shout = true;
  if(shout)
  {
    shout = false;
    printf("NaoQi is working\n");
  }

  return true;
}

const char* NaoBody::getHeadId() const
{
  ASSERT(naoBodyAccess.lbhData != (LBHData*)MAP_FAILED);
  return naoBodyAccess.lbhData->headId;
}

const char* NaoBody::getBodyId() const
{
  ASSERT(naoBodyAccess.lbhData != (LBHData*)MAP_FAILED);
  return naoBodyAccess.lbhData->bodyId;
}

RobotInfo::NaoVersion NaoBody::getHeadVersion()
{
  ASSERT(naoBodyAccess.lbhData != (LBHData*)MAP_FAILED);
  return static_cast<RobotInfo::NaoVersion>(naoBodyAccess.lbhData->headVersion);
}

RobotInfo::NaoVersion NaoBody::getBodyVersion()
{
  ASSERT(naoBodyAccess.lbhData != (LBHData*)MAP_FAILED);
  return static_cast<RobotInfo::NaoVersion>(naoBodyAccess.lbhData->bodyVersion);
}

RobotInfo::NaoType NaoBody::getHeadType()
{
  ASSERT(naoBodyAccess.lbhData != (LBHData*)MAP_FAILED);
  return static_cast<RobotInfo::NaoType>(naoBodyAccess.lbhData->headType);
}

RobotInfo::NaoType NaoBody::getBodyType()
{
  ASSERT(naoBodyAccess.lbhData != (LBHData*)MAP_FAILED);
  return static_cast<RobotInfo::NaoType>(naoBodyAccess.lbhData->bodyType);
}

float* NaoBody::getSensors()
{
  ASSERT(naoBodyAccess.lbhData != (LBHData*)MAP_FAILED);
  return naoBodyAccess.lbhData->sensors[naoBodyAccess.lbhData->readingSensors];
}

const RoboCup::RoboCupGameControlData& NaoBody::getGameControlData() const
{
  ASSERT(naoBodyAccess.lbhData != (LBHData*)MAP_FAILED);
  return naoBodyAccess.lbhData->gameControlData[naoBodyAccess.lbhData->readingSensors];
}

float NaoBody::getCPUTemperature()
{
  float cpu = 0;

  if(!fdCpuTemp)
  {
    fdCpuTemp = fopen("/proc/acpi/thermal_zone/THRM/temperature", "r");
    ASSERT(fdCpuTemp);
  }

  if(fdCpuTemp)
  {
    VERIFY(fseek(fdCpuTemp, 20, SEEK_SET) == 0);
    VERIFY(fscanf(fdCpuTemp, "%f", &cpu) == 1);
  }

  return cpu;
}

bool NaoBody::getWlanStatus()
{
  return access("/sys/class/net/wlan0", F_OK) == 0;
}

void NaoBody::openActuators(float*& actuators)
{
  ASSERT(naoBodyAccess.lbhData != (LBHData*)MAP_FAILED);
  ASSERT(writingActuators == -1);
  writingActuators = 0;
  if(writingActuators == naoBodyAccess.lbhData->newestActuators)
    ++writingActuators;
  if(writingActuators == naoBodyAccess.lbhData->readingActuators)
    if(++writingActuators == naoBodyAccess.lbhData->newestActuators)
      ++writingActuators;
  ASSERT(writingActuators != naoBodyAccess.lbhData->newestActuators);
  ASSERT(writingActuators != naoBodyAccess.lbhData->readingActuators);
  actuators = naoBodyAccess.lbhData->actuators[writingActuators];
}

void NaoBody::closeActuators()
{
  ASSERT(naoBodyAccess.lbhData != (LBHData*)MAP_FAILED);
  ASSERT(writingActuators >= 0);
  naoBodyAccess.lbhData->newestActuators = writingActuators;
  writingActuators = -1;
}

void NaoBody::setTeamInfo(int teamNumber, int teamColor, int playerNumber)
{
  ASSERT(naoBodyAccess.lbhData != (LBHData*)MAP_FAILED);
  naoBodyAccess.lbhData->teamInfo[::teamNumber] = teamNumber;
  naoBodyAccess.lbhData->teamInfo[::teamColor] = teamColor;
  naoBodyAccess.lbhData->teamInfo[::playerNumber] = playerNumber;
  naoBodyAccess.lbhData->bhumanStartTime = Time::getSystemTimeBase() | 1; // make sure it's non zero
}
