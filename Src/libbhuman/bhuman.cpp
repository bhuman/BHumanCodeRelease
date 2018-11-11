/**
 * @file bhuman.cpp
 * Implementation of a NaoQi module that provides basic ipc NaoQi DCM access via semaphore and shared memory.
 */

#include <sys/types.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <semaphore.h>
#include <csignal>
#include <sys/resource.h>
#include <ctime>
#include <cstring>

#ifdef __clang__
#pragma clang diagnostic push

#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma clang diagnostic ignored "-Wunknown-warning-option"
#pragma clang diagnostic ignored "-Wconversion"
#pragma clang diagnostic ignored "-Wunused-variable"
#pragma clang diagnostic ignored "-Wunused-local-typedef"
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
#endif
#define BOOST_SIGNALS_NO_DEPRECATION_WARNING
#include <alcommon/albroker.h>
#include <alcommon/alproxy.h>
#include <alproxies/dcmproxy.h>
#include <alproxies/almemoryproxy.h>
#undef BOOST_SIGNALS_NO_DEPRECATION_WARNING
#ifdef __clang__
#pragma clang diagnostic pop
#endif

#include "bhuman.h"

static const char* sensorNames[] =
{
  // joint sensors
  "Device/SubDeviceList/HeadYaw/Position/Sensor/Value",
  "Device/SubDeviceList/HeadYaw/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/HeadYaw/Temperature/Sensor/Value",
  "Device/SubDeviceList/HeadYaw/Temperature/Sensor/Status",
  "Device/SubDeviceList/HeadPitch/Position/Sensor/Value",
  "Device/SubDeviceList/HeadPitch/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/HeadPitch/Temperature/Sensor/Value",
  "Device/SubDeviceList/HeadPitch/Temperature/Sensor/Status",
  "Device/SubDeviceList/LShoulderPitch/Position/Sensor/Value",
  "Device/SubDeviceList/LShoulderPitch/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/LShoulderPitch/Temperature/Sensor/Value",
  "Device/SubDeviceList/LShoulderPitch/Temperature/Sensor/Status",
  "Device/SubDeviceList/LShoulderRoll/Position/Sensor/Value",
  "Device/SubDeviceList/LShoulderRoll/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/LShoulderRoll/Temperature/Sensor/Value",
  "Device/SubDeviceList/LShoulderRoll/Temperature/Sensor/Status",
  "Device/SubDeviceList/LElbowYaw/Position/Sensor/Value",
  "Device/SubDeviceList/LElbowYaw/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/LElbowYaw/Temperature/Sensor/Value",
  "Device/SubDeviceList/LElbowYaw/Temperature/Sensor/Status",
  "Device/SubDeviceList/LElbowRoll/Position/Sensor/Value",
  "Device/SubDeviceList/LElbowRoll/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/LElbowRoll/Temperature/Sensor/Value",
  "Device/SubDeviceList/LElbowRoll/Temperature/Sensor/Status",
  "Device/SubDeviceList/LWristYaw/Position/Sensor/Value",
  "Device/SubDeviceList/LWristYaw/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/LWristYaw/Temperature/Sensor/Value",
  "Device/SubDeviceList/LWristYaw/Temperature/Sensor/Status",
  "Device/SubDeviceList/LHand/Position/Sensor/Value",
  "Device/SubDeviceList/LHand/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/LHand/Temperature/Sensor/Value",
  "Device/SubDeviceList/LHand/Temperature/Sensor/Status",
  "Device/SubDeviceList/RShoulderPitch/Position/Sensor/Value",
  "Device/SubDeviceList/RShoulderPitch/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/RShoulderPitch/Temperature/Sensor/Value",
  "Device/SubDeviceList/RShoulderPitch/Temperature/Sensor/Status",
  "Device/SubDeviceList/RShoulderRoll/Position/Sensor/Value",
  "Device/SubDeviceList/RShoulderRoll/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/RShoulderRoll/Temperature/Sensor/Value",
  "Device/SubDeviceList/RShoulderRoll/Temperature/Sensor/Status",
  "Device/SubDeviceList/RElbowYaw/Position/Sensor/Value",
  "Device/SubDeviceList/RElbowYaw/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/RElbowYaw/Temperature/Sensor/Value",
  "Device/SubDeviceList/RElbowYaw/Temperature/Sensor/Status",
  "Device/SubDeviceList/RElbowRoll/Position/Sensor/Value",
  "Device/SubDeviceList/RElbowRoll/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/RElbowRoll/Temperature/Sensor/Value",
  "Device/SubDeviceList/RElbowRoll/Temperature/Sensor/Status",
  "Device/SubDeviceList/RWristYaw/Position/Sensor/Value",
  "Device/SubDeviceList/RWristYaw/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/RWristYaw/Temperature/Sensor/Value",
  "Device/SubDeviceList/RWristYaw/Temperature/Sensor/Status",
  "Device/SubDeviceList/RHand/Position/Sensor/Value",
  "Device/SubDeviceList/RHand/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/RHand/Temperature/Sensor/Value",
  "Device/SubDeviceList/RHand/Temperature/Sensor/Status",
  "Device/SubDeviceList/LHipYawPitch/Position/Sensor/Value",
  "Device/SubDeviceList/LHipYawPitch/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/LHipYawPitch/Temperature/Sensor/Value",
  "Device/SubDeviceList/LHipYawPitch/Temperature/Sensor/Status",
  "Device/SubDeviceList/LHipRoll/Position/Sensor/Value",
  "Device/SubDeviceList/LHipRoll/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/LHipRoll/Temperature/Sensor/Value",
  "Device/SubDeviceList/LHipRoll/Temperature/Sensor/Status",
  "Device/SubDeviceList/LHipPitch/Position/Sensor/Value",
  "Device/SubDeviceList/LHipPitch/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/LHipPitch/Temperature/Sensor/Value",
  "Device/SubDeviceList/LHipPitch/Temperature/Sensor/Status",
  "Device/SubDeviceList/LKneePitch/Position/Sensor/Value",
  "Device/SubDeviceList/LKneePitch/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/LKneePitch/Temperature/Sensor/Value",
  "Device/SubDeviceList/LKneePitch/Temperature/Sensor/Status",
  "Device/SubDeviceList/LAnklePitch/Position/Sensor/Value",
  "Device/SubDeviceList/LAnklePitch/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/LAnklePitch/Temperature/Sensor/Value",
  "Device/SubDeviceList/LAnklePitch/Temperature/Sensor/Status",
  "Device/SubDeviceList/LAnkleRoll/Position/Sensor/Value",
  "Device/SubDeviceList/LAnkleRoll/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/LAnkleRoll/Temperature/Sensor/Value",
  "Device/SubDeviceList/LAnkleRoll/Temperature/Sensor/Status",
  "Device/SubDeviceList/RHipRoll/Position/Sensor/Value",
  "Device/SubDeviceList/RHipRoll/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/RHipRoll/Temperature/Sensor/Value",
  "Device/SubDeviceList/RHipRoll/Temperature/Sensor/Status",
  "Device/SubDeviceList/RHipPitch/Position/Sensor/Value",
  "Device/SubDeviceList/RHipPitch/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/RHipPitch/Temperature/Sensor/Value",
  "Device/SubDeviceList/RHipPitch/Temperature/Sensor/Status",
  "Device/SubDeviceList/RKneePitch/Position/Sensor/Value",
  "Device/SubDeviceList/RKneePitch/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/RKneePitch/Temperature/Sensor/Value",
  "Device/SubDeviceList/RKneePitch/Temperature/Sensor/Status",
  "Device/SubDeviceList/RAnklePitch/Position/Sensor/Value",
  "Device/SubDeviceList/RAnklePitch/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/RAnklePitch/Temperature/Sensor/Value",
  "Device/SubDeviceList/RAnklePitch/Temperature/Sensor/Status",
  "Device/SubDeviceList/RAnkleRoll/Position/Sensor/Value",
  "Device/SubDeviceList/RAnkleRoll/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/RAnkleRoll/Temperature/Sensor/Value",
  "Device/SubDeviceList/RAnkleRoll/Temperature/Sensor/Status",

  // touch sensors
  "Device/SubDeviceList/Head/Touch/Front/Sensor/Value",
  "Device/SubDeviceList/Head/Touch/Middle/Sensor/Value",
  "Device/SubDeviceList/Head/Touch/Rear/Sensor/Value",
  "Device/SubDeviceList/LHand/Touch/Back/Sensor/Value",
  "Device/SubDeviceList/LHand/Touch/Left/Sensor/Value",
  "Device/SubDeviceList/LHand/Touch/Right/Sensor/Value",
  "Device/SubDeviceList/RHand/Touch/Back/Sensor/Value",
  "Device/SubDeviceList/RHand/Touch/Left/Sensor/Value",
  "Device/SubDeviceList/RHand/Touch/Right/Sensor/Value",

  // switches
  "Device/SubDeviceList/LFoot/Bumper/Left/Sensor/Value",
  "Device/SubDeviceList/LFoot/Bumper/Right/Sensor/Value",
  "Device/SubDeviceList/RFoot/Bumper/Left/Sensor/Value",
  "Device/SubDeviceList/RFoot/Bumper/Right/Sensor/Value",
  "Device/SubDeviceList/ChestBoard/Button/Sensor/Value",

  // inertial sensors
  "Device/SubDeviceList/InertialSensor/GyroscopeX/Sensor/Value",
  "Device/SubDeviceList/InertialSensor/GyroscopeY/Sensor/Value",
  "Device/SubDeviceList/InertialSensor/GyroscopeZ/Sensor/Value",
  "Device/SubDeviceList/InertialSensor/AccelerometerX/Sensor/Value",
  "Device/SubDeviceList/InertialSensor/AccelerometerY/Sensor/Value",
  "Device/SubDeviceList/InertialSensor/AccelerometerZ/Sensor/Value",
  "Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value",
  "Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value",
  "Device/SubDeviceList/InertialSensor/AngleZ/Sensor/Value",

  // battery sensors
  "Device/SubDeviceList/Battery/Current/Sensor/Value",
  "Device/SubDeviceList/Battery/Charge/Sensor/Value",
  "Device/SubDeviceList/Battery/Charge/Sensor/Status",
  "Device/SubDeviceList/Battery/Temperature/Sensor/Value",
  "Device/SubDeviceList/Battery/Temperature/Sensor/Status",

  // fsr sensors
  "Device/SubDeviceList/LFoot/FSR/FrontLeft/Sensor/Value",
  "Device/SubDeviceList/LFoot/FSR/FrontRight/Sensor/Value",
  "Device/SubDeviceList/LFoot/FSR/RearLeft/Sensor/Value",
  "Device/SubDeviceList/LFoot/FSR/RearRight/Sensor/Value",
  "Device/SubDeviceList/RFoot/FSR/FrontLeft/Sensor/Value",
  "Device/SubDeviceList/RFoot/FSR/FrontRight/Sensor/Value",
  "Device/SubDeviceList/RFoot/FSR/RearLeft/Sensor/Value",
  "Device/SubDeviceList/RFoot/FSR/RearRight/Sensor/Value",
  "Device/SubDeviceList/LFoot/FSR/TotalWeight/Sensor/Value",
  "Device/SubDeviceList/RFoot/FSR/TotalWeight/Sensor/Value",
};

static const char* actuatorNames[] =
{
  "HeadYaw/Position/Actuator/Value",
  "HeadPitch/Position/Actuator/Value",
  "LShoulderPitch/Position/Actuator/Value",
  "LShoulderRoll/Position/Actuator/Value",
  "LElbowYaw/Position/Actuator/Value",
  "LElbowRoll/Position/Actuator/Value",
  "LWristYaw/Position/Actuator/Value",
  "LHand/Position/Actuator/Value",
  "RShoulderPitch/Position/Actuator/Value",
  "RShoulderRoll/Position/Actuator/Value",
  "RElbowYaw/Position/Actuator/Value",
  "RElbowRoll/Position/Actuator/Value",
  "RWristYaw/Position/Actuator/Value",
  "RHand/Position/Actuator/Value",
  "LHipYawPitch/Position/Actuator/Value",
  "LHipRoll/Position/Actuator/Value",
  "LHipPitch/Position/Actuator/Value",
  "LKneePitch/Position/Actuator/Value",
  "LAnklePitch/Position/Actuator/Value",
  "LAnkleRoll/Position/Actuator/Value",
  "RHipRoll/Position/Actuator/Value",
  "RHipPitch/Position/Actuator/Value",
  "RKneePitch/Position/Actuator/Value",
  "RAnklePitch/Position/Actuator/Value",
  "RAnkleRoll/Position/Actuator/Value",

  "HeadYaw/Hardness/Actuator/Value",
  "HeadPitch/Hardness/Actuator/Value",
  "LShoulderPitch/Hardness/Actuator/Value",
  "LShoulderRoll/Hardness/Actuator/Value",
  "LElbowYaw/Hardness/Actuator/Value",
  "LElbowRoll/Hardness/Actuator/Value",
  "LWristYaw/Hardness/Actuator/Value",
  "LHand/Hardness/Actuator/Value",
  "RShoulderPitch/Hardness/Actuator/Value",
  "RShoulderRoll/Hardness/Actuator/Value",
  "RElbowYaw/Hardness/Actuator/Value",
  "RElbowRoll/Hardness/Actuator/Value",
  "RWristYaw/Hardness/Actuator/Value",
  "RHand/Hardness/Actuator/Value",
  "LHipYawPitch/Hardness/Actuator/Value",
  "LHipRoll/Hardness/Actuator/Value",
  "LHipPitch/Hardness/Actuator/Value",
  "LKneePitch/Hardness/Actuator/Value",
  "LAnklePitch/Hardness/Actuator/Value",
  "LAnkleRoll/Hardness/Actuator/Value",
  "RHipRoll/Hardness/Actuator/Value",
  "RHipPitch/Hardness/Actuator/Value",
  "RKneePitch/Hardness/Actuator/Value",
  "RAnklePitch/Hardness/Actuator/Value",
  "RAnkleRoll/Hardness/Actuator/Value",

  "Face/Led/Red/Left/0Deg/Actuator/Value",
  "Face/Led/Red/Left/45Deg/Actuator/Value",
  "Face/Led/Red/Left/90Deg/Actuator/Value",
  "Face/Led/Red/Left/135Deg/Actuator/Value",
  "Face/Led/Red/Left/180Deg/Actuator/Value",
  "Face/Led/Red/Left/225Deg/Actuator/Value",
  "Face/Led/Red/Left/270Deg/Actuator/Value",
  "Face/Led/Red/Left/315Deg/Actuator/Value",
  "Face/Led/Green/Left/0Deg/Actuator/Value",
  "Face/Led/Green/Left/45Deg/Actuator/Value",
  "Face/Led/Green/Left/90Deg/Actuator/Value",
  "Face/Led/Green/Left/135Deg/Actuator/Value",
  "Face/Led/Green/Left/180Deg/Actuator/Value",
  "Face/Led/Green/Left/225Deg/Actuator/Value",
  "Face/Led/Green/Left/270Deg/Actuator/Value",
  "Face/Led/Green/Left/315Deg/Actuator/Value",
  "Face/Led/Blue/Left/0Deg/Actuator/Value",
  "Face/Led/Blue/Left/45Deg/Actuator/Value",
  "Face/Led/Blue/Left/90Deg/Actuator/Value",
  "Face/Led/Blue/Left/135Deg/Actuator/Value",
  "Face/Led/Blue/Left/180Deg/Actuator/Value",
  "Face/Led/Blue/Left/225Deg/Actuator/Value",
  "Face/Led/Blue/Left/270Deg/Actuator/Value",
  "Face/Led/Blue/Left/315Deg/Actuator/Value",
  "Face/Led/Red/Right/0Deg/Actuator/Value",
  "Face/Led/Red/Right/45Deg/Actuator/Value",
  "Face/Led/Red/Right/90Deg/Actuator/Value",
  "Face/Led/Red/Right/135Deg/Actuator/Value",
  "Face/Led/Red/Right/180Deg/Actuator/Value",
  "Face/Led/Red/Right/225Deg/Actuator/Value",
  "Face/Led/Red/Right/270Deg/Actuator/Value",
  "Face/Led/Red/Right/315Deg/Actuator/Value",
  "Face/Led/Green/Right/0Deg/Actuator/Value",
  "Face/Led/Green/Right/45Deg/Actuator/Value",
  "Face/Led/Green/Right/90Deg/Actuator/Value",
  "Face/Led/Green/Right/135Deg/Actuator/Value",
  "Face/Led/Green/Right/180Deg/Actuator/Value",
  "Face/Led/Green/Right/225Deg/Actuator/Value",
  "Face/Led/Green/Right/270Deg/Actuator/Value",
  "Face/Led/Green/Right/315Deg/Actuator/Value",
  "Face/Led/Blue/Right/0Deg/Actuator/Value",
  "Face/Led/Blue/Right/45Deg/Actuator/Value",
  "Face/Led/Blue/Right/90Deg/Actuator/Value",
  "Face/Led/Blue/Right/135Deg/Actuator/Value",
  "Face/Led/Blue/Right/180Deg/Actuator/Value",
  "Face/Led/Blue/Right/225Deg/Actuator/Value",
  "Face/Led/Blue/Right/270Deg/Actuator/Value",
  "Face/Led/Blue/Right/315Deg/Actuator/Value",
  "Ears/Led/Left/36Deg/Actuator/Value",
  "Ears/Led/Left/72Deg/Actuator/Value",
  "Ears/Led/Left/108Deg/Actuator/Value",
  "Ears/Led/Left/144Deg/Actuator/Value",
  "Ears/Led/Left/180Deg/Actuator/Value",
  "Ears/Led/Left/216Deg/Actuator/Value",
  "Ears/Led/Left/252Deg/Actuator/Value",
  "Ears/Led/Left/288Deg/Actuator/Value",
  "Ears/Led/Left/324Deg/Actuator/Value",
  "Ears/Led/Left/0Deg/Actuator/Value",
  "Ears/Led/Right/0Deg/Actuator/Value",
  "Ears/Led/Right/36Deg/Actuator/Value",
  "Ears/Led/Right/72Deg/Actuator/Value",
  "Ears/Led/Right/108Deg/Actuator/Value",
  "Ears/Led/Right/144Deg/Actuator/Value",
  "Ears/Led/Right/180Deg/Actuator/Value",
  "Ears/Led/Right/216Deg/Actuator/Value",
  "Ears/Led/Right/252Deg/Actuator/Value",
  "Ears/Led/Right/288Deg/Actuator/Value",
  "Ears/Led/Right/324Deg/Actuator/Value",
  "ChestBoard/Led/Red/Actuator/Value",
  "ChestBoard/Led/Green/Actuator/Value",
  "ChestBoard/Led/Blue/Actuator/Value",
  "Head/Led/Rear/Left/0/Actuator/Value",
  "Head/Led/Rear/Left/1/Actuator/Value",
  "Head/Led/Rear/Left/2/Actuator/Value",
  "Head/Led/Rear/Right/0/Actuator/Value",
  "Head/Led/Rear/Right/1/Actuator/Value",
  "Head/Led/Rear/Right/2/Actuator/Value",
  "Head/Led/Middle/Right/0/Actuator/Value",
  "Head/Led/Front/Right/0/Actuator/Value",
  "Head/Led/Front/Right/1/Actuator/Value",
  "Head/Led/Front/Left/0/Actuator/Value",
  "Head/Led/Front/Left/1/Actuator/Value",
  "Head/Led/Middle/Left/0/Actuator/Value",
  "LFoot/Led/Red/Actuator/Value",
  "LFoot/Led/Green/Actuator/Value",
  "LFoot/Led/Blue/Actuator/Value",
  "RFoot/Led/Red/Actuator/Value",
  "RFoot/Led/Green/Actuator/Value",
  "RFoot/Led/Blue/Actuator/Value",
};

static const char* teamInfoNames[] =
{
  "GameCtrl/teamNumber",
  "GameCtrl/teamColour",
  "GameCtrl/playerNumber"
};

static const float sitDownAngles[25] =
{
  0.f,
  0.f,

  0.89f,
  0.06f,
  0.26f,
  -0.62f,
  -1.57f,
  0.f,

  0.89f,
  -0.06f,
  -0.26f,
  0.62f,
  1.57f,
  0.f,

  0.f,
  0.f,
  -0.87f,
  2.16f,
  -1.18f,
  0.f,

  0.f,
  -0.87f,
  2.16f,
  -1.18f,
  0.f
};

class BHuman : public AL::ALModule
{
private:
  static BHuman* theInstance; /**< The only instance of this class. */

#ifdef NDEBUG
  static const int allowedFrameDrops = 3; /**< Maximum number of frame drops allowed before Nao sits down. */
#else
  static const int allowedFrameDrops = 6; /**< Maximum number of frame drops allowed before Nao sits down. */
#endif

  int memoryHandle; /**< The file handle of the shared memory. */
  LBHData* data; /**< The shared memory. */
  sem_t* sem; /**< The semaphore used to notify bhuman about new data. */
  AL::DCMProxy* proxy = nullptr;
  AL::ALMemoryProxy* memory = nullptr;
  AL::ALValue positionRequest;
  AL::ALValue stiffnessRequest;
  AL::ALValue ledRequest;
  float* sensorPtrs[lbhNumOfSensorIds]; /** Pointers to where NaoQi stores the current sensor values. */

  int dcmTime = 0; /**< Current dcm time, updated at each onPreProcess call. */

  float requestedActuators[lbhNumOfActuatorIds]; /**< The previous actuator values requested. */

  int lastReadingActuators = -1; /**< The previous actuators read. For detecting frames without seemingly new data from bhuman. */
  int actuatorDrops = 0; /**< The number of frames without seemingly new data from bhuman. */
  int frameDrops = allowedFrameDrops + 1; /**< The number frames without a reaction from bhuman. */

  enum State {sitting, standingUp, standing, sittingDown, preShuttingDown, preShuttingDownWhileSitting, shuttingDown} state;
  float phase = 0.f; /**< How far is the Nao in its current standing up or sitting down motion [0 ... 1]? */
  int ledIndex = 0; /**< The index of the last LED set. */

  int rightEarLEDsChangedTime = 0; // Last time when the right ear LEDs were changed by the B-Human code
  float requestedRightEarLEDs[earsLedRight324DegActuator - earsLedRight0DegActuator + 1]; // The state prevously requested by the B-Human code

  float startAngles[lbhNumOfPositionActuatorIds]; /**< Start angles for standing up or sitting down. */
  float startStiffness[lbhNumOfPositionActuatorIds]; /**< Start stiffness for sitting down. */

  int startPressedTime = 0; /**< The last time the chest button was not pressed. */
  unsigned lastBHumanStartTime = 0; /**< The last time bhuman was started. */

  /** Close all resources acquired. Called when initialization failed or during destruction. */
  void close()
  {
    fprintf(stderr, "libbhuman: Stopping.\n");

    if(proxy)
    {
      proxy->getGenericProxy()->getModule()->removeAllPreProcess();
      proxy->getGenericProxy()->getModule()->removeAllPostProcess();
      delete proxy;
    }
    if(memory)
      delete memory;
    if(sem != SEM_FAILED)
      sem_close(sem);
    if(data != MAP_FAILED)
      munmap(data, sizeof(LBHData));

    fprintf(stderr, "libbhuman: Stopped.\n");
  }

  /**
   * Set the eye LEDs based on the current state.
   * Shutting down -> Lower segments are red.
   * bhuman crashed -> Whole eyes quickly flash red.
   * bhuman not running -> Lower segments flash blue.
   * @param actuators The actuator values a part of which will be set by this method.
   */
  void setEyeLeds(float* actuators)
  {
    for(int i = faceLedRedLeft0DegActuator; i <= faceLedBlueRight315DegActuator; ++i)
      actuators[i] = 0.f;

    if(state == shuttingDown)
    {
      actuators[faceLedRedLeft180DegActuator] = 1.f;
      actuators[faceLedRedRight180DegActuator] = 1.f;
    }
    else if(data->state != okState)
    {
      // set the "libbhuman is active and bhuman crashed" leds
      float blink = float(dcmTime / 200 & 1);
      for(int i = faceLedRedLeft0DegActuator; i <= faceLedRedLeft315DegActuator; ++i)
        actuators[i] = blink;
      for(int i = faceLedRedRight0DegActuator; i <= faceLedRedRight315DegActuator; ++i)
        actuators[i] = blink;
    }
    else
    {
      // set the "libbhuman is active and bhuman is not running" LEDs
      float blink = float(dcmTime / 500 & 1);
      actuators[faceLedBlueLeft180DegActuator] = blink;
      actuators[faceLedBlueRight180DegActuator] = blink;
    }
  }

  /**
   * Shows the battery state in the right ear if the robot is in the standing state
   * and bhuman has not used one of these LEDs in the past 5 seconds.
   * @param actuators The actuator values a part of which will be set by this method.
   */
  void setBatteryLeds(float* actuators)
  {
    for(int i = earsLedRight0DegActuator; i <= earsLedRight324DegActuator; ++i)
      if(actuators[i] != requestedRightEarLEDs[i - earsLedRight0DegActuator])
      {
        rightEarLEDsChangedTime = dcmTime;
        requestedRightEarLEDs[i - earsLedRight0DegActuator] = actuators[i];
      }

    if(state != standing || dcmTime - rightEarLEDsChangedTime > 5000)
      for(int i = 0; i < 10; ++i)
        actuators[earsLedRight0DegActuator + i] = i < int(*sensorPtrs[batteryChargeSensor] * 10.f) ? 1.f : 0.f;
  }

  /**
   * Copies everything that's not for servos from one set of actuator values to another.
   * @param srcActuators The actuator values from which is copied.
   * @param destActuators The actuator values to which is copied.
   */
  void copyNonServos(const float* srcActuators, float* destActuators)
  {
    for(int i = faceLedRedLeft0DegActuator; i < lbhNumOfActuatorIds; ++i)
      destActuators[i] = srcActuators[i];
  }

  /**
   * Handles the different states libbhuman can be in.
   * @param actuators The actuator values requested. They will not be changed, but might
   *                  be used as result of this method.
   * @return The actuator values that should be set. In the standing state, they are
   *         identical to the actuators passed to this method. In all other states,
   *         they are different.
   */
  float* handleState(float* actuators)
  {
    static float controlledActuators[lbhNumOfActuatorIds];

    switch(state)
    {
      sitting:
        state = sitting;

      case sitting:
        memset(controlledActuators, 0, sizeof(controlledActuators));
        if(frameDrops > allowedFrameDrops ||
           (actuators[lHipPitchStiffnessActuator] == 0.f && actuators[rHipPitchStiffnessActuator] == 0.f))
          return controlledActuators;

        for(int i = 0; i < lbhNumOfPositionActuatorIds; ++i)
          startAngles[i] = *sensorPtrs[i * lbhNumOfDifSensors];

      standingUp:
        state = standingUp;
        phase = 0.f;

      case standingUp:
        if(phase < 1.f && frameDrops <= allowedFrameDrops)
        {
          memset(controlledActuators, 0, sizeof(controlledActuators));
          phase = std::min(phase + 0.005f, 1.f);
          for(int i = 0; i < lbhNumOfPositionActuatorIds; ++i)
          {
            controlledActuators[i] = actuators[i] * phase + startAngles[i] * (1.f - phase);
            float h = std::min(actuators[i + headYawStiffnessActuator], 0.5f);
            controlledActuators[i + headYawStiffnessActuator] = actuators[i + headYawStiffnessActuator] * phase + h * (1.f - phase);
          }
          return controlledActuators;
        }
        state = standing;

      case standing:
        if(frameDrops <= allowedFrameDrops)
          return actuators; // use original actuators

      case preShuttingDown:
        for(int i = 0; i < lbhNumOfPositionActuatorIds; ++i)
        {
          startAngles[i] = positionRequest[5][i][0];
          if(actuators[lHipPitchStiffnessActuator] == 0.f && actuators[rHipPitchStiffnessActuator] == 0.f)
            startStiffness[i] = 0.f;
          else if(i >= lShoulderPitchPositionActuator && i <= rElbowRollPositionActuator)
            startStiffness[i] = 0.4f;
          else
            startStiffness[i] = std::min<float>(stiffnessRequest[5][i][0], 0.3f);
        }
        state = state == preShuttingDown ? shuttingDown : sittingDown;
        phase = 0.f;

      case sittingDown:
      case shuttingDown:
      shuttingDown:
        if((phase < 1.f && frameDrops > allowedFrameDrops) || state == shuttingDown)
        {
          memset(controlledActuators, 0, sizeof(controlledActuators));
          phase = std::min(phase + 0.005f, 1.f);
          for(int i = 0; i < lbhNumOfPositionActuatorIds; ++i)
          {
            controlledActuators[i] = sitDownAngles[i] * phase + startAngles[i] * (1.f - phase);
            controlledActuators[i + headYawStiffnessActuator] = startStiffness[i];
          }
          return controlledActuators;
        }
        else if(frameDrops <= allowedFrameDrops)
        {
          for(int i = 0; i < lbhNumOfPositionActuatorIds; ++i)
            startAngles[i] = positionRequest[5][i][0];
          goto standingUp;
        }
        else
          goto sitting;

      case preShuttingDownWhileSitting:
        for(int i = 0; i < lbhNumOfPositionActuatorIds; ++i)
          startStiffness[i] = 0.f;
        state = shuttingDown;
        phase = 0.995f;
        goto shuttingDown;
    }
  }

  /** The method sets all actuators. */
  void setActuators()
  {
    // set all actuator values according to the values in the shared memory block
    try
    {
      dcmTime = proxy->getTime(0);

      data->readingActuators = data->newestActuators;
      if(data->readingActuators == lastReadingActuators)
      {
        if(actuatorDrops == 0)
          fprintf(stderr, "libbhuman: missed actuator request.\n");
        ++actuatorDrops;
      }
      else
        actuatorDrops = 0;
      lastReadingActuators = data->readingActuators;
      float* readingActuators = data->actuators[data->readingActuators];
      float* actuators = handleState(readingActuators);

      if(state != standing)
      {
        if(frameDrops > 0 || state == shuttingDown)
          setEyeLeds(actuators);
        else
          copyNonServos(readingActuators, actuators);
      }
      setBatteryLeds(actuators);

      // set position actuators
      positionRequest[4][0] = dcmTime; // 0 delay!
      for(int i = 0; i < lbhNumOfPositionActuatorIds; ++i)
        positionRequest[5][i][0] = actuators[i];
      proxy->setAlias(positionRequest);

      // set stiffness actuators
      bool requestedStiffness = false;
      for(int i = headYawStiffnessActuator; i < headYawStiffnessActuator + lbhNumOfStiffnessActuatorIds; ++i)
        if(actuators[i] != requestedActuators[i])
        {
          stiffnessRequest[4][0] = dcmTime; // 0 delay!
          for(int j = 0; j < lbhNumOfStiffnessActuatorIds; ++j)
            stiffnessRequest[5][j][0] = requestedActuators[headYawStiffnessActuator + j] = actuators[headYawStiffnessActuator + j];
          proxy->setAlias(stiffnessRequest);
          requestedStiffness = true;
          break;
        }

      // set led
      if(!requestedStiffness)
      {
        for(int i = 0; i < lbhNumOfLedActuatorIds; ++i)
        {
          int index = faceLedRedLeft0DegActuator + ledIndex;
          if(++ledIndex == lbhNumOfLedActuatorIds)
            ledIndex = 0;
          if(actuators[index] != requestedActuators[index])
          {
            ledRequest[0] = std::string(actuatorNames[index]);
            ledRequest[2][0][0] = requestedActuators[index] = actuators[index];
            ledRequest[2][0][1] = dcmTime;
            proxy->set(ledRequest);
            if(data->state == okState) // Allow setting all LEDs only when bhuman is not running
              break;
          }
        }
      }

      // set team info
      // since this should happen very rarely, we don't use a proxy here
      if(data->bhumanStartTime != lastBHumanStartTime)
      {
        for(int i = 0; i < lbhNumOfTeamInfoIds; ++i)
          memory->insertData(teamInfoNames[i], data->teamInfo[i]);
        lastBHumanStartTime = data->bhumanStartTime;
      }
    }
    catch(AL::ALError& e)
    {
      fprintf(stderr, "libbhuman: %s\n", e.what());
    }
  }

  /**
   * The method reads all sensors. It also detects if the chest button was pressed
   * for at least three seconds. In that case, it shuts down the robot.
   */
  void readSensors()
  {
    // get new sensor values and copy them to the shared memory block
    try
    {
      // copy sensor values into the shared memory block
      int writingSensors = 0;
      if(writingSensors == data->newestSensors)
        ++writingSensors;
      if(writingSensors == data->readingSensors)
        if(++writingSensors == data->newestSensors)
          ++writingSensors;
      assert(writingSensors != data->newestSensors);
      assert(writingSensors != data->readingSensors);

      float* sensors = data->sensors[writingSensors];
      for(int i = 0; i < lbhNumOfSensorIds; ++i)
        sensors[i] = *sensorPtrs[i];

      AL::ALValue value = memory->getData("GameCtrl/RoboCupGameControlData");
      if(value.isBinary() && value.getSize() == sizeof(RoboCup::RoboCupGameControlData))
        memcpy(&data->gameControlData[writingSensors], value, sizeof(RoboCup::RoboCupGameControlData));

      data->newestSensors = writingSensors;

      // detect shutdown request via chest-button
      if(*sensorPtrs[chestButtonSensor] == 0.f)
        startPressedTime = dcmTime;
      else if(state != shuttingDown && startPressedTime && dcmTime - startPressedTime > 3000)
      {
        (void) !system("( /home/nao/bin/bhumand stop && sudo shutdown -h now ) &");
        state = state == sitting ? preShuttingDownWhileSitting : preShuttingDown;
      }
    }
    catch(AL::ALError& e)
    {
      fprintf(stderr, "libbhuman: %s\n", e.what());
    }

    // raise the semaphore
    if(sem != SEM_FAILED)
    {
      int sval;
      if(sem_getvalue(sem, &sval) == 0)
      {
        if(sval < 1)
        {
          sem_post(sem);
          frameDrops = 0;
        }
        else
        {
          if(frameDrops == 0)
            fprintf(stderr, "libbhuman: dropped sensor data.\n");
          ++frameDrops;
        }
      }
    }
  }

  /**
   * The method is called by NaoQi immediately before it communicates with the chest board.
   * It sets all the actuators.
   */
  static void onPreProcess()
  {
    theInstance->setActuators();
  }

  /**
   * The method is called by NaoQi immediately after it communicated with the chest board.
   * It reads all sensors.
   */
  static void onPostProcess()
  {
    theInstance->readSensors();
  }

public:
  /**
   * The constructor initializes the shared memory for communicating with bhuman.
   * It also establishes a communication with NaoQi and prepares all data structures
   * required for this communication.
   * @param pBroker A NaoQi broker that allows accessing other NaoQi modules.
   */
  BHuman(boost::shared_ptr<AL::ALBroker> pBroker) :
    ALModule(pBroker, "BHuman"),
    data((LBHData*) MAP_FAILED),
    sem(SEM_FAILED),
    state(sitting)
  {
    setModuleDescription("A module that provides basic ipc NaoQi DCM access using shared memory.");
    fprintf(stderr, "libbhuman: Starting.\n");

    static_assert(lbhNumOfSensorIds == sizeof(sensorNames) / sizeof(*sensorNames), "sensor enum size does not match sensor name array size");
    static_assert(lbhNumOfActuatorIds == sizeof(actuatorNames) / sizeof(*actuatorNames), "actuator enum size does not match actuator name array size");
    static_assert(lbhNumOfTeamInfoIds == sizeof(teamInfoNames) / sizeof(*teamInfoNames), "team info enum size does not match team info name array size");

    // create shared memory
    memoryHandle = shm_open(LBH_MEM_NAME, O_CREAT | O_RDWR, S_IRUSR | S_IWUSR);
    if(memoryHandle == -1)
      perror("libbhuman: shm_open");
    else if(ftruncate(memoryHandle, sizeof(LBHData)) == -1)
      perror("libbhuman: ftruncate");
    else
    {
      // map the shared memory
      data = (LBHData*) mmap(nullptr, sizeof(LBHData), PROT_READ | PROT_WRITE, MAP_SHARED, memoryHandle, 0);
      if(data == MAP_FAILED)
        perror("libbhuman: mmap");
      else
      {
        memset(data, 0, sizeof(LBHData));

        // open semaphore
        sem = sem_open(LBH_SEM_NAME, O_CREAT | O_RDWR, S_IRUSR | S_IWUSR, 0);
        if(sem == SEM_FAILED)
          perror("libbhuman: sem_open");
        else
          try
          {
            // get the robot name
            memory = new AL::ALMemoryProxy(pBroker);

            std::string headVersion = (std::string) memory->getData("RobotConfig/Head/BaseVersion", 0);
            std::string bodyVersion = (std::string) memory->getData("RobotConfig/Body/BaseVersion", 0);

            if(!parseNAOVersion(headVersion, data->headVersion))
            {
              fprintf(stderr, "libbhuman: unknown headVerion: %s!\n", headVersion.c_str());
              goto error;
            }
            if(!parseNAOVersion(bodyVersion, data->bodyVersion))
            {
              fprintf(stderr, "libbhuman: unknown bodyVersion: %s!\n", bodyVersion.c_str());
              goto error;
            }

            bool headIsH25 = (bool) memory->getData("Device/DeviceList/TouchBoard/Available", 0);
            bool bodyIsH25 = (bool) memory->getData("Device/DeviceList/LeftHandBoard/Available", 0);
            data->headType = headIsH25 ? NAOType::H25 : NAOType::H21;
            data->bodyType = bodyIsH25 ? NAOType::H25 : NAOType::H21;

            fprintf(stderr, "libbhuman: headIsH25: %d\n", headIsH25);
            fprintf(stderr, "libbhuman: bodyIsH25: %d\n", bodyIsH25);

            int headIdSize;
            data->headVersion == NAOVersion::V6 ? headIdSize = 20 : headIdSize = 15;
            std::string headId = (std::string) memory->getData("RobotConfig/Head/FullHeadId", 0);
            std::strncpy(data->headId, headId.c_str(), headIdSize);
            data->headId[headIdSize] = 0;

            int bodyIdSize;
            data->bodyVersion == NAOVersion::V6 ? bodyIdSize = 20 : bodyIdSize = 15;
            std::string bodyId = (std::string) memory->getData("Device/DeviceList/ChestBoard/BodyId", 0);
            std::strncpy(data->bodyId, bodyId.c_str(), bodyIdSize);
            data->bodyId[bodyIdSize] = 0;

            // create "positionRequest" and "stiffnessRequest" alias
            proxy = new AL::DCMProxy(pBroker);

            AL::ALValue params;
            AL::ALValue result;
            params.arraySetSize(1);
            params.arraySetSize(2);

            params[0] = std::string("positionActuators");
            params[1].arraySetSize(lbhNumOfPositionActuatorIds);
            for(int i = 0; i < lbhNumOfPositionActuatorIds; ++i)
              params[1][i] = std::string(actuatorNames[i]);
            result = proxy->createAlias(params);

            params[0] = std::string("stiffnessActuators");
            params[1].arraySetSize(lbhNumOfStiffnessActuatorIds);
            for(int i = 0; i < lbhNumOfStiffnessActuatorIds; ++i)
              params[1][i] = std::string(actuatorNames[headYawStiffnessActuator + i]);
            result = proxy->createAlias(params);

            // prepare positionRequest
            positionRequest.arraySetSize(6);
            positionRequest[0] = std::string("positionActuators");
            positionRequest[1] = std::string("ClearAll");
            positionRequest[2] = std::string("time-separate");
            positionRequest[3] = 0;
            positionRequest[4].arraySetSize(1);
            positionRequest[5].arraySetSize(lbhNumOfPositionActuatorIds);
            for(int i = 0; i < lbhNumOfPositionActuatorIds; ++i)
              positionRequest[5][i].arraySetSize(1);

            // prepare stiffnessRequest
            stiffnessRequest.arraySetSize(6);
            stiffnessRequest[0] = std::string("stiffnessActuators");
            stiffnessRequest[1] = std::string("ClearAll");
            stiffnessRequest[2] = std::string("time-separate");
            stiffnessRequest[3] = 0;
            stiffnessRequest[4].arraySetSize(1);
            stiffnessRequest[5].arraySetSize(lbhNumOfStiffnessActuatorIds);
            for(int i = 0; i < lbhNumOfStiffnessActuatorIds; ++i)
              stiffnessRequest[5][i].arraySetSize(1);

            // prepare ledRequest
            ledRequest.arraySetSize(3);
            ledRequest[1] = std::string("ClearAll");
            ledRequest[2].arraySetSize(1);
            ledRequest[2][0].arraySetSize(2);
            ledRequest[2][0][1] = 0;

            // prepare sensor pointers
            for(int i = 0; i < lbhNumOfSensorIds; ++i)
              sensorPtrs[i] = (float*) memory->getDataPtr(sensorNames[i]);

            // initialize requested actuators
            memset(requestedActuators, 0, sizeof(requestedActuators));
            for(int i = faceLedRedLeft0DegActuator; i < chestBoardLedRedActuator; ++i)
              requestedActuators[i] = -1.f;
            for(int i = earsLedRight0DegActuator; i <= earsLedRight324DegActuator; ++i)
              requestedRightEarLEDs[i - earsLedRight0DegActuator] = -1.f;

            // register "onPreProcess" and "onPostProcess" callbacks
            theInstance = this;
            proxy->getGenericProxy()->getModule()->atPreProcess(&onPreProcess);
            proxy->getGenericProxy()->getModule()->atPostProcess(&onPostProcess);

            fprintf(stderr, "libbhuman: Started!\n");
            return; // success
          }
          catch(AL::ALError& e)
          {
            fprintf(stderr, "libbhuman: %s\n", e.what());
          }
      }
    }
  error:
    close(); // error
  }

  /** Close all resources acquired. */
  ~BHuman()
  {
    close();
  }

  bool parseNAOVersion(const std::string& versionString, NAOVersion& version)
  {
    if(versionString.size() == 4)
    {
      if(versionString[1] == '6')
        version = NAOVersion::V6;
      else if(versionString[1] == '5')
        version = NAOVersion::V5;
      else if(versionString[1] == '4')
        version = NAOVersion::V4;
      else if(versionString[1] == '3' && versionString[3] == '3')
        version = NAOVersion::V33;
      else if(versionString[1] == '3' && versionString[3] == '2')
        version = NAOVersion::V32;
      else
        return false;
      return true;
    }
    else
      return false;
  }
};

BHuman* BHuman::theInstance = nullptr;

/**
 * This method is called by NaoQi when loading this library.
 * Creates an instance of class BHuman.
 * @param pBroker A NaoQi broker that allows accessing other NaoQi modules.
 */
extern "C" int _createModule(boost::shared_ptr<AL::ALBroker> pBroker)
{
  AL::ALModule::createModule<BHuman>(pBroker);
  return 0;
}
