/**
 * @file NaoProvider.cpp
 *
 * This file implements a module that communicates with LoLA.
 *
 * @author Thomas Röfer
 */

#include "NaoProvider.h"
#include "Platform/BHAssert.h"
#include "Platform/File.h"
#include "Platform/Thread.h"
#include "Platform/Time.h"
#include "Tools/Communication/MsgPack.h"
#include "Tools/Global.h"
#include "Tools/Settings.h"
#include "Tools/Streams/OutStreams.h"
#include <cstring>
#include <csignal>

MAKE_MODULE(NaoProvider, infrastructure)

#ifdef TARGET_ROBOT

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

thread_local NaoProvider* NaoProvider::theInstance = nullptr;

const Joints::Joint NaoProvider::jointMappings[Joints::numOfJoints - 1] =
{
  Joints::headYaw,
  Joints::headPitch,

  Joints::lShoulderPitch,
  Joints::lShoulderRoll,
  Joints::lElbowYaw,
  Joints::lElbowRoll,
  Joints::lWristYaw,

  Joints::lHipYawPitch,
  Joints::lHipRoll,
  Joints::lHipPitch,
  Joints::lKneePitch,
  Joints::lAnklePitch,
  Joints::lAnkleRoll,

  Joints::rHipRoll,
  Joints::rHipPitch,
  Joints::rKneePitch,
  Joints::rAnklePitch,
  Joints::rAnkleRoll,

  Joints::rShoulderPitch,
  Joints::rShoulderRoll,
  Joints::rElbowYaw,
  Joints::rElbowRoll,
  Joints::rWristYaw,

  Joints::lHand,
  Joints::rHand
};

const KeyStates::Key NaoProvider::keyMappings[KeyStates::numOfKeys] =
{
  KeyStates::chest,

  KeyStates::headFront,
  KeyStates::headMiddle,
  KeyStates::headRear,

  KeyStates::lFootLeft,
  KeyStates::lFootRight,
  KeyStates::lHandBack,
  KeyStates::lHandLeft,
  KeyStates::lHandRight,

  KeyStates::rFootLeft,
  KeyStates::rFootRight,
  KeyStates::rHandBack,
  KeyStates::rHandLeft,
  KeyStates::rHandRight
};

const LEDRequest::LED NaoProvider::leftEyeMappings[] =
{
  LEDRequest::faceLeftRed45Deg,
  LEDRequest::faceLeftRed0Deg,
  LEDRequest::faceLeftRed315Deg,
  LEDRequest::faceLeftRed270Deg,
  LEDRequest::faceLeftRed225Deg,
  LEDRequest::faceLeftRed180Deg,
  LEDRequest::faceLeftRed135Deg,
  LEDRequest::faceLeftRed90Deg,

  LEDRequest::faceLeftGreen45Deg,
  LEDRequest::faceLeftGreen0Deg,
  LEDRequest::faceLeftGreen315Deg,
  LEDRequest::faceLeftGreen270Deg,
  LEDRequest::faceLeftGreen225Deg,
  LEDRequest::faceLeftGreen180Deg,
  LEDRequest::faceLeftGreen135Deg,
  LEDRequest::faceLeftGreen90Deg,

  LEDRequest::faceLeftBlue45Deg,
  LEDRequest::faceLeftBlue0Deg,
  LEDRequest::faceLeftBlue315Deg,
  LEDRequest::faceLeftBlue270Deg,
  LEDRequest::faceLeftBlue225Deg,
  LEDRequest::faceLeftBlue180Deg,
  LEDRequest::faceLeftBlue135Deg,
  LEDRequest::faceLeftBlue90Deg
};

const LEDRequest::LED NaoProvider::rightEyeMappings[] =
{
  LEDRequest::faceRightRed0Deg,
  LEDRequest::faceRightRed45Deg,
  LEDRequest::faceRightRed90Deg,
  LEDRequest::faceRightRed135Deg,
  LEDRequest::faceRightRed180Deg,
  LEDRequest::faceRightRed225Deg,
  LEDRequest::faceRightRed270Deg,
  LEDRequest::faceRightRed315Deg,

  LEDRequest::faceRightGreen0Deg,
  LEDRequest::faceRightGreen45Deg,
  LEDRequest::faceRightGreen90Deg,
  LEDRequest::faceRightGreen135Deg,
  LEDRequest::faceRightGreen180Deg,
  LEDRequest::faceRightGreen225Deg,
  LEDRequest::faceRightGreen270Deg,
  LEDRequest::faceRightGreen315Deg,

  LEDRequest::faceRightBlue0Deg,
  LEDRequest::faceRightBlue45Deg,
  LEDRequest::faceRightBlue90Deg,
  LEDRequest::faceRightBlue135Deg,
  LEDRequest::faceRightBlue180Deg,
  LEDRequest::faceRightBlue225Deg,
  LEDRequest::faceRightBlue270Deg,
  LEDRequest::faceRightBlue315Deg
};

const LEDRequest::LED NaoProvider::leftEarMappings[] =
{
  LEDRequest::earsLeft0Deg,
  LEDRequest::earsLeft36Deg,
  LEDRequest::earsLeft72Deg,
  LEDRequest::earsLeft108Deg,
  LEDRequest::earsLeft144Deg,
  LEDRequest::earsLeft180Deg,
  LEDRequest::earsLeft216Deg,
  LEDRequest::earsLeft252Deg,
  LEDRequest::earsLeft288Deg,
  LEDRequest::earsLeft324Deg
};

const LEDRequest::LED NaoProvider::rightEarMappings[] =
{
  LEDRequest::earsRight324Deg,
  LEDRequest::earsRight288Deg,
  LEDRequest::earsRight252Deg,
  LEDRequest::earsRight216Deg,
  LEDRequest::earsRight180Deg,
  LEDRequest::earsRight144Deg,
  LEDRequest::earsRight108Deg,
  LEDRequest::earsRight72Deg,
  LEDRequest::earsRight36Deg,
  LEDRequest::earsRight0Deg
};

const LEDRequest::LED NaoProvider::chestMappings[] =
{
  LEDRequest::chestRed,
  LEDRequest::chestGreen,
  LEDRequest::chestBlue,
};

const LEDRequest::LED NaoProvider::skullMappings[] =
{
  LEDRequest::headFrontLeft1,
  LEDRequest::headFrontLeft0,
  LEDRequest::headMiddleLeft0,
  LEDRequest::headRearLeft0,
  LEDRequest::headRearLeft1,
  LEDRequest::headRearLeft2,
  LEDRequest::headRearRight2,
  LEDRequest::headRearRight1,
  LEDRequest::headRearRight0,
  LEDRequest::headMiddleRight0,
  LEDRequest::headFrontRight0,
  LEDRequest::headFrontRight1,
};

const LEDRequest::LED NaoProvider::leftFootMappings[] =
{
  LEDRequest::footLeftRed,
  LEDRequest::footLeftGreen,
  LEDRequest::footLeftBlue,
};

const LEDRequest::LED NaoProvider::rightFootMappings[] =
{
  LEDRequest::footRightRed,
  LEDRequest::footRightGreen,
  LEDRequest::footRightBlue,
};

NaoProvider::NaoProvider()
{
  ASSERT(!theInstance);
  theInstance = this;

  // Set all pointers into receivedPacket to nullptr
  std::memset(&fsrs[0][0], 0, sizeof(fsrs));
  std::memset(&gyros[0], 0, sizeof(gyros));
  std::memset(&accs[0], 0, sizeof(accs));
  std::memset(&torsoAngles[0], 0, sizeof(torsoAngles));
  std::memset(&jointAngles[0], 0, sizeof(jointAngles));
  std::memset(&jointCurrents[0], 0, sizeof(jointCurrents));
  std::memset(&jointTemperatures[0], 0, sizeof(jointTemperatures));
  std::memset(&jointStatuses[0], 0, sizeof(jointStatuses));
  std::memset(&keys[0], 0, sizeof(keys));
  std::memset(&jointRequests[0], 0, sizeof(jointRequests));
  std::memset(&jointStiffnesses[0], 0, sizeof(jointStiffnesses));
  std::memset(&leds[0], 0, sizeof(leds));

  socket = ::socket(AF_UNIX, SOCK_STREAM, 0);
  ASSERT(socket > 0);
  sockaddr_un address;
  address.sun_family = AF_UNIX;
  std::strcpy(address.sun_path, "/tmp/robocup");
  VERIFY(!connect(socket, reinterpret_cast<sockaddr*>(&address), sizeof(address)));

  // Receive a first packet and setup all tables
  receivePacket();
}

NaoProvider::~NaoProvider()
{
  close(socket);
  theInstance = nullptr;
}

void NaoProvider::update(FrameInfo& theFrameInfo)
{
  theFrameInfo.time = timeWhenPacketReceived;
}

void NaoProvider::update(FsrSensorData& theFsrSensorData)
{
  FOREACH_ENUM(Legs::Leg, leg)
  {
    theFsrSensorData.totals[leg] = 0.f;
    FOREACH_ENUM(FsrSensors::FsrSensor, fsr)
    {
      theFsrSensorData.pressures[leg][fsr] = MsgPack::readFloat(fsrs[leg][fsr]);
      theFsrSensorData.totals[leg] += theFsrSensorData.pressures[leg][fsr];
    }
  }
}

void NaoProvider::update(InertialSensorData& theInertialSensorData)
{
  for(size_t i = 0; i < 3; ++i)
  {
    theInertialSensorData.gyro(i) = MsgPack::readFloat(gyros[i]);
    theInertialSensorData.acc(i) = MsgPack::readFloat(accs[i]);
    theInertialSensorData.angle(i) = MsgPack::readFloat(torsoAngles[i]);
  }

  // Invert accelerometer signs
  theInertialSensorData.acc *= -1.f;
}

void NaoProvider::update(JointSensorData& theJointSensorData)
{
  FOREACH_ENUM(Joints::Joint, joint)
  {
    if(joint == Joints::rHipYawPitch)
    {
      theJointSensorData.angles[joint] = theJointSensorData.angles[Joints::lHipYawPitch] + theJointCalibration.offsets[Joints::lHipYawPitch]
                                         - theJointCalibration.offsets[joint];
      theJointSensorData.currents[joint] = theJointSensorData.currents[Joints::lHipYawPitch];
      theJointSensorData.temperatures[joint] = theJointSensorData.temperatures[Joints::lHipYawPitch];
      theJointSensorData.status[joint] = theJointSensorData.status[Joints::lHipYawPitch];
    }
    else
    {
      theJointSensorData.angles[joint] = MsgPack::readFloat(jointAngles[joint]) - theJointCalibration.offsets[joint];
      theJointSensorData.currents[joint] = static_cast<short>(1000.f * MsgPack::readFloat(jointCurrents[joint]));
      theJointSensorData.temperatures[joint] = static_cast<unsigned char>(MsgPack::readFloat(jointTemperatures[joint]));
      theJointSensorData.status[joint] = static_cast<JointSensorData::TemperatureStatus>(jointStatuses[joint] ? *jointStatuses[joint] : 0);
    }
  }
  theJointSensorData.timestamp = timeWhenPacketReceived;
}

void NaoProvider::update(KeyStates& theKeyStates)
{
  FOREACH_ENUM(KeyStates::Key, key)
    theKeyStates.pressed[key] = MsgPack::readFloat(keys[key]) != 0.f;

  if(!theKeyStates.pressed[KeyStates::chest])
    timeWhenChestButtonUnpressed = theFrameInfo.time;
  else if(timeWhenChestButtonUnpressed && theFrameInfo.getTimeSince(timeWhenChestButtonUnpressed) >= timeChestButtonPressedUntilShutdown)
  {
    raise(SIGINT);
    timeWhenChestButtonUnpressed = 0;
  }
}

void NaoProvider::update(SystemSensorData& theSystemSensorData)
{
  theSystemSensorData.batteryLevel = MsgPack::readFloat(batteryLevel);
  theSystemSensorData.batteryCurrent = MsgPack::readFloat(batteryCurrent);
  theSystemSensorData.batteryTemperature = MsgPack::readFloat(batteryTemperature);
  theSystemSensorData.batteryCharging = (static_cast<short>(MsgPack::readFloat(batteryCharging)) & 0x80) != 0;
  if(theFrameInfo.getTimeSince(timeWhenBatteryLevelWritten) >= timeBetweenBatteryLevelUpdates)
  {
    OutTextFile("/var/volatile/tmp/batteryLevel.txt") << theSystemSensorData.batteryLevel << MsgPack::readFloat(batteryCharging);
    timeWhenBatteryLevelWritten = theFrameInfo.time;
  }
}

void NaoProvider::receivePacket()
{
  // Read maximum size for first packet, but the smaller size for all further packets.
  const long bytesRead = recv(socket, reinterpret_cast<char*>(receivedPacket), sizeof(receivedPacket), 0);
  if(bytesRead < 0)
    OUTPUT_ERROR("Could not receive packet from NAO");
  else
  {
    timeWhenPacketReceived = std::max(Time::getCurrentSystemTime(), timeWhenPacketReceived + 1);

    // Initialize tables if they have not been so far
    if(!batteryLevel)
    {
      MsgPack::parse(receivedPacket, bytesRead,

        // Most data is encoded as float 32
        [this](const std::string& key, const unsigned char* p)
        {
          const std::string::size_type pos = key.find(":");
          ASSERT(pos != std::string::npos);
          const std::string category = key.substr(0, pos);
          const int index = std::stoi(key.substr(pos + 1));

          if(category == "Current")
            jointCurrents[jointMappings[index]] = p;
          else if(category == "Position")
            jointAngles[jointMappings[index]] = p;
          else if(category == "Temperature")
            jointTemperatures[jointMappings[index]] = p;
          else if(category == "FSR")
            fsrs[index / FsrSensors::numOfFsrSensors][index % FsrSensors::numOfFsrSensors] = p;
          else if(category == "Accelerometer")
            accs[index] = p;
          else if(category == "Gyroscope")
            gyros[index] = p;
          else if(category == "Angles")
            torsoAngles[index] = p;
          else if(category == "Touch")
            keys[keyMappings[index]] = p;
          else if(category == "Battery")
          {
            if(index == 0)
              batteryLevel = p;
            else if(index == 1)
              batteryCharging = p;
            else if(index == 2)
              batteryCurrent = p;
            else
              batteryTemperature = p;
          }
          else if(category != "Sonar" && category != "Stiffness")
            OUTPUT_WARNING("Unknown key " << key);
        },

        // Only joint temperature statuses are encoded as positive fixint
        [this](const std::string& key, const unsigned char* p)
        {
          const std::string::size_type pos = key.find(":");
          ASSERT(pos != std::string::npos);
          const std::string category = key.substr(0, pos);
          const int index = std::stoi(key.substr(pos + 1));

          if(category == "Status")
            jointStatuses[jointMappings[index]] = p;
          else
            OUTPUT_WARNING("Unknown key " << key);
        },

        // Ignore strings
        [](const std::string& key, const unsigned char* p, size_t size) {});

      // Initialize the packet to send to LoLA
      // Please note that the code assumes that the order of sensors and actuators is the same
      unsigned char* p = packetToSend;
      MsgPack::writeMapHeader(10, p);

      // Determine addresses for target positions
      MsgPack::write("Position", p);
      MsgPack::writeArrayHeader(Joints::numOfJoints - 1, p);
      for(int i = 0; i < Joints::numOfJoints - 1; ++i)
        jointRequests[jointMappings[i]] = MsgPack::write(0.f, p);

      // Determine addresses for stiffnesses
      MsgPack::write("Stiffness", p);
      MsgPack::writeArrayHeader(Joints::numOfJoints - 1, p);
      for(int i = 0; i < Joints::numOfJoints - 1; ++i)
        jointStiffnesses[jointMappings[i]] = MsgPack::write(0.f, p);

      // Determine addresses for leds
      writeLEDs("REar", rightEarMappings, LEDRequest::chestRed - LEDRequest::earsRight0Deg, p);
      writeLEDs("LEar", leftEarMappings, LEDRequest::earsRight0Deg - LEDRequest::earsLeft0Deg, p);
      writeLEDs("Chest", chestMappings, LEDRequest::headRearLeft0 - LEDRequest::chestRed, p);
      writeLEDs("LEye", leftEyeMappings, LEDRequest::faceRightRed0Deg - LEDRequest::faceLeftRed0Deg, p);
      writeLEDs("REye", rightEyeMappings, LEDRequest::earsLeft0Deg - LEDRequest::faceRightRed0Deg, p);
      writeLEDs("LFoot", leftFootMappings, LEDRequest::footRightRed - LEDRequest::footLeftRed, p);
      writeLEDs("RFoot", rightFootMappings, LEDRequest::numOfLEDs - LEDRequest::footRightRed, p);
      writeLEDs("Skull", skullMappings, LEDRequest::footLeftRed - LEDRequest::headRearLeft0, p);

      packetToSendSize = static_cast<int>(p - packetToSend);
      ASSERT(packetToSendSize <= static_cast<int>(sizeof(packetToSend)));
    }
  }
}

void NaoProvider::writeLEDs(const std::string category, const LEDRequest::LED* ledMappings, int numOfLEDs, unsigned char*& p)
{
  MsgPack::write(category, p);
  MsgPack::writeArrayHeader(numOfLEDs, p);
  for(int i = 0; i < numOfLEDs; ++i)
    leds[ledMappings[i]] = MsgPack::write(0.f, p);
}

void NaoProvider::sendPacket()
{
  FOREACH_ENUM(Joints::Joint, joint)
    if(joint != Joints::rHipYawPitch)
    {
      if(theJointRequest.angles[joint] == SensorData::off || !theJointRequest.stiffnessData.stiffnesses[joint])
      {
        MsgPack::writeFloat(MsgPack::readFloat(jointAngles[joint]), jointRequests[joint]);
        MsgPack::writeFloat(0.f, jointStiffnesses[joint]);
      }
      else
      {
        MsgPack::writeFloat(theJointRequest.angles[joint] + theJointCalibration.offsets[joint], jointRequests[joint]);
        MsgPack::writeFloat(static_cast<float>(theJointRequest.stiffnessData.stiffnesses[joint]) * 0.01f, jointStiffnesses[joint]);
      }
    }

  bool on = (timeWhenPacketReceived / 400 & 1) == 1;
  bool fastOn = (timeWhenPacketReceived / 80 & 1) == 0;
  FOREACH_ENUM(LEDRequest::LED, led)
  {
    LEDRequest::LEDState state = theLEDRequest.ledStates[led];
    MsgPack::writeFloat(state == LEDRequest::on ||
                        (state == LEDRequest::blinking && on) ||
                        (state == LEDRequest::fastBlinking && fastOn)
                        ? 1.0f
                        : state == LEDRequest::half ? 0.5f : 0.0f, leds[led]);
  }

  VERIFY(send(socket, reinterpret_cast<char*>(packetToSend), packetToSendSize, 0) == static_cast<ssize_t>(packetToSendSize));
}

void NaoProvider::waitForFrameData()
{
  DEBUG_RESPONSE_ONCE("module:NaoProvider:robotName")
  {
    if(Global::getSettings().headName == Global::getSettings().bodyName)
      OUTPUT_TEXT("Hi, I am " << Global::getSettings().headName << ".");
    else
      OUTPUT_TEXT("Hi, I am " << Global::getSettings().headName << " (using " << Global::getSettings().bodyName << "'s body).");

    OUTPUT(idRobotname, bin, Global::getSettings().headName << Global::getSettings().bodyName << Global::getSettings().location << Global::getSettings().scenario << Global::getSettings().playerNumber);
  }

  if(theInstance)
    theInstance->receivePacket();
}

void NaoProvider::finishFrame()
{
  DEBUG_RESPONSE("module:NaoProvider:lag100") Thread::sleep(100);
  DEBUG_RESPONSE("module:NaoProvider:lag200") Thread::sleep(200);
  DEBUG_RESPONSE("module:NaoProvider:lag300") Thread::sleep(300);
  DEBUG_RESPONSE("module:NaoProvider:lag1000") Thread::sleep(1000);
  DEBUG_RESPONSE("module:NaoProvider:lag3000") Thread::sleep(3000);
  DEBUG_RESPONSE("module:NaoProvider:lag6000") Thread::sleep(6000);
  DEBUG_RESPONSE("module:NaoProvider:segfault") *static_cast<volatile char*>(nullptr) = 0;

  if(theInstance)
    theInstance->sendPacket();
}

#endif
