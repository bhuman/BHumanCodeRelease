/**
 * @file NaoProvider.h
 *
 * This file declares a module that communicates with LoLA.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/LEDRequest.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Infrastructure/SensorData/SystemSensorData.h"
#include "Tools/Module/Module.h"

MODULE(NaoProvider,
{,
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo), // This is to ensure that the GameInfo exists in the Motion blackboard and the Logger can access it.
  REQUIRES(JointCalibration),
  USES(JointRequest),
  REQUIRES(LEDRequest),
  REQUIRES(OpponentTeamInfo), // This is to ensure that the OwnTeamInfo exists in the Motion blackboard and the Logger can access it.
  PROVIDES(FrameInfo),
  PROVIDES(FsrSensorData),
  PROVIDES(InertialSensorData),
  PROVIDES(JointSensorData),
  PROVIDES(KeyStates),
  PROVIDES(SystemSensorData),
  DEFINES_PARAMETERS(
  {,
    (int)(3000) timeChestButtonPressedUntilShutdown, /**< Time the chest button must be pressed until shutdown (in ms). */
    (int)(5000) timeBetweenBatteryLevelUpdates, /**< Time between writing updates the battery level to a file (in ms). */
  }),
});

#ifdef TARGET_ROBOT

class NaoProvider : public NaoProviderBase
{
  static thread_local NaoProvider* theInstance; /**< The only instance of this module. */
  static const Joints::Joint jointMappings[Joints::numOfJoints - 1]; /**< Mappings from LoLA's joint indices to B-Human's joint indices. */
  static const KeyStates::Key keyMappings[KeyStates::numOfKeys]; /**< Mappings from LoLA's touch indices to B-Human's key indices. */
  static const LEDRequest::LED leftEyeMappings[LEDRequest::faceRightRed0Deg - LEDRequest::faceLeftRed0Deg]; /**< Mappings from LoLA's LED indices to B-Human's LED indices. */
  static const LEDRequest::LED rightEyeMappings[LEDRequest::earsLeft0Deg - LEDRequest::faceRightRed0Deg]; /**< Mappings from LoLA's LED indices to B-Human's LED indices. */
  static const LEDRequest::LED leftEarMappings[LEDRequest::earsRight0Deg - LEDRequest::earsLeft0Deg]; /**< Mappings from LoLA's LED indices to B-Human's LED indices. */
  static const LEDRequest::LED rightEarMappings[LEDRequest::chestRed - LEDRequest::earsRight0Deg]; /**< Mappings from LoLA's LED indices to B-Human's LED indices. */
  static const LEDRequest::LED chestMappings[LEDRequest::headRearLeft0 - LEDRequest::chestRed]; /**< Mappings from LoLA's LED indices to B-Human's LED indices. */
  static const LEDRequest::LED skullMappings[LEDRequest::footLeftRed - LEDRequest::headRearLeft0]; /**< Mappings from LoLA's LED indices to B-Human's LED indices. */
  static const LEDRequest::LED leftFootMappings[LEDRequest::footRightRed - LEDRequest::footLeftRed]; /**< Mappings from LoLA's LED indices to B-Human's LED indices. */
  static const LEDRequest::LED rightFootMappings[LEDRequest::numOfLEDs - LEDRequest::footRightRed]; /**< Mappings from LoLA's LED indices to B-Human's LED indices. */

  int socket; /**< Socket to connect to LoLA. */
  unsigned char receivedPacket[896]; /**< The last packet received from LoLA. */
  unsigned char packetToSend[1000]; /**< The packet to send to LoLA. */
  size_t packetToSendSize; /**< The size of the packet to send. */

  std::array<std::array<const unsigned char*, FsrSensors::numOfFsrSensors>, Legs::numOfLegs> fsrs; /**< The addresses of fsr data inside the receivedPacket. */
  std::array<const unsigned char*, 3> gyros; /**< The addresses of gyro data inside receivedPacket. */
  std::array<const unsigned char*, 3> accs; /**< The addresses of accelerometer data inside receivedPacket. */
  std::array<const unsigned char*, 3> torsoAngles; /**< The addresses of torso angle data inside receivedPacket. */
  std::array<const unsigned char*, Joints::numOfJoints> jointAngles; /**< The addresses of joint angle data inside receivedPacket. */
  std::array<const unsigned char*, Joints::numOfJoints> jointCurrents; /**< The addresses of joint current data inside receivedPacket. */
  std::array<const unsigned char*, Joints::numOfJoints> jointTemperatures; /**< The addresses of joint temperature data inside receivedPacket. */
  std::array<const unsigned char*, Joints::numOfJoints> jointStatuses; /**< The addresses of joint status data inside receivedPacket. */
  std::array<const unsigned char*, KeyStates::numOfKeys> keys; /**< The addresses of key state data inside receivedPacket. */
  const unsigned char* batteryLevel = nullptr; /**< The address of battery level data inside receivedPacket. */
  const unsigned char* batteryCurrent = nullptr; /**< The address of battery current data inside receivedPacket. */
  const unsigned char* batteryTemperature = nullptr; /**< The address of battery temperature data inside receivedPacket. */
  const unsigned char* batteryCharging = nullptr; /**< The address of battery charging state data inside receivedPacket. */
  std::array<unsigned char*, Joints::numOfJoints> jointRequests; /**< The addresses of joint request data inside packetToSend. */
  std::array<unsigned char*, Joints::numOfJoints> jointStiffnesses; /**< The addresses of joint stiffness data inside packetToSend. */
  std::array<unsigned char*, LEDRequest::numOfLEDs> leds; /**< The addresses of led data inside packetToSend. */
  unsigned timeWhenPacketReceived = 0; /**< The time when the last packet was received. */
  unsigned timeWhenChestButtonUnpressed = 0; /**< The last time the chest buttom was not pressed. */
  unsigned timeWhenBatteryLevelWritten = 0; /**< The last time the battery level was written to a file. */

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theFrameInfo The representation updated.
   */
  void update(FrameInfo& theFrameInfo) override;

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theFsrSensorData The representation updated.
   */
  void update(FsrSensorData& theFsrSensorData) override;

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theInertialSensorData The representation updated.
   */
  void update(InertialSensorData& theInertialSensorData) override;

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theJointSensorData The representation updated.
   */
  void update(JointSensorData& theJointSensorData) override;

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theKeyStates The representation updated.
   */
  void update(KeyStates& theKeyStates) override;

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theSystemSensorData The representation updated.
   */
  void update(SystemSensorData& theSystemSensorData) override;

  /**
   * Wait for a packet from LoLA and accept it. The packet is present in the
   * field receivedPacket. If this is the first packet accepted, all the pointers
   * intended to point into receivedPacket and packetToSend are initialized.
   */
  void receivePacket();

  /**
   * Write a range of leds to the packet to send and initialise the pointers
   * intended to point into packetToSend for this range.
   * @param category LoLA's name for the led range.
   * @param ledMappings Mappings from LoLA's LED indices to B-Human's LED indices.
   * @param numOfLEDs How many leds are in the range?
   * @param p A pointer into packetToSend. It is advanced by the amount of data
   *          written.
   */
  void writeLEDs(const std::string category, const LEDRequest::LED* ledMapping, int numOfLEDs, unsigned char*& p);

  /** Send packet to LoLA. */
  void sendPacket();

public:
  /** Set this as the only instance, zero all pointers, and open UDP port. */
  NaoProvider();

  /** Destructor. Closes the socket and zeros the instance pointer. */
  ~NaoProvider();

  /**
   * Call receivePacket for the only instance of this module.
   * Note that finishFrame must have been called at least once before this
   * function can be called.
   */
  static void waitForFrameData();

  /** Send requests to LoLA. */
  static void finishFrame();
};

#else // !defined TARGET_ROBOT

class NaoProvider : public NaoProviderBase
{
  void update(FrameInfo&) {}
  void update(FsrSensorData&) {}
  void update(InertialSensorData&) {}
  void update(JointSensorData&) {}
  void update(KeyStates&) {}
  void update(SystemSensorData&) {}

public:
  static void waitForFrameData() {}
  static void finishFrame() {}
};

#endif
