/**
 * @file NaoProvider.h
 *
 * This file declares a module that communicates with LoLA.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/LEDRequest.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Infrastructure/SensorData/RawInertialSensorData.h"
#include "Representations/Infrastructure/SensorData/SystemSensorData.h"
#include "Framework/Module.h"

MODULE(NaoProvider,
{,
  REQUIRES(FrameInfo),
  REQUIRES(JointCalibration),
  USES(JointSensorData),
  USES(JointRequest),
  REQUIRES(LEDRequest),
  PROVIDES(FrameInfo),
  PROVIDES(FsrSensorData),
  PROVIDES(JointSensorData),
  PROVIDES(KeyStates),
  PROVIDES(RawInertialSensorData),
  PROVIDES(SystemSensorData),
  DEFINES_PARAMETERS(
  {,
    (int)(3000) timeChestButtonPressedUntilShutdown, /**< Time the chest button must be pressed until shutdown (in ms). */
    (int)(5000) timeBetweenCPUTemperatureUpdates, /**< Time between reading the CPU temperature (in ms). */
    (unsigned)(10) retries, /**< Number of tries to connect socket. */
    (unsigned)(10) retryDelay, /**< Delay before a retry to connect socket. */
  }),
});

#ifdef TARGET_ROBOT

class NaoProvider : public NaoProviderBase
{
  static constexpr size_t numOfCPUCores = 4; /**< The number of CPU cores. */
  static constexpr Joints::Joint numOfJoints = static_cast<Joints::Joint>(Joints::numOfJoints - 2); /**< The number of joints exchanged with LoLA. */

  static thread_local NaoProvider* theInstance; /**< The only instance of this module. */
  static const Joints::Joint jointMappings[numOfJoints]; /**< Mappings from LoLA's joint indices to B-Human's joint indices. */
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
  unsigned timeWhenChestButtonUnpressed = 0; /**< The last time the chest button was not pressed. */
  unsigned timeWhenCPUTemperatureRead = 0; /**< The last time the CPU temperature was read. */
  int cpuTemperatureFiles[numOfCPUCores]; /**< The file descriptors where the CPU core temperatures can be read from. */

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
   * @param theRawInertialSensorData The representation updated.
   */
  void update(RawInertialSensorData& theRawInertialSensorData) override;

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
   * Write a range of LEDs to the packet to send and initialize the pointers
   * intended to point into packetToSend for this range.
   * @param category LoLA's name for the led range.
   * @param ledMappings Mappings from LoLA's LED indices to B-Human's LED indices.
   * @param numOfLEDs How many LEDs are in the range?
   * @param p A pointer into packetToSend. It is advanced by the amount of data
   *          written.
   */
  void writeLEDs(const std::string category, const LEDRequest::LED* ledMapping, int numOfLEDs, unsigned char*& p);

  /** Send packet to LoLA. */
  void sendPacket();

  /**
   * Reads the temperatures of the CPU cores.
   * @return The maximum CPU temperature of any of the cores.
   */
  float readCPUTemperature() const;

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
  void update(RawInertialSensorData&) {}
  void update(JointSensorData& theJointSensorData) {static_cast<JointAngles&>(theJointSensorData) = theJointRequest;}
  void update(KeyStates&) {}
  void update(SystemSensorData&) {}

public:
  static void waitForFrameData() {}
  static void finishFrame();
};

#endif
