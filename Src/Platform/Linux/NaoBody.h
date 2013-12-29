/**
* @file Platform/linux/NaoBody.h
* Declaration of a class for accessing the body of the nao via NaoQi/libbhuman
* @author Colin Graf
*/

#pragma once

#include "Representations/Infrastructure/RoboCupGameControlData.h"

/**
* @class NaoBody
* Encapsulates communication with libbhuman.
*/
class NaoBody
{
public:

  /** Default constructor */
  NaoBody();

  /** Destructor */
  ~NaoBody();

  /** Initializes access to libbhuman
  * @return Whether the initialization was successful or not */
  bool init();

  /** Finalize access to libbhuman */
  void cleanup();

  /** Waits for a new set of sensor data */
  bool wait();

  /** Activates the eye-blinking mode to indicate a crash.
  * @param termSignal The termination signal that was raised by the crash. */
  void setCrashed(int termSignal);

  /** Accesses the name of the robot's body.
  * @return The name. */
  const char* getName() const;

  /** Accesses the sensor value buffer.
  * @return An array of sensor values. Ordered corresponding to \c lbhSensorNames of \c bhuman.h. */
  float* getSensors();

  /** Accesses the lastst data from the GameController. */
  const RoboCup::RoboCupGameControlData& getGameControlData() const;

  /** Accesses temperature sensors */
  void getTemperature(float& cpu, float& mb);

  /** Determine status of wlan hardware. */
  bool getWlanStatus();

  /** Accesses the actuator value buffer for writing.
  * @param actuators A reference to a variable to store a pointer to the actuator value buffer in. Ordered corresponding to \c lbhActuatorNames of \c bhuman.h. */
  void openActuators(float*& actuators);

  /** Commits the actuator value buffer. */
  void closeActuators();

  /** Sets the current team info that will be used by libgamectrl. */
  void setTeamInfo(int teamNumber, int teamColour, int playerNumber);

private:
  int writingActuators; /**< The index of the opened exclusive actuator writing buffer. */

  FILE* fdCpuTemp;
};
