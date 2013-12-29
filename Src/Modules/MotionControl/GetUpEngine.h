/**
* @file GetUpEngine.h
* A minimized motion engine for standing up.
* @author <A href="mailto:judy@tzi.de">Judith Müller</A>
*/

#pragma once
#include "Tools/Module/Module.h"
#include "Tools/Streams/InStreams.h"
#include "Representations/MotionControl/GetUpEngineOutput.h"
#include "Representations/MotionControl/MotionSelection.h"
#include "Platform/File.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/GroundContactState.h"
#include <cstring>

STREAMABLE(MofLine,
{
public: ,
  (float [2]) head,
  (float [4]) leftArm,
  (float [4]) rightArm,
  (float [6]) leftLeg,
  (float [6]) rightLeg,
  (float) duration,
  (bool) critical,
});

STREAMABLE(Mof,
{
public:
  ENUM(Motion, //all motions defined in getUpEngine.cfg
  front,
  back,
  recovering,
  stand);
  ,
  (Motion) name,
  (std::vector<MofLine>) lines,
  (int) balanceStartLine,
  (Pose2D) odometryOffset,
});

MODULE(GetUpEngine)
REQUIRES(MotionSelection)
REQUIRES(FilteredJointData)
REQUIRES(FrameInfo)
REQUIRES(FilteredSensorData)
REQUIRES(RobotDimensions)
REQUIRES(MassCalibration)
REQUIRES(FallDownState)
REQUIRES(GroundContactState)
PROVIDES_WITH_MODIFY(GetUpEngineOutput)
LOADS_PARAMETER(int, maxNumOfUnsuccessfulTries)
LOADS_PARAMETER(Mof[Mof::numOfMotions], mofs)
END_MODULE

class GetUpEngine : public GetUpEngineBase
{
private:
  ENUM(EngineState, decideAction, startUp, working, breakUp, recover, pickUp, schwalbe);

  EngineState state; // defines the current engine state

  bool wasActive, //if the engine was active in the last frame
       balance;   //toggles balance on or off

  int lineCounter, //the current key frame of the motion sequence
       maxCounter, //the number of key frames of the current motion sequence 
       tryCounter, //the number of unsuccessful tries
       motionID;  //the index of the current motion (index in mofs)

  float ratio; //0-1 interpolation state between key frames

  unsigned int lineStartTimeStamp,  //time stamp when a the interpolation between key frames started
               lastNotActiveTimeStamp, //time stamp of the last time the engine was not active
               breakUpTimeStamp,  //time stamp when the last unsuccesful try was detected
               soundTimeStamp; //time stamp to coordinate the cry for help (request for pickup)

  JointData startJoints; //measured joint data when the last key frame was passed
  JointRequest lastUnbalanced, //calculated last target joint angles without balance added
               targetJoints;   //calculated next target joint angles without balance added

  Pose2D internalOdometryOffset; //stores the next odometry offset 
  float kp, //pid factors
        ki, 
        kd, 
        gBuffer, //buffer to summarize gyro data 
        gLast; //stores last gyro data
public:

  void update(GetUpEngineOutput& output);

  GetUpEngine():
    state(decideAction),
    wasActive(false),
    balance(false),
    lineCounter(0),
    maxCounter(-1),
    tryCounter(0),
    motionID(-1),
    lineStartTimeStamp(0),
    lastNotActiveTimeStamp(0),
    breakUpTimeStamp(0),
    soundTimeStamp(0),
    internalOdometryOffset(Pose2D()),
    kp(2.5f),
    ki(0.1f),
    kd(0.01f),
    gBuffer(0.f),
    gLast(0.f)
  {};
 
  /**
  * The method initializes the next motion based on the current engine state, 
  * the FallDownState and the measured body angle.
  * @param output the GetUpEngineOutput object for changing joint hardness
  */
  void pickMotion(GetUpEngineOutput& output);

  /**
  * The method initializes the global variables such as lineCounter
  */
  void initVariables();

  /**
  * The method interpolates between two joint requests.
  * @param from The first source joint request. This one is the starting point.
  * @param to The second source joint request. This one has to be reached over time.
  * @param fromRatio The ratio of "from" in the target joint request.
  * @param target The target joint request.
  * @param interpolateHardness Whether to interpolate hardness.
  */
  void interpolate(const JointData& from, const JointRequest& to, float& ratio, JointRequest& target);

  /**
  * The method adds a gyro feedback balance using PID control to the calculated jointRequest
  * @param jointRequest joint request the balance should be added to
  */
  void addBalance(JointRequest& jointRequest);

  /**
  * The method checks if a critical part is reached. If the condition is not fullfilled the motion is stopped
  * @param output the GetUpEngineOutput object for changing joint hardness to 0
  */
  void checkCriticalParts(GetUpEngineOutput& output);

  /**
  * The method calculates the next JointRequest and counts the key frames up
  * @param output the GetUpEngineOutput object for changing target joint angles
  */
  void setNextJoints(GetUpEngineOutput& output);

  /**
  * The method sets a hardness to the output
  * @param output the GetUpEngineOutput object the hardness should be set to
  * @param hardness 0-100 percent hardness
  */
  void setHardness(GetUpEngineOutput& output, int hardness);

  /**
  * The method looks up the current motion parameters and intializes maxLineCounter, internalOdometryOffset, and motionID
  * @param current the motion to be initialized
  */
  void setCurrentMotion(Mof::Motion current);
};
