/**
 * @file GetUpEngine.h
 * A minimized motion engine for standing up.
 * @author <A href="mailto:judy@tzi.de">Judith Müller</A>
 */

#pragma once
#include "Representations/MotionControl/GetUpEngineOutput.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionSelection.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Sensing/FallDownState.h"
//#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/InertialData.h"
#include "Tools/Module/Module.h"

STREAMABLE(MofLine,
{,
  (float[2]) head,
  (float[6]) leftArm,
  (float[6]) rightArm,
  (float[6]) leftLeg,
  (float[6]) rightLeg,
  (float) duration,
  (bool) critical,
});

STREAMABLE(Mof,
{
  ENUM(Motion,
  {, //all motions defined in getUpEngine.cfg
    front,
    back,
    recovering,
    recoverFromSide,
    stand,
    fromSumo,
    fromSumoKeeper,
    fromLiftBall,
  }),

  (Motion) name,
  (std::vector<MofLine>) lines,
  (int) balanceStartLine,
  (Pose2f) odometryOffset,
});

MODULE(GetUpEngine,
{,
  //REQUIRES(GroundContactState),
  REQUIRES(FallDownState),
  REQUIRES(FrameInfo),
  REQUIRES(InertialData),
  REQUIRES(JointAngles),
  REQUIRES(MotionSelection),
  USES(MotionInfo),
  PROVIDES(GetUpEngineOutput),
  LOADS_PARAMETERS(
  {,
    (int) maxNumOfUnsuccessfulTries,
    (Mof[Mof::numOfMotions]) mofs,
  }),
});

class GetUpEngine : public GetUpEngineBase
{
private:
  ENUM(EngineState,
  {,
    decideAction,
    working,
    breakUp,
    recover,
    pickUp,
    schwalbe,
  });

  EngineState state; // defines the current engine state

  bool wasActive, //if the engine was active in the last frame
       balance;   //toggles balance on or off

  int lineCounter, //the current key frame of the motion sequence
       maxCounter, //the number of key frames of the current motion sequence
       motionID;  //the index of the current motion (index in mofs)

  float ratio; //0-1 interpolation state between key frames

  MotionInfo lastMotionInfo; //saves the last MotionInfo

  unsigned int lineStartTimeStamp,  //time stamp when a the interpolation between key frames started
               lastNotActiveTimeStamp, //time stamp of the last time the engine was not active
               breakUpTimeStamp,  //time stamp when the last unsuccesful try was detected
               soundTimeStamp; //time stamp to coordinate the cry for help (request for pickup)

  JointAngles startJoints; //measured joint data when the last key frame was passed
  JointRequest lastUnbalanced, //calculated last target joint angles without balance added
               targetJoints;   //calculated next target joint angles without balance added

  Pose2f internalOdometryOffset; //stores the next odometry offset
  float kp, //pid factors
        ki,
        kd,
        gBuffer, //buffer to summarize gyro data
        gLast; //stores last gyro data
public:

  void update(GetUpEngineOutput& output);

  GetUpEngine() :
    state(decideAction),
    wasActive(false),
    balance(false),
    lineCounter(0),
    maxCounter(-1),
    motionID(-1),
    lineStartTimeStamp(0),
    lastNotActiveTimeStamp(0),
    breakUpTimeStamp(0),
    soundTimeStamp(0),
    internalOdometryOffset(Pose2f()),
    kp(2.5f),
    ki(0.1f),
    kd(0.01f),
    gBuffer(0.f),
    gLast(0.f)
  {}

  /**
  * The method initializes the next motion based on the current engine state,
  * the FallDownState and the measured body angle.
  * @param output the GetUpEngineOutput object for changing joint stiffness
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
  */
  void interpolate(const JointAngles& from, const JointRequest& to, float& ratio, JointRequest& target);

  /**
  * The method adds a gyro feedback balance using PID control to the calculated jointRequest
  * @param jointRequest joint request the balance should be added to
  */
  void addBalance(JointRequest& jointRequest);

  /**
  * The method checks if a critical part is reached. If the condition is not fullfilled the motion is stopped
  * @param output the GetUpEngineOutput object for changing joint stiffness to 0
  */
  void checkCriticalParts(GetUpEngineOutput& output);

  /**
  * The method calculates the next JointRequest and counts the key frames up
  * @param output the GetUpEngineOutput object for changing target joint angles
  */
  void setNextJoints(GetUpEngineOutput& output);

  /**
  * The method sets a stiffness to the output
  * @param output the GetUpEngineOutput object the stiffness should be set to
  * @param stiffness 0-100 percent stiffness
  */
  void setStiffness(GetUpEngineOutput& output, int stiffness);

  /**
  * The method looks up the current motion parameters and intializes maxLineCounter, internalOdometryOffset, and motionID
  * @param current the motion to be initialized
  */
  void setCurrentMotion(Mof::Motion current);
};
