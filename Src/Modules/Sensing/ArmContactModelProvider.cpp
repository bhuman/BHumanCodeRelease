/**
* @file ArmContactModelProvider.h
*
* Implementation of class ArmContactModelProvider.
* @author <a href="mailto:fynn@informatik.uni-bremen.de">Fynn Feldpausch</a>
* @author <a href="mailto:simont@informatik.uni-bremen.de">Simon Taddiken</a>
* @author <a href="mailto:arneboe@informatik.uni-bremen.de">Arne Böckmann</a>
*/

#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Common.h"
#include "ArmContactModelProvider.h"
#include <sstream>
#include <cmath>
#include <algorithm>

/** Scale factor for debug drawings */
#define SCALE 20

MAKE_MODULE(ArmContactModelProvider, Sensing);

ArmContactModelProvider::ArmContactModelProvider()
: soundDelay(1000), lastSoundTime(0), lastGameState(STATE_INITIAL)
{
}

void ArmContactModelProvider::checkArm(bool left, float factor)
{
  Vector2f retVal;
  /* Calculate arm diffs */
  struct ArmAngles angles = angleBuffer[frameDelay];
  retVal.x = left
                  ? theFilteredJointData.angles[JointData::LShoulderPitch] - angles.leftX
                  : theFilteredJointData.angles[JointData::RShoulderPitch] - angles.rightX;
  retVal.y = left
                  ? theFilteredJointData.angles[JointData::LShoulderRoll] - angles.leftY
                  :  theFilteredJointData.angles[JointData::RShoulderRoll] - angles.rightY;
  retVal *= factor;

  if(left)
  {
    ARROW("module:ArmContactModelProvider:armContact", 0, 0, -(toDegrees(retVal.y) * SCALE), toDegrees(retVal.x) * SCALE, 20, Drawings::ps_solid, ColorClasses::blue);
    leftErrorBuffer.add(retVal);
  }
  else
  {
    ARROW("module:ArmContactModelProvider:armContact", 0, 0, (toDegrees(retVal.y) * SCALE), toDegrees(retVal.x) * SCALE, 20, Drawings::ps_solid, ColorClasses::blue);
    rightErrorBuffer.add(retVal);
  }
}

void ArmContactModelProvider::resetAll(ArmContactModel& model)
{
  model.contactLeft = false;
  model.contactRight = false;
  angleBuffer.init();
  leftErrorBuffer.init();
  rightErrorBuffer.init();
}

void ArmContactModelProvider::update(ArmContactModel& model)
{
  DECLARE_PLOT("module:ArmContactModelProvider:errorLeftX");
  DECLARE_PLOT("module:ArmContactModelProvider:errorRightX");
  DECLARE_PLOT("module:ArmContactModelProvider:errorLeftY");
  DECLARE_PLOT("module:ArmContactModelProvider:errorRightY");
  DECLARE_PLOT("module:ArmContactModelProvider:errorDurationLeft");
  DECLARE_PLOT("module:ArmContactModelProvider:errorDurationRight");
  DECLARE_PLOT("module:ArmContactModelProvider:errorYThreshold");
  DECLARE_PLOT("module:ArmContactModelProvider:errorXThreshold");
  DECLARE_PLOT("module:ArmContactModelProvider:contactLeft");
  DECLARE_PLOT("module:ArmContactModelProvider:contactRight");

  DECLARE_DEBUG_DRAWING("module:ArmContactModelProvider:armContact", "drawingOnField");

	// If in INITIAL or FINISHED, or we are penalized, we reset all buffered errors and detect no arm contacts
	if (theGameInfo.state == STATE_INITIAL || theGameInfo.state == STATE_FINISHED ||
    theFallDownState.state == FallDownState::onGround ||
    theFallDownState.state == FallDownState::falling ||
    theMotionInfo.motion == MotionRequest::getUp ||
    theMotionInfo.motion == MotionRequest::specialAction ||
    theMotionInfo.motion == MotionRequest::bike ||
    !theGroundContactState.contact ||
    (theRobotInfo.penalty != PENALTY_NONE && !detectWhilePenalized))
  {
    resetAll(model);
    return;
  }

  // check if any arm just finished moving, if so reset its error buffer
  if(leftMovingLastFrame && !theArmMotionEngineOutput.arms[LEFT].move)
  {
    model.contactLeft = false;
    leftErrorBuffer.init();
  }

  if(rightMovingLastFrame && !theArmMotionEngineOutput.arms[RIGHT].move)
  {
    model.contactLeft = false;
    rightErrorBuffer.init();
  }

  leftMovingLastFrame = theArmMotionEngineOutput.arms[LEFT].move;
  rightMovingLastFrame = theArmMotionEngineOutput.arms[RIGHT].move;



  // clear on game state changes
  if (lastGameState != theGameInfo.state)
  {
    resetAll(model);
  }
  lastGameState = theGameInfo.state;

  CIRCLE("module:ArmContactModelProvider:armContact", 0, 0, 200, 30, Drawings::ps_solid, ColorClasses::blue, Drawings::ps_null, ColorClasses::blue);
  CIRCLE("module:ArmContactModelProvider:armContact", 0, 0, 400, 30, Drawings::ps_solid, ColorClasses::blue, Drawings::ps_null, ColorClasses::blue);
  CIRCLE("module:ArmContactModelProvider:armContact", 0, 0, 600, 30, Drawings::ps_solid, ColorClasses::blue, Drawings::ps_null, ColorClasses::blue);
  CIRCLE("module:ArmContactModelProvider:armContact", 0, 0, 800, 30, Drawings::ps_solid, ColorClasses::blue, Drawings::ps_null, ColorClasses::blue);
  CIRCLE("module:ArmContactModelProvider:armContact", 0, 0, 1000, 30, Drawings::ps_solid, ColorClasses::blue, Drawings::ps_null, ColorClasses::blue);
  CIRCLE("module:ArmContactModelProvider:armContact", 0, 0, 1200, 30, Drawings::ps_solid, ColorClasses::blue, Drawings::ps_null, ColorClasses::blue);


  /* Buffer arm angles */
  struct ArmAngles angles;
  angles.leftX = theJointRequest.angles[JointData::LShoulderPitch];
  angles.leftY = theJointRequest.angles[JointData::LShoulderRoll];
  angles.rightX = theJointRequest.angles[JointData::RShoulderPitch];
  angles.rightY = theJointRequest.angles[JointData::RShoulderRoll];
  angleBuffer.add(angles);

  Pose2D odometryOffset = theOdometryData - lastOdometry;
  lastOdometry = theOdometryData;

  /* Check for arm contact */
  // motion types to take into account: stand, walk (if the robot is upright)
  if((theMotionInfo.motion == MotionInfo::stand || theMotionInfo.motion == MotionInfo::walk) &&
     (theFallDownState.state == FallDownState::upright || theFallDownState.state == FallDownState::staggering) &&
     angleBuffer.isFilled())
  {

    const float leftFactor = calculateCorrectionFactor(theRobotModel.limbs[MassCalibration::foreArmLeft], odometryOffset, lastLeftHandPos);
    const float rightFactor = calculateCorrectionFactor(theRobotModel.limbs[MassCalibration::foreArmRight], odometryOffset, lastRightHandPos);

    // determine if arm is not moving and time passed since last arm motion for each arm
    // HINT: both conditions should be equivalent as long as waitAfterMovement is > 0
    const bool leftValid = !leftMovingLastFrame &&
      theFrameInfo.getTimeSince(theArmMotionEngineOutput.arms[LEFT].lastMovement) > waitAfterMovement;
    const bool rightValid = !rightMovingLastFrame &&
      theFrameInfo.getTimeSince(theArmMotionEngineOutput.arms[RIGHT].lastMovement) > waitAfterMovement;

    // only integrate new measurement if arm not moving and time passed > waitAfterMovement
    if(leftValid)
      checkArm(LEFT, leftFactor);
    if(rightValid)
      checkArm(RIGHT, rightFactor);

    const Vector2f left = leftErrorBuffer.getAverageFloat();
    const Vector2f right = rightErrorBuffer.getAverageFloat();


    //Determine if we are being pushed or not. Can only be true if last arm movement long enough ago
    bool leftX  = leftValid && fabs(left.x) > fromDegrees(errorXThreshold);
    bool leftY  = leftValid && fabs(left.y) > fromDegrees(errorYThreshold);
    bool rightX = rightValid && fabs(right.x)> fromDegrees(errorXThreshold);
    bool rightY = rightValid && fabs(right.y)> fromDegrees(errorYThreshold);

    // update the model
    model.contactLeft  = (leftX || leftY)  &&
      leftErrorBuffer.isFilled();

    model.contactRight = (rightX || rightY) &&
      rightErrorBuffer.isFilled();

    // There should be no contact for an arm that is currently being moved
    ASSERT(model.contactLeft ? !theArmMotionEngineOutput.arms[LEFT].move : true);
    ASSERT(model.contactRight ? !theArmMotionEngineOutput.arms[RIGHT].move : true);

    // The duration of the contact is counted upwards as long as the error
    // remains. Otherwise it is reseted to 0.
    model.durationLeft  = model.contactLeft  ? model.durationLeft + 1 : 0;
    model.durationRight = model.contactRight ? model.durationRight + 1 : 0;

    model.contactLeft &= model.durationLeft < malfunctionThreshold;
    model.contactRight &= model.durationRight < malfunctionThreshold;


    if(model.contactLeft)
    {
      model.timeOfLastContactLeft = theFrameInfo.time;
    }
    if(model.contactRight)
    {
      model.timeOfLastContactRight = theFrameInfo.time;
    }

    model.pushDirectionLeft = getDirection(LEFT, leftX, leftY, left);
    model.pushDirectionRight = getDirection(RIGHT, rightX, rightY, right);

    model.lastPushDirectionLeft = model.pushDirectionLeft != ArmContactModel::NONE ? model.pushDirectionLeft : model.lastPushDirectionLeft;
    model.lastPushDirectionRight = model.pushDirectionRight != ArmContactModel::NONE ? model.pushDirectionRight : model.lastPushDirectionRight;

    PLOT("module:ArmContactModelProvider:errorLeftX",         toDegrees(left.x));
    PLOT("module:ArmContactModelProvider:errorRightX",        toDegrees(right.x));
    PLOT("module:ArmContactModelProvider:errorLeftY",         toDegrees(left.y));
    PLOT("module:ArmContactModelProvider:errorRightY",        toDegrees(right.y));
    PLOT("module:ArmContactModelProvider:errorDurationLeft",  model.durationLeft);
    PLOT("module:ArmContactModelProvider:errorDurationRight", model.durationRight);
    PLOT("module:ArmContactModelProvider:errorYThreshold",    errorYThreshold);
    PLOT("module:ArmContactModelProvider:errorXThreshold",    errorXThreshold);
    PLOT("module:ArmContactModelProvider:contactLeft",        model.contactLeft ? 10.0 : 0.0);
    PLOT("module:ArmContactModelProvider:contactRight",       model.contactRight ? 10.0 : 0.0);

    ARROW("module:ArmContactModelProvider:armContact", 0, 0, -(toDegrees(left.y) * SCALE), toDegrees(left.x) * SCALE, 20, Drawings::ps_solid, ColorClasses::green);
    ARROW("module:ArmContactModelProvider:armContact", 0, 0, toDegrees(right.y) * SCALE, toDegrees(right.x) * SCALE, 20, Drawings::ps_solid, ColorClasses::red);

    COMPLEX_DRAWING("module:ArmContactModelProvider:armContact",
    {
      DRAWTEXT("module:ArmContactModelProvider:armContact", -2300, 1300, 200, ColorClasses::black, "LEFT");
      DRAWTEXT("module:ArmContactModelProvider:armContact", -2300, 1100, 200, ColorClasses::black, "ErrorX: " << toDegrees(left.x));
      DRAWTEXT("module:ArmContactModelProvider:armContact", -2300, 900, 200, ColorClasses::black,  "ErrorY: " << toDegrees(left.y));
      DRAWTEXT("module:ArmContactModelProvider:armContact", -2300, 500, 200, ColorClasses::black,  ArmContactModel::getName(model.pushDirectionLeft));
      DRAWTEXT("module:ArmContactModelProvider:armContact", -2300, 300, 200, ColorClasses::black,  "Time: " << model.timeOfLastContactLeft);

      DRAWTEXT("module:ArmContactModelProvider:armContact", 1300, 1300, 200, ColorClasses::black, "RIGHT");
      DRAWTEXT("module:ArmContactModelProvider:armContact", 1300, 1100, 200, ColorClasses::black, "ErrorX: " << toDegrees(right.x));
      DRAWTEXT("module:ArmContactModelProvider:armContact", 1300, 900, 200, ColorClasses::black,  "ErrorY: " << toDegrees(right.y));
      DRAWTEXT("module:ArmContactModelProvider:armContact", 1300, 500, 200, ColorClasses::black,  ArmContactModel::getName(model.pushDirectionRight));
      DRAWTEXT("module:ArmContactModelProvider:armContact", 1300, 300, 200, ColorClasses::black,  "Time: " << model.timeOfLastContactRight);

      if (model.contactLeft)
      {
        CROSS("module:ArmContactModelProvider:armContact", -2000, 0, 100, 20, Drawings::ps_solid, ColorClasses::red);
      }
      if (model.contactRight)
      {
        CROSS("module:ArmContactModelProvider:armContact", 2000, 0, 100, 20, Drawings::ps_solid, ColorClasses::red);
      }
  });


    if(debugMode && theFrameInfo.getTimeSince(lastSoundTime) > soundDelay &&
      (model.contactLeft || model.contactRight))
    {
      lastSoundTime = theFrameInfo.time;
#ifdef TARGET_ROBOT
      SystemCall::playSound("arm.wav");
#else
      OUTPUT(idText, text, (model.contactLeft ? "Left" : "") << (model.contactRight ? "Right" : "") << " arm!");
#endif
    }

  }
}


ArmContactModel::PushDirection ArmContactModelProvider::getDirection(bool left, bool contactX, bool contactY, Vector2f error)
{
  // for the left arm, y directions are mirrored!
  if (left)
  {
    error = Vector2f(error.x, -error.y);
  }

  ArmContactModel::PushDirection result = ArmContactModel::NONE;
  if (contactX && contactY)
  {
    if (error.x > 0.0 && error.y < 0.0f)
    {
      result = ArmContactModel::NW;
    }
    else if (error.x > 0.0 && error.y > 0.0)
    {
      result = ArmContactModel::NE;
    }
    if (error.x < 0.0 && error.y < 0.0f)
    {
      result = ArmContactModel::SW;
    }
    else if (error.x < 0.0 && error.y > 0.0)
    {
      result = ArmContactModel::SE;
    }
  }
  else if (contactX)
  {
    if (error.x < 0.0)
    {
      result = ArmContactModel::S;
    }
    else
    {
      result = ArmContactModel::N;
    }
  }
  else if (contactY)
  {
    if (error.y < 0.0)
    {
      result = ArmContactModel::W;
    }
    else
    {
      result = ArmContactModel::E;
    }
  }
  else
  {
    result = ArmContactModel::NONE;
  }

  return result;
}



float ArmContactModelProvider::calculateCorrectionFactor(const Pose3D foreArm, const Pose2D odometryOffset, Vector2<> &lastArmPos)
{
  const Vector3<>& handPos3D = foreArm.translation;
  Vector2<> handPos(handPos3D.x, handPos3D.y);
  Vector2<> handSpeed = (odometryOffset + Pose2D(handPos) - Pose2D(lastArmPos)).translation / theFrameInfo.cycleTime;
  float factor = std::max(0.f, 1.f - handSpeed.abs() / speedBasedErrorReduction);
  lastArmPos = handPos;
  return factor;
}
