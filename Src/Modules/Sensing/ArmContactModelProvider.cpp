/**
 * @file ArmContactModelProvider.h
 *
 * Implementation of class ArmContactModelProvider.
 * @author <a href="mailto:fynn@informatik.uni-bremen.de">Fynn Feldpausch</a>
 * @author <a href="mailto:simont@informatik.uni-bremen.de">Simon Taddiken</a>
 * @author <a href="mailto:arneboe@informatik.uni-bremen.de">Arne Böckmann</a>
 */

#include "Tools/Debugging/DebugDrawings.h"
#include "ArmContactModelProvider.h"
#include <algorithm>

/** Scale factor for debug drawings */
#define SCALE 20

MAKE_MODULE(ArmContactModelProvider, sensing);

ArmContactModelProvider::ArmContactModelProvider() :
leftErrorBuffer(Vector2f::Zero()), rightErrorBuffer(Vector2f::Zero()),
  leftLastMovement(0), rightLastMovement(0),
  soundDelay(1000), lastSoundTime(0), lastGameState(STATE_INITIAL)
{}

void ArmContactModelProvider::checkArm(bool left, float factor)
{
  Vector2f retVal;
  /* Calculate arm diffs */
  struct ArmAngles angles = angleBuffer[frameDelay];
  retVal.x() = left
                  ? angles.leftX - theJointAngles.angles[Joints::lShoulderPitch]
                  : angles.rightX - theJointAngles.angles[Joints::rShoulderPitch];
  retVal.y() = left
                  ? theJointAngles.angles[Joints::lShoulderRoll] - angles.leftY
                  :  angles.rightY - theJointAngles.angles[Joints::rShoulderRoll];
  retVal *= factor;

  if(left)
  {
    ARROW("module:ArmContactModelProvider:armContact", 0, 0, -(toDegrees(retVal.y()) * SCALE), toDegrees(retVal.x()) * SCALE, 20, Drawings::solidPen, ColorRGBA::blue);
    leftErrorBuffer.push_front(retVal);
  }
  else
  {
    ARROW("module:ArmContactModelProvider:armContact", 0, 0, (toDegrees(retVal.y()) * SCALE), toDegrees(retVal.x()) * SCALE, 20, Drawings::solidPen, ColorRGBA::blue);
    rightErrorBuffer.push_front(retVal);
  }
}

void ArmContactModelProvider::resetAll(ArmContactModel& model)
{
  model.contactLeft = false;
  model.contactRight = false;
  angleBuffer.clear();
  leftErrorBuffer.clear();
  rightErrorBuffer.clear();
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
  if(theGameInfo.state == STATE_INITIAL || theGameInfo.state == STATE_FINISHED ||
    theFallDownState.state == FallDownState::onGround ||
    theFallDownState.state == FallDownState::falling ||
    theMotionInfo.motion == MotionRequest::getUp ||
    theMotionInfo.motion == MotionRequest::specialAction ||
    theMotionInfo.motion == MotionRequest::kick ||
    !theGroundContactState.contact ||
    (theRobotInfo.penalty != PENALTY_NONE && !detectWhilePenalized))
  {
    resetAll(model);
    return;
  }

  const bool leftArmIsMoving = theArmMotionSelection.targetArmMotion[Arms::left] != ArmMotionRequest::none ||
    theArmMotionInfo.armMotion[Arms::left] != ArmMotionRequest::none;
  const bool rightArmIsMoving = theArmMotionSelection.targetArmMotion[Arms::right] != ArmMotionRequest::none ||
    theArmMotionInfo.armMotion[Arms::right] != ArmMotionRequest::none;

  if(leftArmIsMoving)
    leftLastMovement = theFrameInfo.time;
  if(rightArmIsMoving)
    rightLastMovement = theFrameInfo.time;

  // check if any arm just finished moving, if so reset its error buffer
  if(leftMovingLastFrame && !(leftArmIsMoving))
  {
    model.contactLeft = false;
    leftErrorBuffer.clear();
  }

  if(rightMovingLastFrame && !(rightArmIsMoving))
  {
    model.contactLeft = false;
    rightErrorBuffer.clear();
  }

  leftMovingLastFrame = leftArmIsMoving;
  rightMovingLastFrame = rightArmIsMoving;

  // clear on game state changes
  if(lastGameState != theGameInfo.state)
  {
    resetAll(model);
  }
  lastGameState = theGameInfo.state;

  CIRCLE("module:ArmContactModelProvider:armContact", 0, 0, 200, 30, Drawings::solidPen, ColorRGBA::blue, Drawings::noPen, ColorRGBA::blue);
  CIRCLE("module:ArmContactModelProvider:armContact", 0, 0, 400, 30, Drawings::solidPen, ColorRGBA::blue, Drawings::noPen, ColorRGBA::blue);
  CIRCLE("module:ArmContactModelProvider:armContact", 0, 0, 600, 30, Drawings::solidPen, ColorRGBA::blue, Drawings::noPen, ColorRGBA::blue);
  CIRCLE("module:ArmContactModelProvider:armContact", 0, 0, 800, 30, Drawings::solidPen, ColorRGBA::blue, Drawings::noPen, ColorRGBA::blue);
  CIRCLE("module:ArmContactModelProvider:armContact", 0, 0, 1000, 30, Drawings::solidPen, ColorRGBA::blue, Drawings::noPen, ColorRGBA::blue);
  CIRCLE("module:ArmContactModelProvider:armContact", 0, 0, 1200, 30, Drawings::solidPen, ColorRGBA::blue, Drawings::noPen, ColorRGBA::blue);

  /* Buffer arm angles */
  struct ArmAngles angles;
  angles.leftX = theJointRequest.angles[Joints::lShoulderPitch];
  angles.leftY = theJointRequest.angles[Joints::lShoulderRoll];
  angles.rightX = theJointRequest.angles[Joints::rShoulderPitch];
  angles.rightY = theJointRequest.angles[Joints::rShoulderRoll];
  angleBuffer.push_front(angles);

  Pose2f odometryOffset = theOdometryData - lastOdometry;
  lastOdometry = theOdometryData;

  /* Check for arm contact */
  // motion types to take into account: stand, walk (if the robot is upright)
  if((theMotionInfo.motion == MotionInfo::stand || theMotionInfo.motion == MotionInfo::walk) &&
     (theFallDownState.state == FallDownState::upright || theFallDownState.state == FallDownState::staggering) &&
     angleBuffer.full())
  {
    const float leftFactor = calculateCorrectionFactor(theRobotModel.limbs[Limbs::foreArmLeft], odometryOffset, lastLeftHandPos);
    const float rightFactor = calculateCorrectionFactor(theRobotModel.limbs[Limbs::foreArmRight], odometryOffset, lastRightHandPos);

    // determine if arm is not moving and time passed since last arm motion for each arm
    // HINT: both conditions should be equivalent as long as waitAfterMovement is > 0
    const bool leftValid = !leftMovingLastFrame &&
      theFrameInfo.getTimeSince(leftLastMovement) > waitAfterMovement;
    const bool rightValid = !rightMovingLastFrame &&
      theFrameInfo.getTimeSince(rightLastMovement) > waitAfterMovement;

    // only integrate new measurement if arm not moving and time passed > waitAfterMovement
    if(leftValid)
      checkArm(LEFT, leftFactor);
    if(rightValid)
      checkArm(RIGHT, rightFactor);

    const Vector2f left = leftErrorBuffer.average();
    const Vector2f right = rightErrorBuffer.average();

    //Determine if we are being pushed or not. Can only be true if last arm movement long enough ago
    bool leftX = leftValid && std::abs(left.x()) > errorXThreshold;
    bool leftY = leftValid && std::abs(left.y()) > errorYThreshold;
    bool rightX = rightValid && std::abs(right.x()) > errorXThreshold;
    bool rightY = rightValid && std::abs(right.y()) > errorYThreshold;

    // update the model
    model.contactLeft  = (leftX || leftY)  &&
      leftErrorBuffer.full();

    model.contactRight = (rightX || rightY) &&
      rightErrorBuffer.full();

    // There should be no contact for an arm that is currently being moved
    ASSERT(model.contactLeft ? !leftArmIsMoving : true);
    ASSERT(model.contactRight ? !rightArmIsMoving : true);

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

    PLOT("module:ArmContactModelProvider:errorLeftX",         toDegrees(left.x()));
    PLOT("module:ArmContactModelProvider:errorRightX",        toDegrees(right.x()));
    PLOT("module:ArmContactModelProvider:errorLeftY",         toDegrees(left.y()));
    PLOT("module:ArmContactModelProvider:errorRightY",        toDegrees(right.y()));
    PLOT("module:ArmContactModelProvider:errorDurationLeft",  model.durationLeft);
    PLOT("module:ArmContactModelProvider:errorDurationRight", model.durationRight);
    PLOT("module:ArmContactModelProvider:errorYThreshold",    errorYThreshold.toDegrees());
    PLOT("module:ArmContactModelProvider:errorXThreshold",    errorXThreshold.toDegrees());
    PLOT("module:ArmContactModelProvider:contactLeft",        model.contactLeft ? 10.0 : 0.0);
    PLOT("module:ArmContactModelProvider:contactRight",       model.contactRight ? 10.0 : 0.0);

    ARROW("module:ArmContactModelProvider:armContact", 0, 0, -(toDegrees(left.y()) * SCALE), toDegrees(left.x()) * SCALE, 20, Drawings::solidPen, ColorRGBA::green);
    ARROW("module:ArmContactModelProvider:armContact", 0, 0, toDegrees(right.y()) * SCALE, toDegrees(right.x()) * SCALE, 20, Drawings::solidPen, ColorRGBA::red);

    COMPLEX_DRAWING("module:ArmContactModelProvider:armContact")
    {
      DRAWTEXT("module:ArmContactModelProvider:armContact", -2300, 1300, 200, ColorRGBA::black, "LEFT");
      DRAWTEXT("module:ArmContactModelProvider:armContact", -2300, 1100, 200, ColorRGBA::black, "ErrorX: " << toDegrees(left.x()));
      DRAWTEXT("module:ArmContactModelProvider:armContact", -2300, 900, 200, ColorRGBA::black,  "ErrorY: " << toDegrees(left.y()));
      DRAWTEXT("module:ArmContactModelProvider:armContact", -2300, 500, 200, ColorRGBA::black,  ArmContactModel::getName(model.pushDirectionLeft));
      DRAWTEXT("module:ArmContactModelProvider:armContact", -2300, 300, 200, ColorRGBA::black,  "Time: " << model.timeOfLastContactLeft);

      DRAWTEXT("module:ArmContactModelProvider:armContact", 1300, 1300, 200, ColorRGBA::black, "RIGHT");
      DRAWTEXT("module:ArmContactModelProvider:armContact", 1300, 1100, 200, ColorRGBA::black, "ErrorX: " << toDegrees(right.x()));
      DRAWTEXT("module:ArmContactModelProvider:armContact", 1300, 900, 200, ColorRGBA::black,  "ErrorY: " << toDegrees(right.y()));
      DRAWTEXT("module:ArmContactModelProvider:armContact", 1300, 500, 200, ColorRGBA::black,  ArmContactModel::getName(model.pushDirectionRight));
      DRAWTEXT("module:ArmContactModelProvider:armContact", 1300, 300, 200, ColorRGBA::black,  "Time: " << model.timeOfLastContactRight);

      if(model.contactLeft)
      {
        CROSS("module:ArmContactModelProvider:armContact", -2000, 0, 100, 20, Drawings::solidPen, ColorRGBA::red);
      }
      if(model.contactRight)
      {
        CROSS("module:ArmContactModelProvider:armContact", 2000, 0, 100, 20, Drawings::solidPen, ColorRGBA::red);
      }
    }

    if(debugMode && theFrameInfo.getTimeSince(lastSoundTime) > soundDelay &&
      (model.contactLeft || model.contactRight))
    {
      lastSoundTime = theFrameInfo.time;
      if(SystemCall::getMode() == SystemCall::physicalRobot)
        SystemCall::playSound("arm.wav");
      else
        OUTPUT_TEXT((model.contactLeft ? "Left" : "") << (model.contactRight ? "Right" : "") << " arm!");
    }
  }
}

ArmContactModel::PushDirection ArmContactModelProvider::getDirection(bool left, bool contactX, bool contactY, Vector2f error)
{
  // for the left arm, y directions are mirrored!
  if(left)
  {
    error.y() *= -1;
  }

  ArmContactModel::PushDirection result = ArmContactModel::NONE;
  if(contactX && contactY)
  {
    if(error.x() > 0.0 && error.y() < 0.0f)
    {
      result = ArmContactModel::NW;
    }
    else if(error.x() > 0.0 && error.y() > 0.0)
    {
      result = ArmContactModel::NE;
    }
    if(error.x() < 0.0 && error.y() < 0.0f)
    {
      result = ArmContactModel::SW;
    }
    else if(error.x() < 0.0 && error.y() > 0.0)
    {
      result = ArmContactModel::SE;
    }
  }
  else if(contactX)
  {
    if(error.x() < 0.0)
    {
      result = ArmContactModel::S;
    }
    else
    {
      result = ArmContactModel::N;
    }
  }
  else if(contactY)
  {
    if(error.y() < 0.0)
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

float ArmContactModelProvider::calculateCorrectionFactor(const Pose3f& foreArm, const Pose2f& odometryOffset, Vector2f& lastArmPos)
{
  const Vector3f& handPos3D = foreArm.translation;
  Vector2f handPos = handPos3D.topRows(2);
  Vector2f handSpeed = (odometryOffset + Pose2f(handPos) - Pose2f(lastArmPos)).translation / theFrameInfo.cycleTime;
  float factor = std::max(0.f, 1.f - handSpeed.norm() / speedBasedErrorReduction);
  lastArmPos = handPos;
  return factor;
}
