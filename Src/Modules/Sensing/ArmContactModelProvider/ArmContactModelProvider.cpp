/**
 * @file ArmContactModelProvider.cpp
 *
 * Implementation of class ArmContactModelProvider.
 * @author <a href="mailto:fynn@informatik.uni-bremen.de">Fynn Feldpausch</a>
 * @author <a href="mailto:simont@informatik.uni-bremen.de">Simon Taddiken</a>
 * @author <a href="mailto:arneboe@informatik.uni-bremen.de">Arne Böckmann</a>
 */

#include "ArmContactModelProvider.h"
#include "Platform/SystemCall.h"
#include "Tools/Debugging/DebugDrawings.h"
#include <algorithm>

/** Scale factor for debug drawings */
#define SCALE 20

MAKE_MODULE(ArmContactModelProvider, sensing);

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
    errorBuffer[0].push_front(retVal);
  }
  else
  {
    ARROW("module:ArmContactModelProvider:armContact", 0, 0, (toDegrees(retVal.y()) * SCALE), toDegrees(retVal.x()) * SCALE, 20, Drawings::solidPen, ColorRGBA::blue);
    errorBuffer[1].push_front(retVal);
  }
}

void ArmContactModelProvider::resetAll(ArmContactModel& model)
{
  model.status[Arms::left].contact = false;
  model.status[Arms::right].contact = false;
  angleBuffer.clear();
  for(int i = 0; i < 2; ++i)
    errorBuffer[i].clear();
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

  CIRCLE("module:ArmContactModelProvider:armContact", 0, 0, 200, 30, Drawings::solidPen, ColorRGBA::blue, Drawings::noPen, ColorRGBA::blue);
  CIRCLE("module:ArmContactModelProvider:armContact", 0, 0, 400, 30, Drawings::solidPen, ColorRGBA::blue, Drawings::noPen, ColorRGBA::blue);
  CIRCLE("module:ArmContactModelProvider:armContact", 0, 0, 600, 30, Drawings::solidPen, ColorRGBA::blue, Drawings::noPen, ColorRGBA::blue);
  CIRCLE("module:ArmContactModelProvider:armContact", 0, 0, 800, 30, Drawings::solidPen, ColorRGBA::blue, Drawings::noPen, ColorRGBA::blue);
  CIRCLE("module:ArmContactModelProvider:armContact", 0, 0, 1000, 30, Drawings::solidPen, ColorRGBA::blue, Drawings::noPen, ColorRGBA::blue);
  CIRCLE("module:ArmContactModelProvider:armContact", 0, 0, 1200, 30, Drawings::solidPen, ColorRGBA::blue, Drawings::noPen, ColorRGBA::blue);

  // If in INITIAL or FINISHED, or we are penalized, we reset all buffered errors and detect no arm contacts
  if(theGameInfo.state == STATE_INITIAL || theGameInfo.state == STATE_FINISHED ||
     theFallDownState.state == FallDownState::falling ||
     theFallDownState.state == FallDownState::fallen ||
     theMotionInfo.motion == MotionRequest::getUp ||
     theMotionInfo.motion == MotionRequest::specialAction ||
     theMotionInfo.motion == MotionRequest::kick ||
     !theGroundContactState.contact ||
     (theRobotInfo.penalty != PENALTY_NONE && !detectWhilePenalized))
  {
    resetAll(model);
    return;
  }
  // clear on game state changes
  if(lastGameState != theGameInfo.state)
    resetAll(model);
  lastGameState = theGameInfo.state;

  prepare(model);
  /* Buffer arm angles */
  angleBuffer.push_front(ArmAngles(theJointRequest.angles[Joints::lShoulderPitch], theJointRequest.angles[Joints::lShoulderRoll], theJointRequest.angles[Joints::rShoulderPitch], theJointRequest.angles[Joints::rShoulderRoll]));

  Pose2f odometryOffset = theOdometryData - lastOdometry;
  lastOdometry = theOdometryData;

  /* Check for arm contact */
  // motion types to take into account: stand, walk (if the robot is upright)
  if((theMotionInfo.motion == MotionInfo::stand || theMotionInfo.motion == MotionInfo::walk) &&
     (theFallDownState.state == FallDownState::upright || theFallDownState.state == FallDownState::staggering) &&
     angleBuffer.full())
  {
    //TODO change representation to use arrays instead of contactLeft/contactRight etc.
    bool* contact[2] = { &model.status[Arms::left].contact, &model.status[Arms::right].contact };
    ArmContactModel::PushDirection* push[2] = { &model.status[Arms::left].pushDirection,  &model.status[Arms::right].pushDirection };
    ArmContactModel::PushDirection* lastPush[2] = { &model.status[Arms::left].lastPushDirection,  &model.status[Arms::right].lastPushDirection };
    unsigned* contactTime[2] = { &model.status[Arms::left].timeOfLastContact, &model.status[Arms::right].timeOfLastContact };
    unsigned* duration[2] = { &model.status[Arms::left].duration, &model.status[Arms::right].duration };

    for(int i = 0; i < 2; i++)
    {
      if(theDamageConfigurationBody.sides[i].armContactDefect)
        continue;

      float factor = calculateCorrectionFactor(theRobotModel.limbs[i ? Limbs::foreArmLeft : Limbs::foreArmRight], odometryOffset, lastHandPos[i]);
      bool valid = !movingLastFrame[i] && theFrameInfo.getTimeSince(lastMovement[i]) > waitAfterMovement;

      if(valid)
        checkArm((i == 0), factor);

      Vector2f avg = errorBuffer[i].average();
      bool x = std::abs(avg.x()) > errorXThreshold && valid;
      bool y = std::abs(avg.y()) > errorYThreshold && valid;

      *contact[i] = (x || y) && errorBuffer[i].full();
      if(*contact[i])
      {
        *contactTime[i] = theFrameInfo.time;
        (*duration[i])++;
      }
      else
        *duration[i] = 0;

      *contact[i] &= *duration[i] < malfunctionThreshold;
      *push[i] = getDirection((i == 0), x, y, avg);

      if(*push[i] != ArmContactModel::NONE)
        *lastPush[i] = *push[i];
    }

    PLOT("module:ArmContactModelProvider:errorLeftX",         toDegrees(errorBuffer[0].average().x()));
    PLOT("module:ArmContactModelProvider:errorRightX",        toDegrees(errorBuffer[1].average().x()));
    PLOT("module:ArmContactModelProvider:errorLeftY",         toDegrees(errorBuffer[0].average().y()));
    PLOT("module:ArmContactModelProvider:errorRightY",        toDegrees(errorBuffer[1].average().y()));
    PLOT("module:ArmContactModelProvider:errorDurationLeft",  model.status[Arms::left].duration);
    PLOT("module:ArmContactModelProvider:errorDurationRight", model.status[Arms::right].duration);
    PLOT("module:ArmContactModelProvider:errorYThreshold",    errorYThreshold.toDegrees());
    PLOT("module:ArmContactModelProvider:errorXThreshold",    errorXThreshold.toDegrees());
    PLOT("module:ArmContactModelProvider:contactLeft",        model.status[Arms::left].contact ? 10.0 : 0.0);
    PLOT("module:ArmContactModelProvider:contactRight",       model.status[Arms::right].contact ? 10.0 : 0.0);

    ARROW("module:ArmContactModelProvider:armContact", 0, 0, -(toDegrees(errorBuffer[0].average().y()) * SCALE), toDegrees(errorBuffer[0].average().x()) * SCALE, 20, Drawings::solidPen, ColorRGBA::green);
    ARROW("module:ArmContactModelProvider:armContact", 0, 0, toDegrees(errorBuffer[1].average().y()) * SCALE, toDegrees(errorBuffer[1].average().x()) * SCALE, 20, Drawings::solidPen, ColorRGBA::red);

    COMPLEX_DRAWING("module:ArmContactModelProvider:armContact")
    {
      DRAWTEXT("module:ArmContactModelProvider:armContact", -2300, 1300, 200, ColorRGBA::black, "LEFT");
      DRAWTEXT("module:ArmContactModelProvider:armContact", -2300, 1100, 200, ColorRGBA::black, "ErrorX: " << toDegrees(errorBuffer[0].average().x()));
      DRAWTEXT("module:ArmContactModelProvider:armContact", -2300, 900, 200, ColorRGBA::black,  "ErrorY: " << toDegrees(errorBuffer[0].average().y()));
      DRAWTEXT("module:ArmContactModelProvider:armContact", -2300, 500, 200, ColorRGBA::black,  TypeRegistry::getEnumName(model.status[Arms::left].pushDirection));
      DRAWTEXT("module:ArmContactModelProvider:armContact", -2300, 300, 200, ColorRGBA::black,  "Time: " << model.status[Arms::left].timeOfLastContact);

      DRAWTEXT("module:ArmContactModelProvider:armContact", 1300, 1300, 200, ColorRGBA::black, "RIGHT");
      DRAWTEXT("module:ArmContactModelProvider:armContact", 1300, 1100, 200, ColorRGBA::black, "ErrorX: " << toDegrees(errorBuffer[1].average().x()));
      DRAWTEXT("module:ArmContactModelProvider:armContact", 1300, 900, 200, ColorRGBA::black,  "ErrorY: " << toDegrees(errorBuffer[1].average().y()));
      DRAWTEXT("module:ArmContactModelProvider:armContact", 1300, 500, 200, ColorRGBA::black,  TypeRegistry::getEnumName(model.status[Arms::right].pushDirection));
      DRAWTEXT("module:ArmContactModelProvider:armContact", 1300, 300, 200, ColorRGBA::black,  "Time: " << model.status[Arms::right].timeOfLastContact);

      if(model.status[Arms::left].contact)
        CROSS("module:ArmContactModelProvider:armContact", -2000, 0, 100, 20, Drawings::solidPen, ColorRGBA::red);

      if(model.status[Arms::right].contact)
        CROSS("module:ArmContactModelProvider:armContact", 2000, 0, 100, 20, Drawings::solidPen, ColorRGBA::red);
    }

    if(debugMode && theFrameInfo.getTimeSince(lastSoundTime) > soundDelay &&
       (model.status[Arms::left].contact || model.status[Arms::right].contact))
    {
      lastSoundTime = theFrameInfo.time;
      if(SystemCall::getMode() == SystemCall::physicalRobot)
        SystemCall::playSound("arm.wav");
    }
  }
}

ArmContactModel::PushDirection ArmContactModelProvider::getDirection(bool left, bool contactX, bool contactY, Vector2f error)
{
  // for the left arm, y directions are mirrored!
  if(left)
    error.y() *= -1;

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
  Vector2f handSpeed = (odometryOffset + Pose2f(handPos) - Pose2f(lastArmPos)).translation / Constants::motionCycleTime;
  float factor = std::max(0.f, 1.f - handSpeed.norm() / speedBasedErrorReduction);
  lastArmPos = handPos;
  return factor;
}

void ArmContactModelProvider::prepare(ArmContactModel& model)
{
  bool* contact[2] = {&model.status[Arms::left].contact, &model.status[Arms::right].contact};
  for(int i = 0; i < 2; i++)
  {
    const bool armIsMoving = theArmMotionSelection.targetArmMotion[i] >= ArmMotionSelection::firstNonBodyMotion || theArmMotionInfo.armMotion[i] != ArmMotionRequest::none;
    if(armIsMoving)
      lastMovement[i] = theFrameInfo.time;
    // check if any arm just finished moving, if so reset its error buffer
    if(movingLastFrame[i] && !armIsMoving)
    {
      *contact[i] = false;
      errorBuffer[i].clear();
    }
    movingLastFrame[i] = armIsMoving;
  }
}
