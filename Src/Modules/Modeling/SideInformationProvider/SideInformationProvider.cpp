/**
* @file SideInformationProvider.cpp
*
* Implementation of the module SideInformationProvider.
* The SPL field is symmetric and perceiving the field elements alone does not
* allow to infer the correct playing direction.
* This module computes some information
* that might help to solve this problem in some situations.
* One part is to compute, if a robot MUST be inside its ow half given the walked distance and the current rules of the game.
* The other part is a flip detection based on a comparison of own and team observations of the ball.
*
* @author Tim Laue
*/

#include "SideInformationProvider.h"
#include "Platform/SystemCall.h"
#include "Tools/Math/Geometry.h"

MAKE_MODULE(SideInformationProvider, modeling);

SideInformationProvider::SideInformationProvider()
{
  distanceWalkedAtKnownPosition = 0.f;
  largestXPossibleAtKnownPosition = 0.f;
  agreemates.reserve(4);
}

void SideInformationProvider::update(SideInformation& sideInformation)
{
  // Update some information
  computeBasicOwnSideInformation(sideInformation);

  // If we mirrored during the last frame, new agreements have to be found
  if(sideInformation.mirror)
    agreemates.clear();

  // In some set plays the ball is replaced, this might be confusing.
  // Thus, all agreements become invalid in this case.
  if((theExtendedGameInfo.setPlayLastFrame != SET_PLAY_GOAL_KICK && theGameInfo.setPlay == SET_PLAY_GOAL_KICK) ||
     (theExtendedGameInfo.setPlayLastFrame != SET_PLAY_CORNER_KICK && theGameInfo.setPlay == SET_PLAY_CORNER_KICK))
    timeWhenLastBallReplacingSetPlayStarted = theFrameInfo.time;
  if(theFrameInfo.getTimeSince(timeWhenLastBallReplacingSetPlayStarted) < 4000) // time is in seconds since last time the ball was put in again
    agreemates.clear();

  // Only compute/update internal representation when we are actually playing.
  if(theGameInfo.state == STATE_PLAYING && theRobotInfo.penalty == PENALTY_NONE)
  {
    // Update the structures that contain information about agreed and disagreed ball perceptions
    findAgreemates();
    removeOldAndJumpedAgreemates();
  }
  else
  {
    agreemates.clear();
  }

  // Finalize the representation:
  setMirrorFlagOfRepresentation(sideInformation);

  // Draw internal representations
  draw();

  // !
  if(sideInformation.mirror)
    SystemCall::playSound("theFlippingChicken.wav");
}

void SideInformationProvider::setMirrorFlagOfRepresentation(SideInformation& sideInformation)
{
  // Initialize representation
  sideInformation.mirror = false;

  // Goalie does not do this flipping stuff
  if(theRobotInfo.isGoalkeeper() && deactivateGoalieFlipping && theGameInfo.state == STATE_PLAYING)
    return;

  // Find all my dis-/agreemates:
  std::vector<int> agree;
  std::vector<int> disagree;
  bool goalieAgrees = false, goalieDisagrees = false;
  for(auto& agreemate : agreemates)
  {
    if(agreemate.ballCompatibility == Mirror && agreemate.agreementCount > minAgreementCount)
    {
      disagree.push_back(agreemate.number);
      if(agreemate.isGoalkeeper)
        goalieDisagrees = true;
    }
    else if(agreemate.ballCompatibility == Agree && agreemate.agreementCount > minAgreementCount)
    {
      agree.push_back(agreemate.number);
      if(agreemate.isGoalkeeper)
        goalieAgrees = true;
    }
  }

  // Set representation:
  if(goalieIsAlwaysRight ? goalieAgrees : (agree.size() >= disagree.size()))
    return;
  if(goalieIsAlwaysRight ? goalieDisagrees : (disagree.size() >= 2 && agree.size() <= 1))
    sideInformation.mirror = true;
}

void SideInformationProvider::findAgreemates()
{
  // If I have not seen the ball recently, no matches can be found anyway ...
  if(!ballModelCanBeUsed(theBallModel, theWorldModelPrediction.robotPose))
    return;
  const Vector2f myGlobalBallPosition = theWorldModelPrediction.robotPose * theBallModel.estimate.position;
  const Vector2f myGlobalBallPositionMirrored(-myGlobalBallPosition.x(), -myGlobalBallPosition.y());
  // Check all teammates:
  for(const auto& teammate : theTeamData.teammates)
  {
    // Robots that are not playing are of no interest
    if(teammate.status != Teammate::PLAYING)
      continue;
    if(ballModelCanBeUsed(teammate.theBallModel, teammate.theRobotPose))
    {
      const Vector2f teammateGlobalBallPosition = teammate.theRobotPose * teammate.theBallModel.estimate.position;
      const TeammateBallCompatibility bc = compareBallWithTeammateBall(myGlobalBallPosition, myGlobalBallPositionMirrored,
                                                                       teammateGlobalBallPosition);
      addToAgreemateList(teammate, bc);
    }
  }
}

void SideInformationProvider::removeOldAndJumpedAgreemates()
{
  // I have jumped recently, all agreements are considered as invalid:
  if(theFrameInfo.getTimeSince(theRobotPose.timestampLastJump) < jumpRemovalTimeout)
    agreemates.clear();
  // Agreements have not been updated for some time -> remove them:
  std::vector<Agreemate>::iterator agreemate = agreemates.begin();
  while(agreemate != agreemates.end())
  {
    if(theFrameInfo.getTimeSince(agreemate->timeOfLastAgreement) > agreementTimeout)
      agreemate = agreemates.erase(agreemate);
    else
      ++agreemate;
  }
  // Teammates might have jumped recently -> agreements with them are considered as invalid:
  agreemate = agreemates.begin();
  while(agreemate != agreemates.end())
  {
    const int agreemateNumber = agreemate->number;
    bool agreeMateErased = false;
    for(const auto& teammate : theTeamData.teammates)
    {
      if(teammate.number == agreemateNumber)
      {
        if(theFrameInfo.getTimeSince(teammate.theRobotPose.timestampLastJump) < jumpRemovalTimeout)
        {
          agreemate = agreemates.erase(agreemate);
          agreeMateErased = true;
          break;
        }
      }
    }
    if(!agreeMateErased)
      ++agreemate;
  }
}

void SideInformationProvider::addToAgreemateList(const Teammate& teammate,
                                                 SideInformationProvider::TeammateBallCompatibility ballCompatibility)
{
  const Vector2f globalBallPos = teammate.theRobotPose * teammate.theBallModel.estimate.position;
  Agreemate* am = nullptr;
  for(auto& agreemate : agreemates)
  {
    if(agreemate.number == teammate.number)
    {
      am = &agreemate;
      break;
    }
  }
  unsigned timeOfAgreement = std::min(theBallModel.timeWhenLastSeen, teammate.theBallModel.timeWhenLastSeen);
  if(am != nullptr)
  {
    if(timeOfAgreement > am->timeOfLastAgreement)
    {
      am->timeOfLastAgreement = timeOfAgreement;
      am->lastGlobalBallPosition = globalBallPos;
      if(ballCompatibility == am->ballCompatibility)
      {
        am->agreementCount++;
      }
      else // Uhmm, something seems to have changed. Reset counter.
      {
        am->ballCompatibility = ballCompatibility;
        am->agreementCount = 1;
      }
    }
  }
  else
  {
    Agreemate newMate;
    newMate.number = teammate.number;
    newMate.isGoalkeeper = teammate.isGoalkeeper;
    newMate.timeOfLastAgreement = timeOfAgreement;
    newMate.ballCompatibility = ballCompatibility;
    newMate.agreementCount = 1;
    newMate.lastGlobalBallPosition = globalBallPos;
    agreemates.push_back(newMate);
  }
}

SideInformationProvider::TeammateBallCompatibility SideInformationProvider::compareBallWithTeammateBall(
  const Vector2f& globalBallPosition,
  const Vector2f& globalBallPositionMirrored,
  const Vector2f& teammateGlobalBallPosition) const
{
  const float distanceToMyBall = (globalBallPosition - teammateGlobalBallPosition).norm();
  const float distanceToMyMirroredBall = (globalBallPositionMirrored - teammateGlobalBallPosition).norm();
  if(distanceToMyBall > 3.f * distanceToMyMirroredBall && distanceToMyMirroredBall < maxBallAgreementDistance)
    return Mirror;
  else if(distanceToMyMirroredBall > 3.f * distanceToMyBall && distanceToMyBall < maxBallAgreementDistance)
    return Agree;
  return Unknown;
}

bool SideInformationProvider::ballModelCanBeUsed(const BallModel& ballModel, const Pose2f& robotPose) const
{
  // Is the ball model still "fresh"?
  if(theFrameInfo.getTimeSince(ballModel.timeWhenLastSeen) > maxBallAge)
    return false;
  // We do not want to consider fast rolling balls for the SideInformation:
  if(ballModel.estimate.velocity.norm() > maxBallVelocity)
    return false;
  // Close to the field center, the ball information does not help:
  const Vector2f globalPos = robotPose * ballModel.estimate.position;
  if(globalPos.norm() < centerBanZoneRadius)
    return false;
  // As the perception of the new ball might be unreliable, we only consider stuff away from lines:
  if(ballIsCloseToPenaltyMarkOrLine(globalPos))
    return false;
  return true;
}

bool SideInformationProvider::ballIsCloseToPenaltyMarkOrLine(const Vector2f& b) const
{
  const Vector2f ownPenaltyMark =      Vector2f(theFieldDimensions.xPosOwnPenaltyMark, 0.f);
  const Vector2f opponentPenaltyMark = Vector2f(theFieldDimensions.xPosOpponentPenaltyMark, 0.f);
  if((b - ownPenaltyMark).norm() < fieldLineBanDistance)
    return true;
  if((b - opponentPenaltyMark).norm() < fieldLineBanDistance)
    return true;
  for(size_t i = 0, count = theFieldDimensions.fieldLines.lines.size(); i < count; ++i)
  {
    const FieldDimensions::LinesTable::Line& l = theFieldDimensions.fieldLines.lines[i];
    if(!l.isPartOfCircle)
    {
      Geometry::Line gl(l.from, l.to - l.from);
      if(Geometry::getDistanceToEdge(gl, b) < fieldLineBanDistance)
        return true;
    }
  }
  for(size_t i = 0, count = theFieldDimensions.goalFrameLines.lines.size(); i < count; ++i)
  {
    const FieldDimensions::LinesTable::Line& l = theFieldDimensions.goalFrameLines.lines[i];
    if(!l.isPartOfCircle)
    {
      Geometry::Line gl(l.from, l.to - l.from);
      if(Geometry::getDistanceToEdge(gl, b) < fieldLineBanDistance)
        return true;
    }
  }
  return false;
}

void SideInformationProvider::computeBasicOwnSideInformation(SideInformation& sideInformation)
{
  if(theGameInfo.gamePhase != GAME_PHASE_PENALTYSHOOT)
  {
    if(theExtendedGameInfo.gameStateLastFrame == STATE_INITIAL && theGameInfo.state == STATE_READY)
    {
      distanceWalkedAtKnownPosition = theOdometer.distanceWalked;
      largestXPossibleAtKnownPosition = theSetupPoses.getPoseOfRobot(theOwnTeamInfo.getSubstitutedPlayerNumber(theRobotInfo.number)).position.x();
    }
    else if(theExtendedGameInfo.returnFromManualPenalty)
    {
      largestXPossibleAtKnownPosition = -theFieldDimensions.centerCircleRadius - awayFromLineDistance;
      distanceWalkedAtKnownPosition = theOdometer.distanceWalked;
    }
    else if(theExtendedGameInfo.returnFromGameControllerPenalty)
    {
      distanceWalkedAtKnownPosition = theOdometer.distanceWalked;
      largestXPossibleAtKnownPosition = theFieldDimensions.xPosOwnPenaltyMark + 700;
    }
    else if(theExtendedGameInfo.gameStateLastFrame == STATE_SET && theExtendedGameInfo.setPlayLastFrame != SET_PLAY_PENALTY_KICK && theGameInfo.state == STATE_PLAYING)
    {
      distanceWalkedAtKnownPosition = theOdometer.distanceWalked;
      if(theExtendedGameInfo.manuallyPlaced)
      {
        if(theRobotInfo.isGoalkeeper())
          largestXPossibleAtKnownPosition = theFieldDimensions.xPosOwnGroundLine + 52.f;
        else if(theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber)
          largestXPossibleAtKnownPosition = -theFieldDimensions.centerCircleRadius - awayFromLineDistance;
        else
          largestXPossibleAtKnownPosition = theFieldDimensions.xPosOwnPenaltyArea + 127.f;
      }
      else
        largestXPossibleAtKnownPosition = -awayFromLineDistance;
    }
  }
  else if(theExtendedGameInfo.gameStateLastFrame == STATE_SET)
  {
    distanceWalkedAtKnownPosition = theOdometer.distanceWalked;
    if(theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber)
      largestXPossibleAtKnownPosition = theFieldDimensions.xPosPenaltyStrikerStartPosition;
    else
      largestXPossibleAtKnownPosition = theFieldDimensions.xPosOwnGroundLine;
  }

  if(theStaticInitialPose.isActive && theRobotInfo.penalty == PENALTY_NONE
     && (theExtendedGameInfo.penaltyLastFrame == PENALTY_SPL_PLAYER_PUSHING || theExtendedGameInfo.penaltyLastFrame == PENALTY_MANUAL))
    largestXPossibleAtKnownPosition = theStaticInitialPose.staticPoseOnField.translation.x();

  sideInformation.largestXCoordinatePossible = largestXPossibleAtKnownPosition + distanceUncertaintyOffset +
                                              (theOdometer.distanceWalked - distanceWalkedAtKnownPosition) * distanceUncertaintyFactor;
  sideInformation.robotMustBeInOwnHalf = sideInformation.largestXCoordinatePossible < 0 || forceOwnHalf;
  sideInformation.robotMustBeInOpponentHalf = forceOpponentHalf;
}

void SideInformationProvider::draw()
{
  DECLARE_DEBUG_DRAWING("module:SideInformationProvider:ballAgreements", "drawingOnField");
  ColorRGBA agreeColor(100, 255, 100, 240);
  ColorRGBA mirrorColor(255, 50, 50, 240);
  ColorRGBA unknownColor(200, 200, 200, 240);
  for(auto& agreemate : agreemates)
  {
    const Vector2f& b = agreemate.lastGlobalBallPosition;
    const ColorRGBA& c = agreemate.ballCompatibility == Agree ? agreeColor : agreemate.ballCompatibility == Mirror ? mirrorColor : unknownColor;
    CIRCLE("module:SideInformationProvider:ballAgreements",
           b.x(), b.y(), 80, 20, Drawings::solidPen, c, Drawings::solidBrush, c);
  }
}
