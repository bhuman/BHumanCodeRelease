/**
 * @file SideConfidenceProvider.cpp
 *
 * Calculates the SideConfidence
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#include "SideConfidenceProvider.h"
#include "Platform/SystemCall.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Probabilistics.h"

MAKE_MODULE(SideConfidenceProvider, modeling)

SideConfidenceProvider::SideConfidenceProvider()
{
  agreemates.reserve(4);
}

void SideConfidenceProvider::update(SideConfidence& sideConfidence)
{
  // If we mirrored during the last frame, new agreements have to be found
  if(sideConfidence.mirror)
    agreemates.clear();

  // In some set plays the ball is replaced, this might be confusing.
  // Thus, all agreements become invalid in this case.
  if((theCognitionStateChanges.lastSetPlay != SET_PLAY_GOAL_FREE_KICK && theGameInfo.setPlay == SET_PLAY_GOAL_FREE_KICK) ||
     (theCognitionStateChanges.lastSetPlay != SET_PLAY_CORNER_KICK && theGameInfo.setPlay == SET_PLAY_CORNER_KICK))
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

  // Compute the representation:
  fillRepresentation(sideConfidence);

  // Draw internal representations
  draw();

  // !
  if(sideConfidence.mirror)
    SystemCall::playSound("theFlippingChicken.wav");
}

void SideConfidenceProvider::fillRepresentation(SideConfidence& sideConfidence)
{
  // Initialize representation
  sideConfidence.agreeMates.clear();
  sideConfidence.mirror = false;
  if(theOwnSideModel.stillInOwnSide)
    sideConfidence.confidenceState = SideConfidence::CONFIDENT;
  else
    sideConfidence.confidenceState = SideConfidence::ALMOST_CONFIDENT;

  // Goalie does not do this flipping stuff
  if(theTeamBehaviorStatus.role.isGoalkeeper && deactivateGoalieFlipping && theGameInfo.state == STATE_PLAYING)
  {
    sideConfidence.confidenceState = SideConfidence::CONFIDENT;
    return;
  }

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
  {
    sideConfidence.agreeMates = agree;
    return;
  }
  if(goalieIsAlwaysRight ? goalieDisagrees : (disagree.size() >= 2 && agree.size() <= 1))
  {
    sideConfidence.agreeMates.clear();
    sideConfidence.mirror = true;
  }
}

void SideConfidenceProvider::findAgreemates()
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
    // Only trust B-Human robots
    if(teammate.mateType != Teammate::TeamOrigin::BHumanRobot)
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

void SideConfidenceProvider::removeOldAndJumpedAgreemates()
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

void SideConfidenceProvider::addToAgreemateList(const Teammate& teammate,
                                                SideConfidenceProvider::TeammateBallCompatibility ballCompatibility)
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

SideConfidenceProvider::TeammateBallCompatibility SideConfidenceProvider::compareBallWithTeammateBall(
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

bool SideConfidenceProvider::ballModelCanBeUsed(const BallModel& ballModel, const Pose2f& robotPose) const
{
  // Is the ball model still "fresh"?
  if(theFrameInfo.getTimeSince(ballModel.timeWhenLastSeen) > maxBallAge)
    return false;
  // We do not want to consider fast rolling balls for the SideConfidence:
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

bool SideConfidenceProvider::ballIsCloseToPenaltyMarkOrLine(const Vector2f& b) const
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

void SideConfidenceProvider::draw()
{
  DECLARE_DEBUG_DRAWING("module:SideConfidenceProvider:ballAgreements", "drawingOnField");
  ColorRGBA agreeColor(100, 255, 100, 240);
  ColorRGBA mirrorColor(255, 50, 50, 240);
  ColorRGBA unknownColor(200, 200, 200, 240);
  for(auto& agreemate : agreemates)
  {
    const Vector2f& b = agreemate.lastGlobalBallPosition;
    const ColorRGBA& c = agreemate.ballCompatibility == Agree ? agreeColor : agreemate.ballCompatibility == Mirror ? mirrorColor : unknownColor;
    CIRCLE("module:SideConfidenceProvider:ballAgreements",
           b.x(), b.y(), 80, 20, Drawings::solidPen, c, Drawings::solidBrush, c);
  }
}
