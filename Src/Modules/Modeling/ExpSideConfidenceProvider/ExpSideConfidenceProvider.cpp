/**
 * @file ExpSideConfidenceProvider.h
 *
 * Calculates the SideConfidence
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#include "ExpSideConfidenceProvider.h"
#include "Platform/SystemCall.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Probabilistics.h"

MAKE_MODULE(ExpSideConfidenceProvider, modeling)

ExpSideConfidenceProvider::ExpSideConfidenceProvider()
{
  agreemates.reserve(4);
}

void ExpSideConfidenceProvider::update(SideConfidence& sideConfidence)
{
  // Set the robot pose
  robotPose = theRobotPose + theOdometer.odometryOffset;

  // If we mirrored during the last frame, new agreements have to be found
  if(sideConfidence.mirror)
    agreemates.clear();

  // Only compute/update internal representation when we are actually playing.
  if(theGameInfo.state == STATE_PLAYING && theRobotInfo.penalty == PENALTY_NONE)
  {
    // Update the structures that contain information about agreed and disagreed ball perceptions
    findAgreemates();
    removeOldAgreemates();
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

void ExpSideConfidenceProvider::fillRepresentation(SideConfidence& sideConfidence)
{
  // Initialize representation
  sideConfidence.agreeMates.clear();
  sideConfidence.sideConfidence = 1.f;
  sideConfidence.mirror = false;
  if(theOwnSideModel.stillInOwnSide)
    sideConfidence.confidenceState = SideConfidence::CONFIDENT;
  else
    sideConfidence.confidenceState = SideConfidence::ALMOST_CONFIDENT;

  // Goalie does not do this flipping stuff
  if(Global::getSettings().isGoalkeeper && deactivateGoalieFlipping && theGameInfo.state == STATE_PLAYING)
  {
    sideConfidence.confidenceState = SideConfidence::CONFIDENT;
    return;
  }

  // Find all my dis-/agreemates:
  std::vector<int> agree;
  std::vector<int> disagree;
  for(auto& agreemate : agreemates)
  {
    if(agreemate.ballCompatibility == Mirror)
      disagree.push_back(agreemate.number);
    else if(agreemate.ballCompatibility == Agree)
      agree.push_back(agreemate.number);
  }

  // Set representation:
  if(agree.size() >= disagree.size())
  {
    sideConfidence.agreeMates = agree;
    return;
  }
  if(disagree.size() >= 2 && agree.size() == 0)
  {
    sideConfidence.agreeMates.clear();
    sideConfidence.mirror = true;
  }
}

void ExpSideConfidenceProvider::findAgreemates()
{
  // If I have not seen the ball recently, no matches can be found anyway ...
  if(!ballModelCanBeUsed(theBallModel, robotPose))
    return;
  const Vector2f myGlobalBallPosition = robotPose * theBallModel.estimate.position;
  const Vector2f myGlobalBallPositionMirrored(-myGlobalBallPosition.x(), -myGlobalBallPosition.y());
  // Check all teammates:
  for(const auto& teammate : theTeammateData.teammates)
  {
    if(teammate.status != Teammate::PLAYING)
      continue;
    if(ballModelCanBeUsed(teammate.ball, teammate.pose))
    {
      const Vector2f teammateGlobalBallPosition = teammate.pose * teammate.ball.estimate.position;
      const TeammateBallCompatibility bc = compareBallWithTeammateBall(myGlobalBallPosition, myGlobalBallPositionMirrored,
                                                                       teammateGlobalBallPosition);
      addToAgreemateList(teammate, bc);
    }
  }
}

void ExpSideConfidenceProvider::removeOldAgreemates()
{
  std::vector<Agreemate>::iterator agreemate = agreemates.begin();
  while(agreemate != agreemates.end())
  {
    if(theFrameInfo.getTimeSince(agreemate->timeOfLastAgreement) > agreementTimeout)
      agreemate = agreemates.erase(agreemate);
    else
      ++agreemate;
  }
}

void ExpSideConfidenceProvider::addToAgreemateList(const Teammate& teammate,
                                                   ExpSideConfidenceProvider::TeammateBallCompatibility ballCompatibility)
{
  Agreemate* am = nullptr;
  for(auto& agreemate : agreemates)
  {
    if(agreemate.number == teammate.number)
    {
      am = &agreemate;
      break;
    }
  }
  unsigned timeOfAgreement = std::max(theBallModel.timeWhenLastSeen, teammate.ball.timeWhenLastSeen);
  if(am != nullptr)
  {
    if(timeOfAgreement > am->timeOfLastAgreement)
    {
      am->timeOfLastAgreement = timeOfAgreement;
      am->ballCompatibility = ballCompatibility;
      am->lastMatePosition = teammate.pose.translation;
      am->otherAgreements.clear();
      for(unsigned int i=0; i<teammate.sideConfidence.agreeMates.size(); ++i)
      {
        if(teammate.sideConfidence.agreeMates[i] != theRobotInfo.number)
          am->otherAgreements.push_back(teammate.sideConfidence.agreeMates[i]);
      }
    }
  }
  else
  {
    Agreemate newMate;
    newMate.number = teammate.number;
    newMate.timeOfLastAgreement = timeOfAgreement;
    newMate.ballCompatibility = ballCompatibility;
    newMate.lastMatePosition = teammate.pose.translation;
    for(unsigned int i=0; i<teammate.sideConfidence.agreeMates.size(); ++i)
    {
      if(teammate.sideConfidence.agreeMates[i] != theRobotInfo.number)
        newMate.otherAgreements.push_back(teammate.sideConfidence.agreeMates[i]);
    }
    agreemates.push_back(newMate);
  }
}

ExpSideConfidenceProvider::TeammateBallCompatibility ExpSideConfidenceProvider::compareBallWithTeammateBall(
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

bool ExpSideConfidenceProvider::ballModelCanBeUsed(const BallModel& ballModel, const Pose2f& robotPose) const
{
  // Is the ball model still "fresh"?
  if(theFrameInfo.getTimeSince(ballModel.timeWhenLastSeen) > maxBallAge)
    return false;
  // We do not want to consider fast rolling balls for the SideConfidence:
  if(ballModel.estimate.velocity.norm() > maxBallVelocity)
    return false;
  // Close to the field center, the ball information does not help:
  Vector2f globalPos = theRobotPose * ballModel.estimate.position;
  if(globalPos.norm() < centerBanZoneRadius)
    return false;
  // As the perception of the new ball might be unreliable, we only consider stuff away from lines:
  if(ballIsCloseToPenaltyMarkOrLine(globalPos))
    return false;
  return true;
}

bool ExpSideConfidenceProvider::ballIsCloseToPenaltyMarkOrLine(const Vector2f& b) const
{
  const Vector2f ownPenaltyMark =      Vector2f(theFieldDimensions.xPosOwnPenaltyMark, 0.f);
  const Vector2f opponentPenaltyMark = Vector2f(theFieldDimensions.xPosOpponentPenaltyMark, 0.f);
  if((b-ownPenaltyMark).norm() < fieldLineBanDistance)
    return true;
  if((b-opponentPenaltyMark).norm() < fieldLineBanDistance)
    return true;
  for(size_t i = 0, count = theFieldDimensions.fieldLines.lines.size(); i < count; ++i)
  {
    const FieldDimensions::LinesTable::Line& l = theFieldDimensions.fieldLines.lines[i];
    if(!l.isPartOfCircle)
    {
      Geometry::Line gl(l.from, l.to-l.from);
      if(Geometry::getDistanceToEdge(gl, b) < fieldLineBanDistance)
        return true;
    }
  }
  for(size_t i = 0, count = theFieldDimensions.goalFrameLines.lines.size(); i < count; ++i)
  {
    const FieldDimensions::LinesTable::Line& l = theFieldDimensions.goalFrameLines.lines[i];
    if(!l.isPartOfCircle)
    {
      Geometry::Line gl(l.from, l.to-l.from);
      if(Geometry::getDistanceToEdge(gl, b) < fieldLineBanDistance)
        return true;
    }
  }
  return false;
}

void ExpSideConfidenceProvider::draw()
{
  DECLARE_DEBUG_DRAWING("module:ExpSideConfidenceProvider:agreemates", "drawingOnField");
  for(const auto& mate : agreemates)
  {
    LINE("module:ExpSideConfidenceProvider:agreemates", robotPose.translation.x(), robotPose.translation.y(),
         mate.lastMatePosition.x(), mate.lastMatePosition.y(), 100, Drawings::solidPen, ColorRGBA(0,80,0,128));
  }
}
