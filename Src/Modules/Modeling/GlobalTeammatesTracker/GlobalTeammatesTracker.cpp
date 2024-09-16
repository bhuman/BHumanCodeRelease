/**
 * @file GlobalTeammatesTracker.cpp
 *
 * Implementation of a module the aims to track all teammates on the pitch
 * by fusing local estimates with information sent by teammates and by the GameController.
 *
 * @author Tim Laue
 */

#include "GlobalTeammatesTracker.h"
#include "Debugging/DebugDrawings.h"

MAKE_MODULE(GlobalTeammatesTracker);


GlobalTeammatesTracker::GlobalTeammatesTracker()
{
  // Set the zones in which robots are situated during a penalty
  // and when returning from a penalty.
  // The exact positions depend on the referees, thus the hardcoded values
  // here are rough guesses that should cover most situations.
  const Geometry::Rect penaltyPlacementLeftOwn  (Vector2f(-1000.f, theFieldDimensions.yPosLeftTouchline + 200.f),
                                              Vector2f(theFieldDimensions.xPosOwnFieldBorder, theFieldDimensions.yPosLeftFieldBorder + 100.f));
  const Geometry::Rect penaltyPlacementRightOwn (Vector2f(-1000.f, theFieldDimensions.yPosRightTouchline - 200.f),
                                              Vector2f(theFieldDimensions.xPosOwnFieldBorder, theFieldDimensions.yPosRightFieldBorder - 100.f));
  const Geometry::Rect returnFromPenaltyLeftOwn (Vector2f(theFieldDimensions.xPosReturnFromPenalty - 700.f, theFieldDimensions.yPosLeftReturnFromPenalty - 400.f),
                                              Vector2f(theFieldDimensions.xPosReturnFromPenalty + 700.f, theFieldDimensions.yPosLeftReturnFromPenalty + 200.f));
  const Geometry::Rect returnFromPenaltyRightOwn(Vector2f(theFieldDimensions.xPosReturnFromPenalty - 700.f, theFieldDimensions.yPosRightReturnFromPenalty - 200.f),
                                              Vector2f(theFieldDimensions.xPosReturnFromPenalty + 700.f, theFieldDimensions.yPosRightReturnFromPenalty + 400.f));
  penalizedRobotZonesOwnTeam.push_back(penaltyPlacementLeftOwn);
  penalizedRobotZonesOwnTeam.push_back(penaltyPlacementRightOwn);
  returnFromPenaltyZonesOwnTeam.push_back(returnFromPenaltyLeftOwn);
  returnFromPenaltyZonesOwnTeam.push_back(returnFromPenaltyRightOwn);
  // List of teammates, set size+1 to leave out element 0 and
  // have the robots inside the list at indices equaling their player numbers
  ownTeam.resize(MAX_NUM_PLAYERS+1);

  // Read self locator parameters
  InMapFile stream("selfLocator.cfg");
  if(stream.exists())
    stream >> selfLocatorParameters;
}

void GlobalTeammatesTracker::update(GlobalTeammatesModel& globalTeammatesModel)
{
  updateGameAndTeammateInfo();
  integratePerceivedTeammates();
  fillModel(globalTeammatesModel);
  draw();
}

void GlobalTeammatesTracker::fillModel(GlobalTeammatesModel& globalTeammatesModel)
{
  globalTeammatesModel.teammates.clear();
  for(std::size_t i=1; i < ownTeam.size(); i++)
  {
    const TeammateInformation& tmi = ownTeam.at(i);
    if(tmi.state == playing && static_cast<int>(i) != theGameState.playerNumber)
    {
      GlobalTeammatesModel::TeammateEstimate teammate;
      teammate.pose.translation = tmi.estimatedPosition;
      teammate.pose.rotation = tmi.lastCommunicatedRotation;
      teammate.playerNumber = static_cast<int>(i);
      teammate.relativeWalkTarget = teammate.pose.inverse() * tmi.walkTargetOnField;
      teammate.speed = tmi.speed;
      globalTeammatesModel.teammates.push_back(teammate);
    }
  }
}

void GlobalTeammatesTracker::updateGameAndTeammateInfo()
{
  numberOfPenalizedTeammates = 0;
  numberOfTeammatesReturningFromPenalty = 0;

  // Myself:
  TeammateInformation& aboutMe = ownTeam.at(theGameState.playerNumber);
  aboutMe.lastCommunicatedRotation = theRobotPose.rotation;
  aboutMe.covariance = theRobotPose.covariance.topLeftCorner<2, 2>();
  aboutMe.estimatedPosition = theRobotPose.translation;
  aboutMe.extrapolatedPositionLastFrame = theRobotPose.translation; // Does not matter anyway
  aboutMe.perceivedInThisFrame = false; // Does not matter anyway
  aboutMe.lastCommunicationUpdate = theFrameInfo.time;
  aboutMe.state = theGameState.isPenalized() ? penalized : playing; // I cannot be invalid and returning from penalty can be skipped
  aboutMe.speed = 0.f; // Does not matter anyway
  aboutMe.walkTargetOnField = Vector2f::Zero(); // Does not matter anyway

  // Update information about teammates in internal list (first element is a dummy):
  for(int playerNumber = 1; static_cast<std::size_t>(playerNumber) < ownTeam.size(); playerNumber++)
  {
    // Skip myself:
    if(playerNumber == theGameState.playerNumber)
      continue;
    TeammateInformation& tmi = ownTeam.at(playerNumber);
    tmi.perceivedInThisFrame = false; // Reset at beginning of each frame
    // Check, if GameController marks this robot as penalized
    bool isPenalized = GameState::isPenalized(theGameState.ownTeam.playerStates[playerNumber - Settings::lowestValidPlayerNumber]);
    if(isPenalized)
    {
      // Set state of teammate to penalized, if teammate was on the pitch.
      // In case the teammate was still invalid or penalized, nothing happens here.
      if(tmi.state == playing || tmi.state == waitingAfterUnpenalize)
        tmi.state = penalized;
      numberOfPenalizedTeammates++;
      // Go to next robot, as nothing else is of any relevance for this robot:
      continue;
    }
    // If robot is not penalized, we try to update the internal information.
    // First check, if there is any communicated information about this teammate:
    const Teammate* tm = findTeammateByNumber(playerNumber);
    if(tm == nullptr)
    {
      // OK, no data available. But as we know that the robot is not penalized,
      // we might want to change its state.
      if(tmi.state == penalized)
        tmi.state = waitingAfterUnpenalize;
      numberOfTeammatesReturningFromPenalty++;
      continue;
    }
    // We have some data and robot seems to be active now: Initialize or update internal data.
    if(tmi.state == playing)
      updateTeammateInformation(tmi, *tm);
    else
      initializeTeammateInformation(tmi, *tm);
  }
}

void GlobalTeammatesTracker::initializeTeammateInformation(TeammateInformation& tmi, const Teammate& teammateData)
{
  tmi.lastCommunicatedRotation = teammateData.theRobotPose.rotation;
  tmi.covariance = teammateData.theRobotPose.covariance.topLeftCorner<2, 2>();
  tmi.estimatedPosition = teammateData.getEstimatedPosition(theFrameInfo.time);
  tmi.extrapolatedPositionLastFrame = tmi.estimatedPosition;
  tmi.lastCommunicationUpdate = teammateData.theFrameInfo.time;
  tmi.state = playing;
  tmi.walkTargetOnField = teammateData.theRobotPose * teammateData.theBehaviorStatus.walkingTo;
  tmi.speed = teammateData.theBehaviorStatus.speed;
  tmi.isGoalkeeper = teammateData.isGoalkeeper;
}

void GlobalTeammatesTracker::updateTeammateInformation(TeammateInformation& tmi, const Teammate& teammateData)
{
  // Is there some new information available?
  if(teammateData.theFrameInfo.time > tmi.lastCommunicationUpdate)
  {
    // Does the new position differ a lot from the estimated position?
    const float dist = (teammateData.theRobotPose.translation - tmi.estimatedPosition).norm();
    // If yes, we can reset the whole internal data:
    if(dist >= reinitThreshold)
    {
      initializeTeammateInformation(tmi, teammateData);
    }
    // If not, we treat the new information as a measurement:
    else
    {
      tmi.measurementUpdate(teammateData.theRobotPose.translation, teammateData.theRobotPose.covariance.topLeftCorner<2, 2>());
      tmi.extrapolatedPositionLastFrame = teammateData.getEstimatedPosition(theFrameInfo.time);
      tmi.lastCommunicatedRotation = teammateData.theRobotPose.rotation;
      tmi.lastCommunicationUpdate = teammateData.theFrameInfo.time;
      tmi.walkTargetOnField = teammateData.theRobotPose * teammateData.theBehaviorStatus.walkingTo;
      tmi.speed = teammateData.theBehaviorStatus.speed;
    }
  }
  // If there is no new information, we propagate the position in the direction that is indicated
  // by the communicated target of the robot. Traveled distance is used to increase the covariance
  // of the position estimate:
  else
  {
    const Vector2f currentPos(teammateData.getEstimatedPosition(theFrameInfo.time));
    const Vector2f delta(currentPos - tmi.extrapolatedPositionLastFrame);
    tmi.extrapolatedPositionLastFrame = currentPos;
    tmi.estimatedPosition += delta;
    Matrix2f& cov = tmi.covariance;
    // add process noise:
    cov(0, 0) += sqr(selfLocatorParameters.filterProcessDeviation.translation.x());
    cov(1, 1) += sqr(selfLocatorParameters.filterProcessDeviation.translation.y());
    // increase covariance based on traveled distance
    float dist = delta.norm();
    float angle = delta.angle();
    float xFactor = 4.f; // TODO: Make proper parameters
    float yFactor = 1.f;
    Vector2f dVec(sqr(xFactor * dist), sqr(yFactor * dist));
    Matrix2f c2;
    c2(0,0) = dVec.x();;
    c2(0,1) = c2(1,0) = 0.f;
    c2(1,1) = dVec.y();
    c2 = Covariance::rotateCovarianceMatrix(c2,angle);
    cov += c2;
  }
}

const Teammate* GlobalTeammatesTracker::findTeammateByNumber(int playerNumber) const
{
  for(auto const& teammate : theTeamData.teammates)
    if(teammate.number == playerNumber)
      return &teammate;
  return nullptr;
}

void GlobalTeammatesTracker::integratePerceivedTeammates()
{
  teammatePercepts.clear();
  for(const auto& teammateObservation : theObstaclesFieldPercept.obstacles)
  {
    // Only consider teammates, ignore opponents and unknown objects:
    if(teammateObservation.type == ObstaclesFieldPercept::ownPlayer || teammateObservation.type == ObstaclesFieldPercept::ownGoalkeeper)
    {
      const Vector2f poseOnField = theRobotPose * teammateObservation.center;
      if(teammatePerceptCanBeUsed(poseOnField))
      {
        TeammatePercept tp;
        tp.pos = poseOnField;
        tp.cov = Covariance::rotateCovarianceMatrix(teammateObservation.covariance, theRobotPose.rotation);
        tp.isGoalkeeper = teammateObservation.type == ObstaclesFieldPercept::ownGoalkeeper;
        teammatePercepts.push_back(tp);
      }
    }
  }

  // TODO: Refine this approach, current assignment is greedy and probably suboptimal in some cases!
  // Loop over all percepts to assign them to robots.
  for(const TeammatePercept& tp : teammatePercepts)
  {
    float smallestDistance = squaredDistanceThreshold;
    std::size_t bestIdx = 0;
    // Find the best fitting, playing robot that is not me and that was not assigned a percept in this frame:
    for(std::size_t i=1; i < ownTeam.size(); i++)
    {
      const TeammateInformation& tmi = ownTeam.at(i);
      if(tmi.state == playing && !tmi.perceivedInThisFrame && static_cast<int>(i) != theGameState.playerNumber &&
         tmi.isGoalkeeper == tp.isGoalkeeper)
      {
        float dist = distanceTeammateAndPerception(tmi, tp);
        if(dist < smallestDistance)
        {
          smallestDistance = dist;
          bestIdx = i;
        }
      }
    }
    if(bestIdx != 0)
    {
      TeammateInformation& tmi = ownTeam.at(bestIdx);
      tmi.measurementUpdate(tp.pos, tp.cov);
      tmi.perceivedInThisFrame = true;
    }
  }
}

bool GlobalTeammatesTracker::teammatePerceptCanBeUsed(const Vector2f& p)
{
  // Check, if observation is within the reasonable area of the pitch:
  const FieldDimensions& f = theFieldDimensions;
  if(p.x() > f.xPosOpponentFieldBorder - borderThreshold || p.x() < f.xPosOwnFieldBorder + borderThreshold ||
     p.y() > f.yPosLeftFieldBorder - borderThreshold || p.y() < f.yPosRightFieldBorder + borderThreshold)
    return false;
  // Check, if observation is within an area in which penalized/returning opponents are usually placed:
  if(numberOfPenalizedTeammates)
    for(const auto& zone : penalizedRobotZonesOwnTeam)
      if(Geometry::isPointInsideRectangle(zone, p))
        return false;
  if(numberOfTeammatesReturningFromPenalty)
    for(const auto& zone : returnFromPenaltyZonesOwnTeam)
      if(Geometry::isPointInsideRectangle(zone, p))
        return false;
  return true;
}

float GlobalTeammatesTracker::distanceTeammateAndPerception(const TeammateInformation& tmi, const TeammatePercept& tp)
{
  const Eigen::Matrix<float, 2, 1> meanDiff = tmi.estimatedPosition - tp.pos;
  // Check Euclidean distance first:
  if(meanDiff.squaredNorm() >= squaredDistanceThreshold)
    return squaredDistanceThreshold;
  // If both robots are not too far away from each other, return squared Mahalanobis distance:
  const Eigen::Matrix<float, 2, 2> combinedCovs = (tmi.covariance + tp.cov) * .5f;
  return meanDiff.transpose() * combinedCovs.inverse() * meanDiff;;
}

void GlobalTeammatesTracker::draw()
{
  // The zones that will get a special handling:
  DECLARE_DEBUG_DRAWING("module:GlobalTeammatesTracker:penaltyZones", "drawingOnField");
  COMPLEX_DRAWING("module:GlobalTeammatesTracker:penaltyZones")
  {
    for(const auto& rect : returnFromPenaltyZonesOwnTeam)
    {
      RECTANGLE("module:GlobalTeammatesTracker:penaltyZones", rect.a.x(), rect.a.y(), rect.b.x(), rect.b.y(), 10, Drawings::solidPen, ColorRGBA(200,200,200));
    }
    for(const auto& rect : penalizedRobotZonesOwnTeam)
    {
      RECTANGLE("module:GlobalTeammatesTracker:penaltyZones", rect.a.x(), rect.a.y(), rect.b.x(), rect.b.y(), 10, Drawings::solidPen, ColorRGBA(200,200,200));
    }
  }
  // The filtered list of observations that will be used for the clustering:
  DECLARE_DEBUG_DRAWING("module:GlobalTeammatesTracker:teammatesDetails", "drawingOnField");
  COMPLEX_DRAWING("module:GlobalTeammatesTracker:teammatesDetails")
  {
    for(std::size_t i=1; i < ownTeam.size(); i++)
    {
      const TeammateInformation& tmi = ownTeam.at(i);
      if(tmi.state == playing && static_cast<int>(i) != theGameState.playerNumber)
      {
        COVARIANCE_ELLIPSES_2D("module:GlobalTeammatesTracker:teammatesDetails", tmi.covariance, tmi.estimatedPosition);
      }
    }
  }
  // The perceived teammates:
  DECLARE_DEBUG_DRAWING("module:GlobalTeammatesTracker:perceptions", "drawingOnField");
  COMPLEX_DRAWING("module:GlobalTeammatesTracker:perceptions")
  {
    for(const auto& tp : teammatePercepts)
    {
      COVARIANCE_ELLIPSES_2D("module:GlobalTeammatesTracker:perceptions", tp.cov, tp.pos);
    }
  }
}
