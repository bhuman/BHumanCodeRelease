/**
 * @file GlobalRobotsTracker.cpp
 *
 * Implementation of a module the aims to track all robots on the pitch
 * by fusing local estimates with information sent by teammates and by the GameController.
 *
 * @author Tim Laue
 */

// TODO:
//  Major steps:
//    -> Determine number of opponents in play
//    -> Pool all valid obervations
//    -> Map observations to robots
//  Things to do:
// - Implement the clustering
//    - Add flag to own observations?
// - Areas in different colors (light gray [inactive]; dark gray [active]; opponent color [detection)]
// - Use the "Return from penalty" zones
// - Handling of penalties:
// -   -> Shootout
// -   -> During the game

#include "GlobalRobotsTracker.h"
#include "Debugging/DebugDrawings.h"
#include "Tools/Modeling/Measurements.h"


MAKE_MODULE(GlobalRobotsTracker, modeling);

void GlobalRobotsTracker::ObservedSetOfOpponents::draw([[maybe_unused]] const ColorRGBA& color) const
{
  for(const auto& obs : observations)
  {
    CIRCLE("module:GlobalRobotsTracker:internalOpponents", obs.position.x(), obs.position.y(), 500, 20, Drawings::solidPen, color, Drawings::noBrush, color);
  }
}

GlobalRobotsTracker::GlobalRobotsTracker()
{
  // Set the zones in which robots are situated during a penalty
  // and when returning from a penalty.
  // The exact positions depend on the referees, thus the hardcoded values
  // here are rough guesses that should cover most situations.
  const Geometry::Rect penaltyPlacementLeftOpp  (Vector2f(1000.f, theFieldDimensions.yPosLeftSideline + 200.f),
                                              Vector2f(theFieldDimensions.xPosOpponentFieldBorder, theFieldDimensions.yPosLeftFieldBorder + 100.f));
  const Geometry::Rect penaltyPlacementRightOpp (Vector2f(1000.f, theFieldDimensions.yPosRightSideline - 200.f),
                                              Vector2f(theFieldDimensions.xPosOpponentFieldBorder, theFieldDimensions.yPosRightFieldBorder - 100.f));
  const Geometry::Rect returnFromPenaltyLeftOpp (Vector2f(theFieldDimensions.xPosOpponentPenaltyMark - 700.f, theFieldDimensions.yPosLeftSideline-200),
                                              Vector2f(theFieldDimensions.xPosOpponentPenaltyMark + 700.f, theFieldDimensions.yPosLeftSideline + 400.f));
  const Geometry::Rect returnFromPenaltyRightOpp(Vector2f(theFieldDimensions.xPosOpponentPenaltyMark - 700.f, theFieldDimensions.yPosRightSideline - 400.f),
                                              Vector2f(theFieldDimensions.xPosOpponentPenaltyMark + 700.f, theFieldDimensions.yPosRightSideline+200));
  penalizedRobotZonesOpponentTeam.push_back(penaltyPlacementLeftOpp);
  penalizedRobotZonesOpponentTeam.push_back(penaltyPlacementRightOpp);
  returnFromPenaltyZonesOpponentTeam.push_back(returnFromPenaltyLeftOpp);
  returnFromPenaltyZonesOpponentTeam.push_back(returnFromPenaltyRightOpp);
  const Geometry::Rect penaltyPlacementLeftOwn  (Vector2f(-1000.f, theFieldDimensions.yPosLeftSideline + 200.f),
                                              Vector2f(theFieldDimensions.xPosOwnFieldBorder, theFieldDimensions.yPosLeftFieldBorder + 100.f));
  const Geometry::Rect penaltyPlacementRightOwn (Vector2f(-1000.f, theFieldDimensions.yPosRightSideline - 200.f),
                                              Vector2f(theFieldDimensions.xPosOwnFieldBorder, theFieldDimensions.yPosRightFieldBorder - 100.f));
  const Geometry::Rect returnFromPenaltyLeftOwn (Vector2f(theFieldDimensions.xPosOwnPenaltyMark - 700.f, theFieldDimensions.yPosLeftSideline - 200),
                                              Vector2f(theFieldDimensions.xPosOwnPenaltyMark + 700.f, theFieldDimensions.yPosLeftSideline + 400.f));
  const Geometry::Rect returnFromPenaltyRightOwn(Vector2f(theFieldDimensions.xPosOwnPenaltyMark - 700.f, theFieldDimensions.yPosRightSideline - 400.f),
                                              Vector2f(theFieldDimensions.xPosOwnPenaltyMark + 700.f, theFieldDimensions.yPosRightSideline+200));
  penalizedRobotZonesOpponentTeam.push_back(penaltyPlacementLeftOwn);
  penalizedRobotZonesOpponentTeam.push_back(penaltyPlacementRightOwn);
  returnFromPenaltyZonesOpponentTeam.push_back(returnFromPenaltyLeftOwn);
  returnFromPenaltyZonesOpponentTeam.push_back(returnFromPenaltyRightOwn);
  // Initialize list for teammates:
  opponentsObservedByTeammates.resize(MAX_NUM_PLAYERS);
  // List of teammates, set size+1 to leave out element 0 and
  // have the robots inside the list at indices equaling their player numbers
  ownTeam.resize(MAX_NUM_PLAYERS+1);

  // Read self locator parameters
  InMapFile stream("selfLocator.cfg");
  if(stream.exists())
    stream >> selfLocatorParameters;
}

void GlobalRobotsTracker::update(GlobalOpponentsModel& globalOpponentsModel)
{
  fillModel(globalOpponentsModel);
}

void GlobalRobotsTracker::update(GlobalTeammatesModel& globalTeammatesModel)
{
  updateGameAndTeammateInfo();
  integratePerceivedTeammates();

  // TODO: Implement and activate these:
  //updateLocalOpponentObservations();
  //updateTeamOpponentObservations();
  //computePoolOfObservations();
  //computeInternalModelOfOpponents();

  fillModel(globalTeammatesModel);
  draw();
}

void GlobalRobotsTracker::fillModel(GlobalTeammatesModel& globalTeammatesModel)
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
      teammate.relativeWalkTarget = Transformation::fieldToRobot(teammate.pose, tmi.walkTargetOnField);
      teammate.speed = tmi.speed;
      globalTeammatesModel.teammates.push_back(teammate);
    }
  }
}

void GlobalRobotsTracker::fillModel(GlobalOpponentsModel& globalOpponentsModel)
{
  globalOpponentsModel.opponents.clear();
  // TODO -> Use transform internal representation to model content.
  globalOpponentsModel.numOfUnknownOpponents = numberOfUnpenalizedOpponents;
  globalOpponentsModel.numOfPenalizedOpponents = numberOfPenalizedOpponents;
}

void GlobalRobotsTracker::updateGameAndTeammateInfo()
{
  // Opponents: **********
  int totalNumberOfOpponents = 0;
  numberOfUnpenalizedOpponents = 0;
  // Loop through list of opponents and count all those that are not substitutes
  // as well those that are currently supposed to play:
  for(int i=0; i < MAX_NUM_PLAYERS; i++)
  {
    if(theGameState.opponentTeam.playerStates[i] != GameState::substitute)
    {
      totalNumberOfOpponents++;
      if(!GameState::isPenalized(theGameState.opponentTeam.playerStates[i]) ||                  // Playing
         theGameState.opponentTeam.playerStates[i] == GameState::penalizedIllegalMotionInSet)   // Not playing but still on pitch
        numberOfUnpenalizedOpponents++;
    }
  }
  numberOfPenalizedOpponents = totalNumberOfOpponents - numberOfUnpenalizedOpponents;

  // Own team: **********
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
    // First check, if there is any communicated infomation about this teammate:
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

void GlobalRobotsTracker::initializeTeammateInformation(TeammateInformation& tmi, const Teammate& teammateData)
{
  tmi.lastCommunicatedRotation = teammateData.theRobotPose.rotation;
  tmi.covariance = teammateData.theRobotPose.covariance.topLeftCorner<2, 2>();
  tmi.estimatedPosition = teammateData.getEstimatedPosition(theFrameInfo.time);
  tmi.extrapolatedPositionLastFrame = tmi.estimatedPosition;
  tmi.lastCommunicationUpdate = teammateData.theFrameInfo.time;
  tmi.state = playing;
  tmi.walkTargetOnField = teammateData.theRobotPose * teammateData.theBehaviorStatus.walkingTo;
  tmi.speed = teammateData.theBehaviorStatus.speed;
}

void GlobalRobotsTracker::updateTeammateInformation(TeammateInformation& tmi, const Teammate& teammateData)
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
   // cov(0, 0) += sqr(delta.x() * selfLocatorParameters.odometryDeviation.translation.x());
   // cov(1, 1) += sqr(delta.y() * selfLocatorParameters.odometryDeviation.translation.y());

    // increase covariance based on traveled distance (version 2)
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

const Teammate* GlobalRobotsTracker::findTeammateByNumber(int playerNumber) const
{
  for(auto const& teammate : theTeamData.teammates)
    if(teammate.number == playerNumber)
      return &teammate;
  return nullptr;
}

void GlobalRobotsTracker::updateLocalOpponentObservations()
{
  // Do not update local information (TODO: Check more conditions)
  if(theFallDownState.state != FallDownState::upright ||
     theGameState.isPenalized())
    return;
  // Always use the most recent local obstacle model:
  obstacleModelToSetofOpponents(theObstacleModel.obstacles, theRobotPose, opponentsObservedByMe);
}

void GlobalRobotsTracker::updateTeamOpponentObservations()
{
  for(auto const& teammate : theTeamData.teammates)
  {
    if(teammate.isUpright &&
       theFrameInfo.getTimeSince(teammate.theFrameInfo.time) < maximumAgeOfObservationSet) // TODO: Check for more constraints
    {
      obstacleModelToSetofOpponents(teammate.theObstacleModel.obstacles,
                                    teammate.theRobotPose,
                                    opponentsObservedByTeammates[teammate.number - 1]);
    }
  }
}

void GlobalRobotsTracker::computeInternalModelOfOpponents()
{
  // Solve the chaos!
  // Notes: Consider distances + ages; require observations by multiple robots in some cases
}

void GlobalRobotsTracker::obstacleModelToSetofOpponents(const std::vector<Obstacle>& obstacles,
                                                           const Pose2f& robotPose,
                                                           ObservedSetOfOpponents& setOfOpponents)
{
  setOfOpponents.observations.clear();
  for(const auto& obstacle : obstacles)
  {
    if(obstacle.isOpponent()) // TODO: Consider unknown robots
    {
      OpponentObservation obs;
      obs.position = robotPose * obstacle.center;
      obs.observerPose = robotPose;
      obs.distanceOfObserver = obstacle.center.norm();
      obs.lastSeen = obstacle.lastSeen;
      setOfOpponents.observations.push_back(obs);
      if(obstacle.lastSeen > setOfOpponents.lastUpdate)
        setOfOpponents.lastUpdate = obstacle.lastSeen;
    }
  }
}

void GlobalRobotsTracker::computePoolOfObservations()
{
  pooledObservations.clear();
  if(theFrameInfo.getTimeSince(opponentsObservedByMe.lastUpdate) < maximumAgeOfObservationSet)
  {
    for(const auto& obs : opponentsObservedByMe.observations)
    {
      if(observationCanBeUsed(obs))
        pooledObservations.push_back(obs);
    }
  }
  for(const auto& teammate : opponentsObservedByTeammates)
  {
    if(theFrameInfo.getTimeSince(teammate.lastUpdate) < maximumAgeOfObservationSet)
    {
      for(const auto& obs : teammate.observations)
      {
        if(observationCanBeUsed(obs))
          pooledObservations.push_back(obs);
      }
    }
  }
}

void GlobalRobotsTracker::integratePerceivedTeammates()
{
  teammatePercepts.clear();
  const Vector2f& rotationDeviation = theMotionInfo.executedPhase == MotionPhase::stand ? selfLocatorParameters.robotRotationDeviationInStand : selfLocatorParameters.robotRotationDeviation;
  for(const auto& teammateObservation : theObstaclesFieldPercept.obstacles)
  {
    // Only consider teammates, ignore opponents and unknown objects:
    if(teammateObservation.type == ObstaclesFieldPercept::ownPlayer)
    {
      const Vector2f poseOnField = theRobotPose * teammateObservation.center;
      if(teammatePerceptCanBeUsed(poseOnField))
      {
        TeammatePercept tp;
        tp.pos = poseOnField;
        const Matrix2f& covRelative = Measurements::positionToCovarianceMatrixInRobotCoordinates(teammateObservation.center, 0.f, theCameraMatrix, theCameraMatrix.inverse(), rotationDeviation);
        tp.cov = Covariance::rotateCovarianceMatrix(covRelative, theRobotPose.rotation);
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
      if(tmi.state == playing && !tmi.perceivedInThisFrame && static_cast<int>(i) != theGameState.playerNumber)
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

bool GlobalRobotsTracker::teammatePerceptCanBeUsed(const Vector2f& p)
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

float GlobalRobotsTracker::distanceTeammateAndPerception(const TeammateInformation& tmi, const TeammatePercept& tp)
{
  const Eigen::Matrix<float, 2, 1> meanDiff = tmi.estimatedPosition - tp.pos;
  // Check Euclidian distance first:
  if(meanDiff.squaredNorm() >= squaredDistanceThreshold)
    return squaredDistanceThreshold;
  // If both robots are not too far away from each other, return squared Mahalanobis distance:
  const Eigen::Matrix<float, 2, 2> combinedCovs = (tmi.covariance + tp.cov) * .5f;
  return meanDiff.transpose() * combinedCovs.inverse() * meanDiff;;
}

bool GlobalRobotsTracker::observationCanBeUsed(const OpponentObservation& observation)
{
  /*
  const Vector2f& p = observation.position;
  const FieldDimensions& f = theFieldDimensions;
  // Check, if observation is within the reasonable area of the pitch:
  if(p.x() > f.xPosOpponentFieldBorder - borderThreshold || p.x() < f.xPosOwnFieldBorder + borderThreshold ||
     p.y() > f.yPosLeftFieldBorder - borderThreshold || p.y() < f.yPosRightFieldBorder + borderThreshold)
    return false;
  // Check, if observation is within an area in which penalized opponents are usually placed:
  if(numberOfPenalizedOpponents)
    for(const auto& zone : penalizedRobotZones)
      if(Geometry::isPointInsideRectangle(zone, p))
        return false;
  // Check for confusion with teammates:
  float sqrThres = teammateExclusionDistThreshold * teammateExclusionDistThreshold;
  for(const auto& tm : ownTeamPoses)
    if((p - tm.translation).squaredNorm() < sqrThres)
      return false;
  // Observation seems to be OK:

  return true;*/

  // TODO: Rethink all this stuff. This is stupid dummy code for avoiding an "unused parameter warning"
  if(observation.lastSeen == 0)
    return false;
  else
    return false;
}

void GlobalRobotsTracker::draw()
{
  // The zones that will get a special handling:
  DECLARE_DEBUG_DRAWING("module:GlobalRobotsTracker:penaltyZones", "drawingOnField");
  COMPLEX_DRAWING("module:GlobalRobotsTracker:penaltyZones")
  {
    for(const auto& rect : returnFromPenaltyZonesOpponentTeam)
    {
      RECTANGLE("module:GlobalRobotsTracker:penaltyZones", rect.a.x(), rect.a.y(), rect.b.x(), rect.b.y(), 10, Drawings::solidPen, ColorRGBA(200,200,200));
    }
    for(const auto& rect : penalizedRobotZonesOpponentTeam)
    {
      RECTANGLE("module:GlobalRobotsTracker:penaltyZones", rect.a.x(), rect.a.y(), rect.b.x(), rect.b.y(), 10, Drawings::solidPen, ColorRGBA(200,200,200));
    }
    for(const auto& rect : returnFromPenaltyZonesOwnTeam)
    {
      RECTANGLE("module:GlobalRobotsTracker:penaltyZones", rect.a.x(), rect.a.y(), rect.b.x(), rect.b.y(), 10, Drawings::solidPen, ColorRGBA(200,200,200));
    }
    for(const auto& rect : penalizedRobotZonesOwnTeam)
    {
      RECTANGLE("module:GlobalRobotsTracker:penaltyZones", rect.a.x(), rect.a.y(), rect.b.x(), rect.b.y(), 10, Drawings::solidPen, ColorRGBA(200,200,200));
    }
  }
  // The internal representations of the tracked opponents:
  DECLARE_DEBUG_DRAWING("module:GlobalRobotsTracker:internalOpponents", "drawingOnField");
  COMPLEX_DRAWING("module:GlobalRobotsTracker:internalOpponents")
  {
    if(theFrameInfo.getTimeSince(opponentsObservedByMe.lastUpdate) < maximumAgeOfObservationSet)
    {
      ColorRGBA col(50,50,50);
      opponentsObservedByMe.draw(col);
    }
    ColorRGBA col2(100,100,100);
    for(const auto& teammateObservations : opponentsObservedByTeammates)
    {
      if(theFrameInfo.getTimeSince(teammateObservations.lastUpdate) < maximumAgeOfObservationSet)
        teammateObservations.draw(col2);
    }
  }
  // The filtered list of observations that will be used for the clustering:
  DECLARE_DEBUG_DRAWING("module:GlobalRobotsTracker:pooledOpponents", "drawingOnField");
  COMPLEX_DRAWING("module:GlobalRobotsTracker:pooledOpponents")
  {
    ColorRGBA col3(255,0,0);
    for(const auto& poolObs : pooledObservations)
    {
      CIRCLE("module:GlobalRobotsTracker:pooledOpponents", poolObs.position.x(), poolObs.position.y(), 500, 20, Drawings::solidPen, col3, Drawings::noBrush, col3);
    }
  }
  // The filtered list of observations that will be used for the clustering:
  DECLARE_DEBUG_DRAWING("module:GlobalRobotsTracker:teammatesDetails", "drawingOnField");
  COMPLEX_DRAWING("module:GlobalRobotsTracker:teammatesDetails")
  {
    for(std::size_t i=1; i < ownTeam.size(); i++)
    {
      const TeammateInformation& tmi = ownTeam.at(i);
      if(tmi.state == playing && static_cast<int>(i) != theGameState.playerNumber)
      {
        COVARIANCE_ELLIPSES_2D("module:GlobalRobotsTracker:teammatesDetails", tmi.covariance, tmi.estimatedPosition);
      }
    }
  }
  // The perceived teammates:
  DECLARE_DEBUG_DRAWING("module:GlobalRobotsTracker:perceptions", "drawingOnField");
  COMPLEX_DRAWING("module:GlobalRobotsTracker:perceptions")
  {
    for(const auto& tp : teammatePercepts)
    {
      COVARIANCE_ELLIPSES_2D("module:GlobalRobotsTracker:perceptions", tp.cov, tp.pos);
    }
  }
}
