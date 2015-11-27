/**
* @file TemplateGenerator.cpp
*
* This file implements a submodule that generates robot positions from percepts.
*
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#include "TemplateGenerator.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Probabilistics.h"
#include "Tools/Math/Random.h"
#include "Tools/Modeling/PoseComputation.h"

TemplateGenerator::TemplateGenerator(const SelfLocatorBase::Parameters& parameters,
                                     const GoalPercept& goalPercept, const LinePercept& linePercept,
                                     const FrameInfo& frameInfo,
                                     const FieldDimensions& fieldDimensions,
                                     const OdometryData& odometryData,
                                     const MotionRequest& motionRequest,
                                     const RobotInfo& robotInfo,
                                     const VerifiedCenterCircle& verifiedCenterCircle,
                                     const VerifiedPenaltyMark& verifiedPenaltyMark) :
  parameters(parameters),
  theGoalPercept(goalPercept),
  theLinePercept(linePercept),
  theFrameInfo(frameInfo),
  theFieldDimensions(fieldDimensions),
  theOdometryData(odometryData),
  theMotionRequest(motionRequest),
  theRobotInfo(robotInfo),
  verifiedCenterCircle(verifiedCenterCircle),
  verifiedPenaltyMark(verifiedPenaltyMark),
  nextWalkInTemplateNumber(0)
{}

void TemplateGenerator::init()
{
  realPostPositions[GoalPost::IS_LEFT] =
    Vector2f(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosLeftGoal);
  realPostPositions[GoalPost::IS_RIGHT] =
    Vector2f(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosRightGoal);
  fullGoals.clear();
  knownGoalposts.clear();
  unknownGoalposts.clear();
  //              //
  //  2        5  //
  //  4        3  //
  //  1           //
  //     Goal     //
  walkInPositions.clear();
  Vector2f pos1(-3000.f, theFieldDimensions.yPosLeftSideline);
  Vector2f pos2(-1000.f, theFieldDimensions.yPosLeftSideline);
  Vector2f pos3(-2000.f, theFieldDimensions.yPosRightSideline);
  Vector2f pos4(-2000.f, theFieldDimensions.yPosLeftSideline);
  Vector2f pos5(-1000.f, theFieldDimensions.yPosRightSideline);
  Vector2f centerOfGoal(theFieldDimensions.xPosOwnGroundline, 0.f);
  Vector2f pos1ToGoal = centerOfGoal - pos1;
  Vector2f pos2ToGoal = centerOfGoal - pos2;
  Vector2f pos3ToGoal = centerOfGoal - pos3;
  Vector2f pos4ToGoal = centerOfGoal - pos4;
  Vector2f pos5ToGoal = centerOfGoal - pos5;
  float angle1 = std::atan2(pos1ToGoal.y(), pos1ToGoal.x());
  float angle2 = std::atan2(pos2ToGoal.y(), pos2ToGoal.x());
  float angle3 = std::atan2(pos3ToGoal.y(), pos3ToGoal.x());
  float angle4 = std::atan2(pos4ToGoal.y(), pos4ToGoal.x());
  float angle5 = std::atan2(pos5ToGoal.y(), pos5ToGoal.x());
  walkInPositions.push_back(Pose2f(angle1, pos1)); // Robot 1
  walkInPositions.push_back(Pose2f(angle2, pos2)); // Robot 2
  walkInPositions.push_back(Pose2f(angle3, pos3)); // Robot 3
  walkInPositions.push_back(Pose2f(angle4, pos4)); // Robot 4
  walkInPositions.push_back(Pose2f(angle5, pos5)); // Robot 5
}

Pose2f TemplateGenerator::getTemplate(TemplateGenerator::ForceHalf forceHalf, const Pose2f& robotPose, const Pose2f& lastRobotPose) const
{
  if(forceHalf == TemplateGenerator::CONSIDER_POSE)
  {
    TemplateGenerator::SampleTemplate t = generateTemplate(lastRobotPose);
    if(isMirrorCloser(t, lastRobotPose))
      return Pose2f(pi) + t;
    else
      return t;
  }
  else
  {
    TemplateGenerator::SampleTemplate t = generateTemplate(lastRobotPose);
    if(forceHalf == TemplateGenerator::OWN_HALF)
    {
      if(t.translation.x() <= 0.f)
        return t;
      else
        return Pose2f(pi) + t;
    }
    else if(forceHalf == TemplateGenerator::OPPONENT_HALF)
    {
      if(t.translation.x() > 0.f)
        return t;
      else
        return Pose2f(pi) + t;
    }
    else // forceHalf == RANDOM_HALF
    {
      if(t.origin == TemplateGenerator::SampleTemplate::RANDOM)
        return t;
      else
      {
        if(randomFloat() < 0.5)
          return t;
        else
          return Pose2f(pi) + t;
      }
    }
  }
}

void TemplateGenerator::bufferNewPerceptions(const Pose2f& robotPose)
{
  if(!theGoalPercept.goalPosts.empty() &&
     (theMotionRequest.motion == MotionRequest::walk || theMotionRequest.motion == MotionRequest::stand ||
      (theMotionRequest.motion == MotionRequest::specialAction && theMotionRequest.specialActionRequest.specialAction == SpecialActionRequest::standHigh)))
  {
    // If there is any invalid goal post, discard them all:
    bool goalieInPenaltyArea = Global::getSettings().isGoalkeeper &&
                               robotPose.translation.x() < theFieldDimensions.xPosOwnPenaltyArea &&
                               std::abs(robotPose.translation.y()) < theFieldDimensions.yPosLeftPenaltyArea;
    for(auto checkPost : theGoalPercept.goalPosts)
    {
      if(!verifiedCenterCircle.isGoalpostCompatibleToCenterCircle(checkPost.positionOnField, theOdometryData))
        return;
      if(!verifiedPenaltyMark.isGoalpostCompatibleToPenaltyMark(checkPost.positionOnField, theOdometryData))
        return;
      if(goalieInPenaltyArea && checkPost.positionOnField.norm() > 3000.f)
        return;
    }

    // Check, if there is a line on which the posts are standing, aka the "ground line"
    bool groundLineFound = false;
    Vector2f groundLineStart;
    Vector2f groundLineEnd;
    for(auto line : theLinePercept.lines)
    {
      const Vector2f groundLineDir = line.last - line.first;
      if(groundLineDir.norm() >= parameters.minGroundLineLength)
      {
        bool allPostsOnLine = true;
        for(auto goalPost : theGoalPercept.goalPosts)
        {
          if(Geometry::getDistanceToLine(Geometry::Line(line.first, groundLineDir), goalPost.positionOnField) >
             parameters.maxGoalPostDistFromGroundLine)
          {
            allPostsOnLine = false;
            break;
          }
        }
        if(allPostsOnLine)
        {
          groundLineFound = true;
          groundLineStart = line.first;
          groundLineEnd = line.last;
          break;
        }
      }
    }

    // First post in list is always considered:
    const Vector2f& positionOnField1(theGoalPercept.goalPosts[0].positionOnField);

    // Buffer data generated from GoalPercept:
    if(theGoalPercept.goalPosts.size() == 2)
    {
      const float realPostDist = 2.f * std::abs(theFieldDimensions.yPosLeftGoal);
      const float seenPostDist = (theGoalPercept.goalPosts[0].positionOnField - theGoalPercept.goalPosts[1].positionOnField).norm();
      if(seenPostDist < 1.2f * realPostDist)
      {
        const Vector2f& positionOnField2(theGoalPercept.goalPosts[1].positionOnField);
        FullGoal newFullGoal;
        const bool leftBeforeRight = theGoalPercept.goalPosts[0].position == GoalPost::IS_LEFT;
        newFullGoal.realLeftPosition = realPostPositions[GoalPost::IS_LEFT];
        newFullGoal.realRightPosition = realPostPositions[GoalPost::IS_RIGHT];
        newFullGoal.seenLeftPosition = leftBeforeRight ? positionOnField1 : positionOnField2;
        newFullGoal.seenRightPosition = leftBeforeRight ? positionOnField2 : positionOnField1;
        newFullGoal.timestamp = theFrameInfo.time;
        newFullGoal.odometry = theOdometryData;
        newFullGoal.groundLineSeen = groundLineFound;
        if(groundLineFound)
        {
          newFullGoal.lineStart = groundLineStart;
          newFullGoal.lineEnd = groundLineEnd;
        }
        // Before adding, check if templates can be generated from this perception
        SampleTemplate checkTemplate = generateTemplateFromFullGoal(newFullGoal);
        if(checkTemplate.timestamp)
          fullGoals.push_front(newFullGoal);
      }
      else
      {
        //OUTPUT_TEXT("Post not accepted! " << seenPostDist);
      }
    }
    else
    {
      const GoalPost& post = theGoalPercept.goalPosts[0];
      if(post.position != GoalPost::IS_UNKNOWN)
      {
        // We might currently see a single goal post with known side (but not a complete goal)
        KnownGoalpost newPost;
        newPost.realPosition = realPostPositions[post.position];
        newPost.seenPosition = positionOnField1;
        if(groundLineFound)
          fillSingleGoalPostMembers(newPost, groundLineFound, groundLineStart, groundLineEnd);
        else
          fillSingleGoalPostMembers(newPost);
        knownGoalposts.push_front(newPost);
      }
      else
      {
        // Maybe we have seen some goalpost of which we do not know the side:
        UnknownGoalpost newPost;
        newPost.realPositions[0] = realPostPositions[GoalPost::IS_LEFT];
        newPost.realPositions[1] = realPostPositions[GoalPost::IS_RIGHT];
        newPost.seenPosition = positionOnField1;
        if(groundLineFound)
          fillSingleGoalPostMembers(newPost, groundLineFound, groundLineStart, groundLineEnd);
        else
          fillSingleGoalPostMembers(newPost);
        // Try to make a guess about the real position of the goal post
        if(positionOnField1.norm() < parameters.templateUnknownPostAssumptionMaxDistance)
        {
          Vector2f postWorld = robotPose * positionOnField1;
          if(postWorld.x() < 0) // Always assume opponent half here
            postWorld = Pose2f(pi) * postWorld;
          const float d0 = (postWorld - newPost.realPositions[0]).norm();
          const float d1 = (postWorld - newPost.realPositions[1]).norm();
          if(3 * d0 < d1)
            newPost.realPositionGuess = 0;
          else if(3 * d1 < d0)
            newPost.realPositionGuess = 1;
          else
            newPost.realPositionGuess = -1;
        }
        else
        {
          newPost.realPositionGuess = -1;
        }
        unknownGoalposts.push_front(newPost);
      }
    }
  }

  // If there are still some too old percepts after adding new ones -> delete them:
  removeOldPercepts(fullGoals);
  removeOldPercepts(knownGoalposts);
  removeOldPercepts(unknownGoalposts);
}

void TemplateGenerator::fillSingleGoalPostMembers(SingleGoalPost& goalPost, bool groundLineSeen,
                                                  Vector2f groundLineStart, Vector2f groundLineEnd)
{
  goalPost.timestamp = theFrameInfo.time;
  goalPost.odometry = theOdometryData;
  goalPost.midLineSeen = false;
  goalPost.groundLineSeen = groundLineSeen;
  goalPost.lineStart = groundLineStart;
  goalPost.lineEnd = groundLineEnd;
  goalPost.centerCircleSeen = false;
  // Check, if the seen center circle is roughly compatible to the goal post:
  if(theLinePercept.circle.found)
  {
    const float realDistance = Vector2f(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosLeftGoal).norm();
    const float measuredDistance = (goalPost.seenPosition - theLinePercept.circle.pos).norm();
    if(std::abs(realDistance - measuredDistance) < parameters.centerCircleGoalPostMaxDistanceDiff)
      goalPost.centerCircleSeen = true;
  }
  if(goalPost.centerCircleSeen)
  {
    goalPost.centerCircleSeenPosition = theLinePercept.circle.pos;
    for(auto line : theLinePercept.lines)
    {
      if(line.midLine)
      {
        goalPost.midLineSeen = true;
        goalPost.lineStart = line.first;
        goalPost.lineEnd = line.last;
        break;
      }
    }
  }
}

template<typename T> void TemplateGenerator::removeOldPercepts(RingBuffer<T, MAX_PERCEPTS>& buffer)
{
  while(!buffer.empty())
  {
    if(theFrameInfo.getTimeSince(buffer.back().timestamp) > parameters.templateMaxKeepTime)
      buffer.pop_back();
    else
      break;
  }
}

TemplateGenerator::SampleTemplate TemplateGenerator::generateTemplate(const Pose2f& lastPose) const
{
  TemplateGenerator::SampleTemplate newTemplate;
  // Current solution: Prefer to construct templates from full goals only:
  if(!fullGoals.empty())
  {
    const FullGoal& goal = fullGoals[random(fullGoals.size())];
    newTemplate = generateTemplateFromFullGoal(goal);
  }
  else if(!knownGoalposts.empty())
  {
    const KnownGoalpost& goalPost = knownGoalposts[random(knownGoalposts.size())];
    if(goalPost.centerCircleSeen)
      newTemplate = generateTemplateFromPositionAndCenterCircle(goalPost.seenPosition, goalPost.centerCircleSeenPosition, goalPost.realPosition, goalPost.odometry);
    if(newTemplate.timestamp == 0)
      newTemplate = generateTemplateFromPosition(goalPost.seenPosition, goalPost.realPosition, goalPost.odometry);
  }
  else if(!unknownGoalposts.empty())
  {
    const UnknownGoalpost& goalPost = unknownGoalposts[random(unknownGoalposts.size())];
    const Vector2f& realPosition = goalPost.realPositionGuess == -1 ? goalPost.realPositions[random(2)] : goalPost.realPositions[goalPost.realPositionGuess];
    if(goalPost.centerCircleSeen)
      newTemplate = generateTemplateFromPositionAndCenterCircle(goalPost.seenPosition, goalPost.centerCircleSeenPosition, realPosition, goalPost.odometry);
    if(newTemplate.timestamp == 0)
      newTemplate = generateTemplateFromPosition(goalPost.seenPosition, realPosition, goalPost.odometry);
  }
  if(newTemplate.timestamp == 0) // In some cases, no proper sample is generated, return the last position and go ahead...
  {
    newTemplate = lastPose;
    newTemplate.origin = TemplateGenerator::SampleTemplate::RANDOM; // Needs to be changed in the future...
  }
  return newTemplate;
}

bool TemplateGenerator::isMirrorCloser(const Pose2f& currentPose, const Pose2f& lastPose) const
{
  const Vector2f& translation = currentPose.translation;
  Vector2f rotationWeight(std::max(theFieldDimensions.yPosLeftSideline * 1.1f - std::min(std::abs(translation.x()), std::abs(lastPose.translation.x())), 0.f), 0);
  Vector2f opponentGoal(theFieldDimensions.xPosOpponentGoalPost, 0.f);
  const Vector2f rotation = Pose2f(Geometry::angleTo(currentPose, opponentGoal)) * rotationWeight;
  const Vector2f lastRotation = Pose2f(Geometry::angleTo(lastPose, opponentGoal)) * rotationWeight;
  bool result = (lastPose.translation - translation).norm() + (lastRotation - rotation).norm() >
                (lastPose.translation + translation).norm() + (lastRotation + rotation).norm();
  return result;
}

bool TemplateGenerator::templatesAvailable() const
{
  return !fullGoals.empty() || !knownGoalposts.empty() || !unknownGoalposts.empty();
}

TemplateGenerator::SampleTemplate TemplateGenerator::generateTemplateFromFullGoal(const FullGoal& goal) const
{
  TemplateGenerator::SampleTemplate newTemplate;
  Pose2f odometryOffset = theOdometryData - goal.odometry;
  Pose2f templatePose = PoseComputation::computePoseFromTwoGoalposts(goal.seenLeftPosition, goal.seenRightPosition,
                                                                     goal.realLeftPosition, goal.realRightPosition,
                                                                     theFieldDimensions.goalPostRadius);
  templatePose += odometryOffset;
  newTemplate = templatePose;
  newTemplate.timestamp = theFrameInfo.time;
  newTemplate.origin = TemplateGenerator::SampleTemplate::GOAL;
  return newTemplate;
}

TemplateGenerator::SampleTemplate TemplateGenerator::generateTemplateFromPositionAndCenterCircle(const Vector2f& posSeen, const Vector2f& circlePosSeen,
    const Vector2f& posReal, const Pose2f& postOdometry) const
{
  TemplateGenerator::SampleTemplate newTemplate;
  Pose2f odometryOffset = theOdometryData - postOdometry;
  float postDist = posSeen.norm();
  float postDistUncertainty = sampleTriangularDistribution(parameters.standardDeviationGoalpostSamplingDistance);
  if(postDist + postDistUncertainty > parameters.standardDeviationGoalpostSamplingDistance)
    postDist += postDistUncertainty;
  float circleDist = circlePosSeen.norm();
  float circleDistUncertainty = sampleTriangularDistribution(parameters.standardDeviationGoalpostSamplingDistance); //No special uncertainty for center circle available
  if(circleDist + circleDistUncertainty > parameters.standardDeviationGoalpostSamplingDistance)
    circleDist += circleDistUncertainty;
  Geometry::Circle c1(posReal, postDist + theFieldDimensions.goalPostRadius);
  Geometry::Circle c2(Vector2f(0.0f, 0.0f), circleDist);
  // If there are intersections, take the first one that is in the field:
  Vector2f p1, p2;
  int result = Geometry::getIntersectionOfCircles(c1, c2, p1, p2);
  if(result)
  {
    Vector2f p = Vector2f::Zero();
    bool p1InsideCarpet = theFieldDimensions.isInsideCarpet(p1);
    bool p2InsideCarpet = theFieldDimensions.isInsideCarpet(p2);
    if(p1InsideCarpet && !p2InsideCarpet)
    {
      p = p1;
    }
    else if(p2InsideCarpet && !p1InsideCarpet)
    {
      p = p2;
    }
    else if(p2InsideCarpet && p1InsideCarpet)
    {
      p = theFieldDimensions.isInsideField(p1) ? p1 : p2;
    }
    if(p1InsideCarpet || p2InsideCarpet)
    {
      float origAngle = (posReal - p).angle();
      float observedAngle = posSeen.angle();
      Pose2f templatePose(origAngle - observedAngle, p1);
      templatePose += odometryOffset;
      newTemplate = templatePose;
      newTemplate.timestamp = theFrameInfo.time;
      newTemplate.origin = TemplateGenerator::SampleTemplate::GOAL;
    }
  }
  // The else case is omitted, calling function has to check the timestamp of the generated sample
  return newTemplate;
}

TemplateGenerator::SampleTemplate TemplateGenerator::generateTemplateFromPosition(
  const Vector2f& posSeen, const Vector2f& posReal,
  const Pose2f& postOdometry) const
{
  TemplateGenerator::SampleTemplate newTemplate;
  float r = posSeen.norm() + theFieldDimensions.goalPostRadius;
  const float distUncertainty = sampleTriangularDistribution(parameters.standardDeviationGoalpostSamplingDistance);
  if(r + distUncertainty > parameters.standardDeviationGoalpostSamplingDistance)
    r += distUncertainty;
  Vector2f realPosition = posReal;
  float minY = std::max(posReal.y() - r, theFieldDimensions.yPosRightFieldBorder);
  float maxY = std::min(posReal.y() + r, theFieldDimensions.yPosLeftFieldBorder);
  Vector2f p;
  p.y() = minY + randomFloat() * (maxY - minY);
  float xOffset(std::sqrt(sqr(r) - sqr(p.y() - posReal.y())));
  p.x() = posReal.x();
  p.x() += (p.x() > 0) ? -xOffset : xOffset;
  if(theFieldDimensions.isInsideCarpet(p))
  {
    float origAngle = (realPosition - p).angle();
    float observedAngle = posSeen.angle();
    Pose2f templatePose(origAngle - observedAngle, p);
    Pose2f odometryOffset = theOdometryData - postOdometry;
    templatePose += odometryOffset;
    newTemplate = templatePose;
    newTemplate.timestamp = theFrameInfo.time;
    newTemplate.origin = TemplateGenerator::SampleTemplate::GOAL;
  }
  return newTemplate;
}

TemplateGenerator::SampleTemplate TemplateGenerator::generateTemplateFromPostAndLine(
  const Vector2f& postSeen, const Vector2f& postReal, const Pose2f& postOdometry,
  const Vector2f& lineBase, const Vector2f& lineDirection, bool isGroundLine)
{
  // Compute x-coordinate by using the line:
  const float lineXReal = isGroundLine ? theFieldDimensions.xPosOpponentGroundline : 0.f;
  const float distance  = std::abs(Geometry::getDistanceToLine(Geometry::Line(lineBase, lineDirection), Vector2f::Zero()));
  const float sampleX   = lineXReal - distance;
  // Radius of circle around goal post:
  float r = postSeen.norm() + theFieldDimensions.goalPostRadius;
  const float distUncertainty = sampleTriangularDistribution(parameters.standardDeviationGoalpostSamplingDistance);
  if(r + distUncertainty > parameters.standardDeviationGoalpostSamplingDistance)
    r += distUncertainty;
  // Intersect circle with line parallel to seen line:
  Vector2f int1, int2;
  bool intersectionFound = Geometry::getIntersectionOfLineAndCircle(Geometry::Line(Vector2f(sampleX, 0.f), Vector2f(0.f, 1.f)),
                           Geometry::Circle(postReal, r), int1, int2) != 0;
  if(intersectionFound)
  {

  }
  else // Find point on line that is closest to the circle
  {

  }
  SampleTemplate newTemplate;
  return newTemplate;
}

Pose2f TemplateGenerator::getTemplateAtReenterPosition()
{
  if(nextReenterTemplateIsLeft)
  {
    nextReenterTemplateIsLeft = false;
    return Pose2f(-pi_2, theFieldDimensions.xPosOwnPenaltyMark, theFieldDimensions.yPosLeftSideline);
  }
  else
  {
    nextReenterTemplateIsLeft = true;
    return Pose2f(pi_2, theFieldDimensions.xPosOwnPenaltyMark, theFieldDimensions.yPosRightSideline);
  }
}

Pose2f TemplateGenerator::getTemplateAtWalkInPosition()
{
  if(theRobotInfo.number >= 1 && theRobotInfo.number <= 5 && !Global::getSettings().isDropInGame)
  {
    const unsigned index = theRobotInfo.number - 1;
    return walkInPositions[index];
  }
  else
  {
    Pose2f result = walkInPositions[nextWalkInTemplateNumber];
    nextWalkInTemplateNumber = (nextWalkInTemplateNumber + 1) % walkInPositions.size();
    return result;
  }
}

Pose2f TemplateGenerator::getTemplateAtManualPlacementPosition(int robotNumber)
{
  // Goalie
  if(Global::getSettings().isGoalkeeper)
  {
    return Pose2f(0.f, theFieldDimensions.xPosOwnGroundline, 0.f);
  }
  else
  {
    float x = theFieldDimensions.xPosOwnPenaltyArea + 100.f;
    float minY = theFieldDimensions.yPosRightSideline + 750.f;
    float y = minY + randomFloat() * (2 * std::abs(minY));
    return Pose2f(0.f, x, y);
  }
}

void TemplateGenerator::draw()
{
  for(const FullGoal& goal : fullGoals)
  {
    Pose2f odometryOffset = goal.odometry - theOdometryData;
    Vector2f leftPost = odometryOffset * goal.seenLeftPosition;
    Vector2f rightPost = odometryOffset * goal.seenRightPosition;
    LINE("module:SelfLocator:templates", leftPost.x(), leftPost.y(),
         rightPost.x(), rightPost.y(), 50, Drawings::solidPen, ColorRGBA(140, 140, 255));
    CIRCLE("module:SelfLocator:templates", leftPost.x(), leftPost.y(),
           100, 20, Drawings::solidPen, ColorRGBA(0, 0, 0), Drawings::solidBrush, ColorRGBA(140, 140, 255));
    CIRCLE("module:SelfLocator:templates", rightPost.x(), rightPost.y(),
           100, 20, Drawings::solidPen, ColorRGBA(0, 0, 0), Drawings::solidBrush, ColorRGBA(140, 140, 255));
    if(goal.groundLineSeen)
    {
      Vector2f lineStartPos = odometryOffset * goal.lineStart;
      Vector2f lineEndPos = odometryOffset * goal.lineEnd;
      CIRCLE("module:SelfLocator:templatesWithLines", leftPost.x(), leftPost.y(),
             100, 20, Drawings::solidPen, ColorRGBA(0, 0, 0), Drawings::solidBrush, ColorRGBA(140, 140, 255));
      CIRCLE("module:SelfLocator:templatesWithLines", rightPost.x(), rightPost.y(),
             100, 20, Drawings::solidPen, ColorRGBA(0, 0, 0), Drawings::solidBrush, ColorRGBA(140, 140, 255));
      LINE("module:SelfLocator:templatesWithLines", lineStartPos.x(), lineStartPos.y(), lineEndPos.x(), lineEndPos.y(),
           20, Drawings::solidPen, ColorRGBA(140, 140, 255));
    }

  }
  for(const KnownGoalpost& post : knownGoalposts)
  {
    Pose2f odometryOffset = post.odometry - theOdometryData;
    Vector2f postPos = odometryOffset * post.seenPosition;
    CIRCLE("module:SelfLocator:templates", postPos.x(), postPos.y(),
           100, 20, Drawings::solidPen, ColorRGBA(140, 140, 255), Drawings::solidBrush, ColorRGBA(140, 140, 255));
    CIRCLE("module:SelfLocator:templates", postPos.x(), postPos.y(),
           200, 20, Drawings::solidPen, ColorRGBA(0, 0, 0), Drawings::noBrush, ColorRGBA(140, 140, 255));
    if(post.centerCircleSeen)
    {
      Vector2f circlePos = odometryOffset * post.centerCircleSeenPosition;
      CIRCLE("module:SelfLocator:templatesWithCenterCircles", postPos.x(), postPos.y(),
             100, 20, Drawings::solidPen, ColorRGBA(140, 140, 255), Drawings::solidBrush, ColorRGBA(140, 140, 255));
      CIRCLE("module:SelfLocator:templatesWithCenterCircles", postPos.x(), postPos.y(),
             200, 20, Drawings::solidPen, ColorRGBA(0, 0, 0), Drawings::noBrush, ColorRGBA(140, 140, 255));
      CIRCLE("module:SelfLocator:templatesWithCenterCircles", circlePos.x(), circlePos.y(),
             200, 20, Drawings::solidPen, ColorRGBA(0, 0, 0), Drawings::noBrush, ColorRGBA(140, 140, 255));
      LINE("module:SelfLocator:templatesWithCenterCircles", postPos.x(), postPos.y(), circlePos.x(), circlePos.y(),
           20, Drawings::solidPen, ColorRGBA(140, 140, 255));
    }
    if(post.groundLineSeen)
    {
      Vector2f lineStartPos = odometryOffset * post.lineStart;
      Vector2f lineEndPos = odometryOffset * post.lineEnd;
      CIRCLE("module:SelfLocator:templatesWithLines", postPos.x(), postPos.y(),
             100, 20, Drawings::solidPen, ColorRGBA(140, 140, 255), Drawings::solidBrush, ColorRGBA(140, 140, 255));
      CIRCLE("module:SelfLocator:templatesWithLines", postPos.x(), postPos.y(),
             200, 20, Drawings::solidPen, ColorRGBA(0, 0, 0), Drawings::noBrush, ColorRGBA(140, 140, 255));
      LINE("module:SelfLocator:templatesWithLines", lineStartPos.x(), lineStartPos.y(), lineEndPos.x(), lineEndPos.y(),
           20, Drawings::solidPen, ColorRGBA(140, 140, 255));
    }
  }
  for(const UnknownGoalpost& post : unknownGoalposts)
  {
    Pose2f odometryOffset = post.odometry - theOdometryData;
    Vector2f postPos = odometryOffset * post.seenPosition;
    CIRCLE("module:SelfLocator:templates", postPos.x(), postPos.y(),
           100, 20, Drawings::solidPen, ColorRGBA(140, 0, 0), Drawings::solidBrush, ColorRGBA(140, 0, 0));
    CIRCLE("module:SelfLocator:templates", postPos.x(), postPos.y(),
           200, 20, Drawings::solidPen, ColorRGBA(0, 0, 0), Drawings::noBrush, ColorRGBA(140, 0, 0));
    if(post.centerCircleSeen)
    {
      Vector2f circlePos = odometryOffset * post.centerCircleSeenPosition;
      CIRCLE("module:SelfLocator:templatesWithCenterCircles", postPos.x(), postPos.y(),
             100, 20, Drawings::solidPen, ColorRGBA(140, 0, 0), Drawings::solidBrush, ColorRGBA(140, 0, 0));
      CIRCLE("module:SelfLocator:templatesWithCenterCircles", postPos.x(), postPos.y(),
             200, 20, Drawings::solidPen, ColorRGBA(0, 0, 0), Drawings::noBrush, ColorRGBA(140, 0, 0));
      CIRCLE("module:SelfLocator:templatesWithCenterCircles", circlePos.x(), circlePos.y(),
             200, 20, Drawings::solidPen, ColorRGBA(0, 0, 0), Drawings::noBrush, ColorRGBA(140, 0, 0));
      LINE("module:SelfLocator:templatesWithCenterCircles", postPos.x(), postPos.y(), circlePos.x(), circlePos.y(),
           20, Drawings::solidPen, ColorRGBA(140, 0, 0));
    }
    if(post.groundLineSeen)
    {
      Vector2f lineStartPos = odometryOffset * post.lineStart;
      Vector2f lineEndPos = odometryOffset * post.lineEnd;
      CIRCLE("module:SelfLocator:templatesWithLines", postPos.x(), postPos.y(),
             100, 20, Drawings::solidPen, ColorRGBA(140, 0, 0), Drawings::solidBrush, ColorRGBA(140, 0, 0));
      CIRCLE("module:SelfLocator:templatesWithLines", postPos.x(), postPos.y(),
             200, 20, Drawings::solidPen, ColorRGBA(0, 0, 0), Drawings::noBrush, ColorRGBA(140, 140, 255));
      LINE("module:SelfLocator:templatesWithLines", lineStartPos.x(), lineStartPos.y(), lineEndPos.x(), lineEndPos.y(),
           20, Drawings::solidPen, ColorRGBA(140, 0, 0));
    }
  }
}

void TemplateGenerator::plot()
{
  DECLARE_PLOT("module:SelfLocator:fullGoalsBuffer");
  PLOT("module:SelfLocator:fullGoalsBuffer", static_cast<float>(fullGoals.size()) / fullGoals.capacity());
  DECLARE_PLOT("module:SelfLocator:knownGoalPostsBuffer");
  PLOT("module:SelfLocator:knownGoalPostsBuffer", static_cast<float>(knownGoalposts.size()) / knownGoalposts.capacity());
  DECLARE_PLOT("module:SelfLocator:unknownGoalPostsBuffer");
  PLOT("module:SelfLocator:unknownGoalPostsBuffer", static_cast<float>(unknownGoalposts.size()) / unknownGoalposts.capacity());
}
