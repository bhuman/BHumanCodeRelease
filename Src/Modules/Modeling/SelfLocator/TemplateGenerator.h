/**
* @file TemplateGenerator.h
*
* This file declares a submodule that generates robot positions from percepts.
*
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once

#include "SelfLocatorBase.h"
#include "VerifiedCenterCircle.h"
#include "VerifiedPenaltyMark.h"
#include "Tools/RingBuffer.h"

/**
* @class TemplateGenerator
*
* A module for computing poses from percepts
*/
class TemplateGenerator
{
private:
  const SelfLocatorBase::Parameters& parameters;
  const GoalPercept& theGoalPercept;
  const LinePercept& theLinePercept;
  const FrameInfo& theFrameInfo;
  const FieldDimensions& theFieldDimensions;
  const OdometryData& theOdometryData;
  const MotionRequest& theMotionRequest;
  const RobotInfo& theRobotInfo;
  const VerifiedCenterCircle& verifiedCenterCircle;
  const VerifiedPenaltyMark& verifiedPenaltyMark;

  /**
  * @class SampleTemplate
  * Robot position generated from perceptions
  */
  class SampleTemplate : public Pose2f
  {
  public:
    /** Constructor */
    SampleTemplate(): Pose2f(), timestamp(0) {}

    /** Constructor */
    SampleTemplate(const Pose2f& pose): Pose2f(pose), timestamp(0) {}

    /** Timestamp of visual input for construction of this template */
    unsigned timestamp;

    /** Indicates whether the sample is just a random position or has been computed from (a) seen goal post(s) */
    enum Origin {GOAL, RANDOM} origin;
  };

  class FullGoal
  {
  public:
    Vector2f seenLeftPosition = Vector2f::Zero();
    Vector2f realLeftPosition = Vector2f::Zero();
    Vector2f seenRightPosition = Vector2f::Zero();
    Vector2f realRightPosition = Vector2f::Zero();
    bool groundLineSeen;
    Vector2f lineStart = Vector2f::Zero();
    Vector2f lineEnd = Vector2f::Zero();
    int timestamp;
    Pose2f odometry;
  };
  
  class SingleGoalPost
  {
  public:
    Vector2f seenPosition = Vector2f::Zero();
    int timestamp;
    Pose2f odometry;
    bool centerCircleSeen;
    Vector2f centerCircleSeenPosition = Vector2f::Zero();
    bool midLineSeen;
    bool groundLineSeen;
    Vector2f lineStart = Vector2f::Zero();
    Vector2f lineEnd = Vector2f::Zero();
  };

  class KnownGoalpost : public SingleGoalPost
  {
  public:
    Vector2f realPosition = Vector2f::Zero();
  };

  class UnknownGoalpost : public SingleGoalPost
  {
  public:
    Vector2f realPositions[2];
    int realPositionGuess;
  };

  enum {MAX_PERCEPTS = 10};
  RingBuffer<FullGoal, MAX_PERCEPTS> fullGoals;
  RingBuffer<KnownGoalpost, MAX_PERCEPTS> knownGoalposts;
  RingBuffer<UnknownGoalpost, MAX_PERCEPTS> unknownGoalposts;
  Vector2f realPostPositions[GoalPost::numOfPositions];
  std::vector<Pose2f> walkInPositions;
  unsigned int nextWalkInTemplateNumber;
  bool nextReenterTemplateIsLeft;  // Does not need any initialization

  template<typename T> void removeOldPercepts(RingBuffer<T, MAX_PERCEPTS>& buffer);

  /**
  * Generates a new sample by using the perceptions of both posts of a goal
  * @param goal The goal
  * @return A samples; calling function has to check the timestamp of the generated sample to determine its validity
  */
  SampleTemplate generateTemplateFromFullGoal(const FullGoal& goal) const;

  SampleTemplate generateTemplateFromPositionAndCenterCircle(const Vector2f& posSeen, const Vector2f& circlePosSeen,
      const Vector2f& posReal, const Pose2f& postOdometry) const;

  SampleTemplate generateTemplateFromPosition(const Vector2f& posSeen,
      const Vector2f& posReal, const Pose2f& postOdometry) const;

  SampleTemplate generateTemplate(const Pose2f& lastPose) const;

  SampleTemplate generateTemplateFromPostAndLine(const Vector2f& postSeen, const Vector2f& postReal, const Pose2f& postOdometry,
                                                 const Vector2f& lineBase, const Vector2f& lineDirection, bool isGroundLine = true);

  bool halfChangeNeeded(const Pose2f& pose, const Pose2f& odometry, const Vector2f& seenPost, const Vector2f& realPost) const;
  
  void fillSingleGoalPostMembers(SingleGoalPost& goalPost, bool groundLineSeen = false,
                                 Vector2f groundLineStart = Vector2f::Zero(), Vector2f groundLineEnd = Vector2f::Zero());

public:
  enum ForceHalf {OWN_HALF, OPPONENT_HALF, CONSIDER_POSE, RANDOM_HALF};

  TemplateGenerator(const SelfLocatorBase::Parameters& parameters,
                    const GoalPercept& goalPercept, const LinePercept& linePercept, const FrameInfo& frameInfo,
                    const FieldDimensions& fieldDimensions, const OdometryData& odometryData,
                    const MotionRequest& motionRequest, const RobotInfo& robotInfo,
                    const VerifiedCenterCircle& verifiedCenterCircle,
                    const VerifiedPenaltyMark& verifiedPenaltyMark);

  /** Empty all buffers. */
  void init();

  /** Buffers current goal perceptions. */
  void bufferNewPerceptions(const Pose2f& robotPose);

  Pose2f getTemplate(ForceHalf forceHalf, const Pose2f& robotPose, const Pose2f& lastRobotPose) const;

  Pose2f getTemplateAtReenterPosition();

  Pose2f getTemplateAtWalkInPosition();

  Pose2f getTemplateAtManualPlacementPosition(int robotNumber);

  bool templatesAvailable() const;

  bool isMirrorCloser(const Pose2f& currentPose, const Pose2f& lastPose) const;

  void draw();

  void plot();
};
