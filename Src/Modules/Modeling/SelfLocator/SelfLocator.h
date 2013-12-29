/**
* @file SelfLocator.h
*
* Declares a class that performs self-localization
*
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
*/

#pragma once

#include "FieldModel.h"
#include "SelfLocatorParameters.h"
#include "UKFSample.h"
#include "TemplateGenerator.h"
#include "Tools/SampleSet.h"
#include "Tools/Module/Module.h"
#include "Representations/BehaviorControl/BehaviorControlOutput.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/CombinedWorldModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Modeling/OwnSideModel.h"
#include "Representations/Modeling/SideConfidence.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Perception/BallPercept.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/FieldBoundary.h"
#include "Representations/Sensing/ArmContactModel.h"
#include "Representations/Sensing/FallDownState.h"


MODULE(SelfLocator)
  REQUIRES(Odometer)
  REQUIRES(OdometryData)
  REQUIRES(OwnSideModel)
  REQUIRES(OwnTeamInfo)
  REQUIRES(GameInfo)
  REQUIRES(RobotInfo)
  REQUIRES(GoalPercept)
  REQUIRES(LinePercept)
  REQUIRES(FieldDimensions)
  REQUIRES(FrameInfo)
  REQUIRES(MotionInfo)
  REQUIRES(CameraMatrix)
  REQUIRES(BallModel)
  REQUIRES(FallDownState)
  REQUIRES(ArmContactModel)
  REQUIRES(FieldBoundary)
  REQUIRES(RobotPose)
  REQUIRES(SideConfidence)                      //  <--- For GO 2013
  USES(CombinedWorldModel)
  USES(BehaviorControlOutput)
  PROVIDES_WITH_OUTPUT(RobotPose)
  // PROVIDES_WITH_MODIFY_AND_DRAW(SideConfidence)  <--- Deactivated for GO 2013
END_MODULE


/**
* @class SelfLocator
*
* A new module for self-localization
*/
class SelfLocator : public SelfLocatorBase
{
private:
  SampleSet<UKFSample>* samples;       /**< Container for all samples. */
  SelfLocatorParameters parameters; /**< Several parameters */
  FieldModel fieldModel;            /**< Information about the robot's environment */
  TemplateGenerator sampleGenerator;  /**< Several parameters */
  float mirrorLikelihood;              /**< Value [0..1] indicating how many samples assume to be mirrored*/
  int lastPenalty;                     /**< Was the robot penalised in the last frame? */
  int lastGameState;                   /**< The game state in the last frame. */
  Pose2D lastRobotPose;                /**< The result of the last computation */
  unsigned timeOfLastFall;             /**< Timestamp to see if the robot has fallen down */
  unsigned lastTimeWithoutArmContact;  /**< Timestamp for incorporating arm collisions into the model */
  unsigned lastTimeFarGoalSeen;        /**< Timestamp for checking goalie localization */
  bool sampleSetHasBeenResetted;       /**< Flag indicating that all samples have been replaced in the current frame */

  /**
  * The method provides the robot pose (equal to the filtered robot pose).
  *
  * @param robotPose The robot pose representation that is updated by this module.
  */
  void update(RobotPose& robotPose);

   /**
  * The method provides the SideConfidence based on the uniformity of the distibution
  *
  * @param sideConfidence The side confidence
  */
  void update(SideConfidence& sideConfidence);

  /** Integrate odometry offset into hypotheses */
  void motionUpdate();

  void mirrorFlagUpdate();

  void sensorUpdate();

  void resampling();

  void sensorResetting(const RobotPose& robotPose);

  int sampleSetIsMultimodal();

  bool sufficientBallInformationAvailable();

  void handleGameStateChanges(const Pose2D& propagatedRobotPose);

  void computeMirrorLikelihoodAndAdaptMirrorFlags();

  void handleSideConfidence();

  void computeModel(RobotPose& robotPose);

  void domainSpecificSituationHandling();

  UKFSample& getMostValidSample();

  void computeSampleValidities();

  bool isMirrorCloser(const Pose2D& samplePose, const Pose2D& robotPose) const;

  /** draw debug information */
  void draw();

public:
  /** Default constructor. */
  SelfLocator();

  /** Destructor. */
  ~SelfLocator();
};
