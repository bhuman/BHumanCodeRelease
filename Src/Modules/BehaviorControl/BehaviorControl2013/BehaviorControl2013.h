/**
 * @file BehaviorControl2013.h
 * Declaration of the base class of the C-based state machine behavior control module.
 * @author Thomas Röfer
 * @author Tim Laue
 */

#pragma once

#include <sstream>
#include <limits>
#include <algorithm>

#include "Tools/Math/Geometry.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Module/Module.h"
#include "Tools/PathFinder.h"
#include "Representations/BehaviorControl/BehaviorControlOutput.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/BehaviorLEDRequest.h"
#include "Representations/BehaviorControl/Role.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/HeadLimits.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/KeyStates.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Modeling/TeammateReliability.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/CombinedWorldModel.h"
#include "Representations/Modeling/GlobalFieldCoverage.h"
#include "Representations/Modeling/ExpObstacleModel.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/SoundSignal.h"
#include "Representations/MotionControl/ArmMotionEngineOutput.h"
#include "Representations/MotionControl/ArmMotionRequest.h"
#include "Representations/MotionControl/HeadJointRequest.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/KickEngineOutput.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/Perception/GoalPercept.h"
#include "Representations/Perception/LinePercept.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/FieldBoundary.h"
#include "Representations/Modeling/ObstacleWheel.h"
#include "Representations/Sensing/ArmContactModel.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/FootContactModel.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/Sensing/InertiaSensorData.h"

MODULE(BehaviorControl2013,
{,
  REQUIRES(ArmContactModel),
  REQUIRES(ArmMotionEngineOutput),
  REQUIRES(BallModel),
  REQUIRES(KickEngineOutput),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(CombinedWorldModel),
  REQUIRES(DamageConfiguration),
  REQUIRES(ExpObstacleModel),
  REQUIRES(FallDownState),
  REQUIRES(FrameInfo),
  REQUIRES(FieldBoundary),
  REQUIRES(FilteredJointData),
  REQUIRES(FilteredSensorData),
  REQUIRES(FieldDimensions),
  REQUIRES(FootContactModel),
  REQUIRES(GameInfo),
  REQUIRES(GlobalFieldCoverage),
  REQUIRES(GoalPercept),
  REQUIRES(GroundContactState),
  REQUIRES(HeadJointRequest),
  REQUIRES(HeadLimits),
  REQUIRES(Image),
  REQUIRES(InertiaSensorData),
  REQUIRES(JointData),
  REQUIRES(JointRequest),
  REQUIRES(KeyStates),
  REQUIRES(LinePercept),
  REQUIRES(MotionInfo),
  REQUIRES(ObstacleModel),
  REQUIRES(ObstacleWheel),
  REQUIRES(OwnTeamInfo),
  REQUIRES(OpponentTeamInfo),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotInfo),
  REQUIRES(RobotPose),
  REQUIRES(Role),
  REQUIRES(SoundSignal),
  REQUIRES(TeammateData),
  REQUIRES(TorsoMatrix), // Required for kicks
  REQUIRES(TeammateReliability),
  REQUIRES(WalkingEngineOutput),
  PROVIDES_WITH_MODIFY(BehaviorControlOutput),
  REQUIRES(BehaviorControlOutput),
  PROVIDES_WITH_MODIFY(ArmMotionRequest),
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(MotionRequest),
  PROVIDES_WITH_MODIFY(HeadMotionRequest),
  PROVIDES_WITH_MODIFY_AND_OUTPUT(ActivationGraph),
  PROVIDES_WITH_MODIFY(BehaviorLEDRequest),
});

namespace Behavior2013
{
  /**
   * Common base class for behavior options and libraries.
   */
  class BehaviorBase : public BehaviorControl2013Base
  {
  private:
    void update(BehaviorControlOutput& behaviorControlOutput) {}
    void update(MotionRequest& motionRequest) {}
    void update(ArmMotionRequest& armMotionRequest) {}
    void update(HeadMotionRequest& headMotionRequest) {}
    void update(BehaviorLEDRequest& behaviorLEDRequest) {}
    void update(ActivationGraph& executionGraph) {}

  public:
    BehaviorStatus& theBehaviorStatus;
    ArmMotionRequest& theArmMotionRequest;
    MotionRequest& theMotionRequest;
    HeadMotionRequest& theHeadMotionRequest; /**< The head motion request that will be set. */
    GameInfo& theGameInfo; /**< The game info that might be modified through button clicks. */
    RobotInfo& theRobotInfo; /**< The robot info that might be modified through button clicks. */
    OwnTeamInfo& theOwnTeamInfo; /**< The own team info that might be modified through button clicks. */
    ActivationGraph& theActivationGraph;
    BehaviorLEDRequest& theBehaviorLEDRequest;

    /**
     * Constructor.
     * Note that the constructor uses the default constructor of the base class.
     * @param base The behavior base class some attributes are copied from.
     * @param behaviorControlOutput The behavior output some attributes of which are provided as
     *                              separate entries.
     */
    BehaviorBase(const BehaviorControl2013Base& base,
                 BehaviorControlOutput& behaviorControlOutput)
    : theBehaviorStatus(behaviorControlOutput.behaviorStatus),
      theArmMotionRequest(behaviorControlOutput.armMotionRequest),
      theMotionRequest(behaviorControlOutput.motionRequest),
      theHeadMotionRequest(behaviorControlOutput.headMotionRequest),
      theGameInfo(behaviorControlOutput.gameInfo),
      theRobotInfo(behaviorControlOutput.robotInfo),
      theOwnTeamInfo(behaviorControlOutput.ownTeamInfo),
      theActivationGraph(behaviorControlOutput.executionGraph),
      theBehaviorLEDRequest(behaviorControlOutput.behaviorLEDRequest)
    {}

    /**
     * Copy constructor.
     * Note that the constructor uses the default constructor of the base class, because
     * the attributes of the base class should not be copied.
     * @param other The object that is copied.
     */
    BehaviorBase(const BehaviorBase& other)
    : theBehaviorStatus(other.theBehaviorStatus),
      theArmMotionRequest(other.theArmMotionRequest),
      theMotionRequest(other.theMotionRequest),
      theHeadMotionRequest(other.theHeadMotionRequest),
      theGameInfo(other.theGameInfo),
      theRobotInfo(other.theRobotInfo),
      theOwnTeamInfo(other.theOwnTeamInfo),
      theActivationGraph(other.theActivationGraph),
      theBehaviorLEDRequest(other.theBehaviorLEDRequest)
    {}
  };
}
