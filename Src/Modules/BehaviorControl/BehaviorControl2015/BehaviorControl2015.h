/**
 * @file BehaviorControl2015.h
 * Declaration of the base class of the C-based state machine behavior control module.
 * @author Thomas Röfer
 * @author Tim Laue
 */

#pragma once

#include "Representations/BehaviorControl/ActivationGraph.h"
#include "Representations/BehaviorControl/BehaviorLEDRequest.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/Role.h"
#include "Representations/BehaviorControl/SPLStandardBehaviorStatus.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/HeadLimits.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/SideConfidence.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Modeling/TeammateReliability.h"
#include "Representations/Modeling/TeamPlayersModel.h"
#include "Representations/Modeling/Whistle.h"
#include "Representations/MotionControl/ArmKeyFrameEngineOutput.h"
#include "Representations/MotionControl/ArmMotionSelection.h"
#include "Representations/MotionControl/HeadJointRequest.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/MotionControl/KickEngineOutput.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionSelection.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/FieldBoundary.h"
#include "Representations/Perception/GoalPercept.h"
#include "Representations/Perception/LinePercept.h"
#include "Representations/Sensing/ArmContactModel.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/FootContactModel.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Tools/Debugging/Annotation.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Random.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Modeling/BallPhysics.h"
#include "Tools/Module/Module.h"

#include <algorithm>
#include <limits>
#include <sstream>

MODULE(BehaviorControl2015,
{,
  REQUIRES(ArmContactModel),
  REQUIRES(ArmKeyFrameEngineOutput),
  REQUIRES(BallModel),
  REQUIRES(CameraCalibration),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(DamageConfigurationBody),
  REQUIRES(FallDownState),
  REQUIRES(FieldBoundary),
  REQUIRES(FieldDimensions),
  REQUIRES(FootContactModel),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(GoalPercept),
  REQUIRES(GroundContactState),
  REQUIRES(HeadJointRequest),
  REQUIRES(HeadLimits),
  REQUIRES(Image),
  REQUIRES(JointAngles),
  REQUIRES(JointRequest),
  REQUIRES(KeyStates),
  REQUIRES(KickEngineOutput),
  REQUIRES(LinePercept),
  REQUIRES(MotionInfo),
  REQUIRES(MotionSelection),
  REQUIRES(ArmMotionSelection),
  REQUIRES(ObstacleModel),
  REQUIRES(Odometer),
  REQUIRES(OpponentTeamInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(RawGameInfo),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotInfo),
  REQUIRES(RobotModel),
  REQUIRES(RobotPose),
  REQUIRES(Role),
  REQUIRES(SideConfidence),
  REQUIRES(TeamBallModel),
  REQUIRES(TeammateData),
  REQUIRES(TeammateReliability),
  REQUIRES(TeamPlayersModel),
  REQUIRES(TorsoMatrix), // Required for kicks
  REQUIRES(WalkingEngineOutput),
  REQUIRES(Whistle),
  REQUIRES(ActivationGraph),
  PROVIDES(ActivationGraph),
  PROVIDES(ArmMotionRequest),
  PROVIDES(BehaviorLEDRequest),
  PROVIDES(MotionRequest),
  PROVIDES(BehaviorStatus),
  PROVIDES(HeadMotionRequest),
  PROVIDES(SPLStandardBehaviorStatus),
});

namespace Behavior2015
{
  /**
   * Container for references to representations modified by the behavior.
   */
  class BehaviorData
  {
  public:
    ActivationGraph& theActivationGraph;
    BehaviorLEDRequest& theBehaviorLEDRequest;
    BehaviorStatus& theBehaviorStatus;
    HeadMotionRequest& theHeadMotionRequest;
    ArmMotionRequest& theArmMotionRequest;
    MotionRequest& theMotionRequest;
    SPLStandardBehaviorStatus& theSPLStandardBehaviorStatus;

    BehaviorData(ActivationGraph& theActivationGraph,
                 BehaviorLEDRequest& theBehaviorLEDRequest,
                 BehaviorStatus& theBehaviorStatus,
                 HeadMotionRequest& theHeadMotionRequest,
                 ArmMotionRequest& theArmMotionRequest,
                 MotionRequest& theMotionRequest,
                 SPLStandardBehaviorStatus& theSPLStandardBehaviorStatus)
    : theActivationGraph(theActivationGraph),
      theBehaviorLEDRequest(theBehaviorLEDRequest),
      theBehaviorStatus(theBehaviorStatus),
      theHeadMotionRequest(theHeadMotionRequest),
      theArmMotionRequest(theArmMotionRequest),
      theMotionRequest(theMotionRequest),
      theSPLStandardBehaviorStatus(theSPLStandardBehaviorStatus){}
  };

  /**
   * Common base class for behavior options and libraries.
   */
  class BehaviorBase : public BehaviorControl2015Base, public BehaviorData
  {
  private:
    void update(ActivationGraph&) {}
    void update(BehaviorLEDRequest&) {}
    void update(MotionRequest&) {}
    void update(BehaviorStatus&) {}
    void update(HeadMotionRequest&) {}
    void update(ArmMotionRequest&) {}
    void update(SPLStandardBehaviorStatus&) {}

  public:
    using BehaviorData::theActivationGraph; /**< Use the non-const version. */

    /**
     * Constructor.
     * Note that the constructor uses the default constructor of the base class.
     * @param base The behavior base class some attributes are copied from.
     * @param behaviorData The data modified by the behavior.
     */
    BehaviorBase(const BehaviorControl2015Base& base,
                 BehaviorData& behaviorData)
    : BehaviorData(behaviorData) {}

    /**
     * Copy constructor.
     * Note that the constructor uses the default constructor of the base class, because
     * the attributes of the base class should not be copied.
     * @param other The object that is copied.
     */
    BehaviorBase(const BehaviorBase& other)
    : BehaviorData(other) {}
  };
}
