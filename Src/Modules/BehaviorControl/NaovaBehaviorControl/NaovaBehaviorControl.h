/**
 * @file NaovaBehaviorControl.h
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
#include "Representations/Communication/NoWirelessReturnData.h"
#include "Representations/Communication/TeammateData.h"
#include "Representations/Configuration/NaovaBehaviorParameters.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/HeadLimits.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/CameraStatus.h"
#include "Representations/Infrastructure/CognitionStateChanges.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/FieldCoverage.h"
#include "Representations/Modeling/GlobalFieldCoverage.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/SideConfidence.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Modeling/TeammateReliability.h"
#include "Representations/Modeling/TeamPlayersModel.h"
#include "Representations/MotionControl/ArmKeyFrameEngineOutput.h"
#include "Representations/MotionControl/ArmMotionRequest.h"
#include "Representations/MotionControl/ArmMotionSelection.h"
#include "Representations/MotionControl/HeadJointRequest.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/MotionControl/KickEngineOutput.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/LegMotionSelection.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/FieldBoundary.h"
#include "Representations/Perception/FieldPercepts/FieldLines.h"
#include "Representations/Perception/FieldFeatures/FieldFeatureOverview.h"
#include "Representations/Sensing/ArmContactModel.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/FootBumperState.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Tools/Debugging/Annotation.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Random.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Modeling/BallPhysics.h"
#include "Tools/Module/Module.h"
#include "Tools/RingBufferWithSum.h"

#include <algorithm>
#include <limits>
#include <sstream>

MODULE(NaovaBehaviorControl,
{,
  REQUIRES(ActivationGraph),
  REQUIRES(ArmContactModel),
  REQUIRES(ArmKeyFrameEngineOutput),
  REQUIRES(ArmMotionInfo),
  REQUIRES(ArmMotionSelection),
  REQUIRES(BallModel),
  REQUIRES(NaovaBehaviorParameters),
  REQUIRES(CameraCalibration),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(CameraStatus),
  REQUIRES(CognitionStateChanges),
  REQUIRES(DamageConfigurationBody),
  REQUIRES(FallDownState),
  REQUIRES(FieldCoverage),
  REQUIRES(FieldBoundary),
  REQUIRES(FieldDimensions),
  REQUIRES(FieldFeatureOverview),
  REQUIRES(FootBumperState),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(GlobalFieldCoverage),
  REQUIRES(GroundContactState),
  REQUIRES(HeadJointRequest),
  REQUIRES(HeadLimits),
  REQUIRES(Image),
  REQUIRES(JointAngles),
  REQUIRES(JointRequest),
  REQUIRES(KeyStates),
  REQUIRES(KickEngineOutput),
  REQUIRES(FieldLines),
  REQUIRES(MotionInfo),
  REQUIRES(LegMotionSelection),
  REQUIRES(ObstacleModel),
  REQUIRES(Odometer),
  REQUIRES(OpponentTeamInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(NoWirelessReturnData),
  REQUIRES(RawGameInfo),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotInfo),
  REQUIRES(RobotModel),
  REQUIRES(RobotPose),
  REQUIRES(Role),
  REQUIRES(SideConfidence),
  REQUIRES(TeamBallModel),
  REQUIRES(TeamPlayersModel),
  REQUIRES(TeammateData),
  REQUIRES(TeammateReliability),
  REQUIRES(TorsoMatrix), // Required for kicks
  REQUIRES(WalkingEngineOutput),
  PROVIDES(ActivationGraph),
  PROVIDES(ArmMotionRequest),
  PROVIDES(BehaviorLEDRequest),
  PROVIDES(BehaviorMotionRequest),
  PROVIDES(BehaviorStatus),
  PROVIDES(HeadMotionRequest),
  PROVIDES(SPLStandardBehaviorStatus),
});

namespace NaovaBehavior
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
                 SPLStandardBehaviorStatus& theSPLStandardBehaviorStatus) :
      theActivationGraph(theActivationGraph),
      theBehaviorLEDRequest(theBehaviorLEDRequest),
      theBehaviorStatus(theBehaviorStatus),
      theHeadMotionRequest(theHeadMotionRequest),
      theArmMotionRequest(theArmMotionRequest),
      theMotionRequest(theMotionRequest),
      theSPLStandardBehaviorStatus(theSPLStandardBehaviorStatus)
    {}
  };

  /**
   * Common base class for behavior options and libraries.
   */
  class BehaviorBase : public NaovaBehaviorControlBase, public BehaviorData
  {
  private:
    void update(ActivationGraph&) {}
    void update(BehaviorLEDRequest&) {}
    void update(BehaviorMotionRequest&) {}
    void update(BehaviorStatus&) {}
    void update(HeadMotionRequest&) {}
    void update(ArmMotionRequest&) {}
    void update(SPLStandardBehaviorStatus&) {}

    using NaovaBehaviorControlBase::theNaovaBehaviorParameters; /**< Hide this symbol from derived class. */

  public:
    const NaovaBehaviorParameters& theBehaviorParameters;
    using BehaviorData::theActivationGraph; /**< Use the non-const version. */

    /**
     * Constructor.
     * Note that the constructor uses the default constructor of the base class.
     * @param base The behavior base class some attributes are copied from.
     * @param behaviorData The data modified by the behavior.
     */
    BehaviorBase(const NaovaBehaviorControlBase& base, BehaviorData& behaviorData) :
      BehaviorData(behaviorData), theBehaviorParameters(theNaovaBehaviorParameters)
    {}

    /**
     * Copy constructor.
     * Note that the constructor uses the default constructor of the base class, because
     * the attributes of the base class should not be copied.
     * @param other The object that is copied.
     */
    BehaviorBase(const BehaviorBase& other) :
      BehaviorData(other), theBehaviorParameters(other.theBehaviorParameters)
    {}
  };
}
