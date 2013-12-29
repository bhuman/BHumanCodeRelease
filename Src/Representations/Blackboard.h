/**
* @file Blackboard.h
* Declaration of a class representing the blackboard.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once

#include <cstddef>
#include "Platform/SystemCall.h"

// Declare prototypes of all representations here:

// Infrastructure
class ButtonInterface;
class JointData;
class JointRequest;
class SensorData;
class KeyStates;
class LEDRequest;
class Image;
class CameraInfo;
class FrameInfo;
class CognitionFrameInfo;
class RobotInfo;
class OwnTeamInfo;
class OpponentTeamInfo;
class GameInfo;
class TeamMateData;
class MotionRobotHealth;
class RobotHealth;
class TeamDataSenderOutput;
class USRequest;
class Thumbnail;

// Configuration
class CameraSettings;
class FieldDimensions;
class RobotDimensions;
class JointCalibration;
class SensorCalibration;
class CameraCalibration;
class MassCalibration;
class HardnessSettings;
class DamageConfiguration;
class HeadLimits;

// Perception
class CameraMatrix;
class RobotCameraMatrix;
class ImageCoordinateSystem;
class BallSpots;
class LineSpots;
class PossibleObstacleSpots;
class BallPercept;
class LinePercept;
class RegionPercept;
class GoalPercept;
class GroundContactState;
class BodyContour;
class ColorReference;
class FieldBoundary;
class ObstacleSpots;

// Modeling
class ArmContactModel;
class FallDownState;
class BallModel;
class CombinedWorldModel;
class GroundTruthBallModel;
class ObstacleModel;
class USObstacleModel;
class RobotPose;
class FilteredRobotPose;
class FootContactModel;
class GroundTruthRobotPose;
class RobotsModel;
class GroundTruthRobotsModel;
class FreePartOfOpponentGoalModel;
class FieldCoverage;
class GlobalFieldCoverage;
class SideConfidence;
class Odometer;
class OwnSideModel;
class ObstacleWheel;
class ObstacleClusters;

// BehaviorControl
class ActivationGraph;
class BehaviorControlOutput;
class BehaviorLEDRequest;
class Path;

// Sensing
class FilteredJointData;
class FilteredSensorData;
class InertiaSensorData;
class OrientationData;
class GroundTruthOrientationData;
class TorsoMatrix;
class RobotModel;
class JointDynamics;
class FutureJointDynamics;
class RobotBalance;
class FsrData;
class FsrZmp;

// MotionControl
class ArmMotionEngineOutput;
class ArmMotionRequest;
class OdometryData;
class GroundTruthOdometryData;
class GetUpEngineOutput;
class MotionRequest;
class HeadMotionRequest;
class HeadAngleRequest;
class HeadJointRequest;
class MotionSelection;
class SpecialActionsOutput;
class WalkingEngineOutput;
class WalkingEngineStandOutput;
class BikeEngineOutput;
class MotionInfo;
class BallTakingOutput;
class IndykickEngineOutput;

// friends
class Process;
class Cognition;
class Motion;
class Framework;
class CognitionLogger;

/**
* @class Blackboard
* The class represents the blackboard that contains all representation.
* Note: The blackboard only contains references to the objects as attributes.
* The actual representations are constructed on the heap, because many copies of
* of the blackboard exist but only a single set of the representations shared
* by all instances.
*/
class Blackboard
{
protected:
  // Add all representations as constant references here:
  // Infrastructure
  const ButtonInterface& theButtonInterface;
  const JointData& theJointData;
  const JointRequest& theJointRequest;
  const SensorData& theSensorData;
  const KeyStates& theKeyStates;
  const LEDRequest& theLEDRequest;
  const Image& theImage;
  const CameraInfo& theCameraInfo;
  const FrameInfo& theFrameInfo;
  const CognitionFrameInfo& theCognitionFrameInfo;
  const RobotInfo& theRobotInfo;
  const OwnTeamInfo& theOwnTeamInfo;
  const OpponentTeamInfo& theOpponentTeamInfo;
  const GameInfo& theGameInfo;
  const TeamMateData& theTeamMateData;
  const MotionRobotHealth& theMotionRobotHealth;
  const RobotHealth& theRobotHealth;
  const TeamDataSenderOutput& theTeamDataSenderOutput;
  const USRequest& theUSRequest;
  const Thumbnail& theThumbnail;

  // Configuration
  const CameraSettings& theCameraSettings;
  const FieldDimensions& theFieldDimensions;
  const RobotDimensions& theRobotDimensions;
  const JointCalibration& theJointCalibration;
  const SensorCalibration& theSensorCalibration;
  const CameraCalibration& theCameraCalibration;
  const MassCalibration& theMassCalibration;
  const HardnessSettings& theHardnessSettings;
  const DamageConfiguration& theDamageConfiguration;
  const HeadLimits& theHeadLimits;

  // Perception
  const CameraMatrix& theCameraMatrix;
  const RobotCameraMatrix& theRobotCameraMatrix;
  const ImageCoordinateSystem& theImageCoordinateSystem;
  const BallSpots& theBallSpots;
  const LineSpots& theLineSpots;
  const PossibleObstacleSpots& thePossibleObstacleSpots;
  const BallPercept& theBallPercept;
  const LinePercept& theLinePercept;
  const RegionPercept& theRegionPercept;
  const GoalPercept& theGoalPercept;
  const GroundContactState& theGroundContactState;
  const BodyContour& theBodyContour;
  const ColorReference& theColorReference;
  const FieldBoundary& theFieldBoundary;
  const ObstacleSpots& theObstacleSpots;

  // Modeling
  const ArmContactModel& theArmContactModel;
  const FallDownState& theFallDownState;
  const BallModel& theBallModel;
  const CombinedWorldModel& theCombinedWorldModel;
  const GroundTruthBallModel& theGroundTruthBallModel;
  const ObstacleModel& theObstacleModel;
  const USObstacleModel& theUSObstacleModel;
  const RobotPose& theRobotPose;
  const FilteredRobotPose& theFilteredRobotPose;
  const FootContactModel& theFootContactModel;
  const GroundTruthRobotPose& theGroundTruthRobotPose;
  const RobotsModel& theRobotsModel;
  const GroundTruthRobotsModel& theGroundTruthRobotsModel;
  const FreePartOfOpponentGoalModel& theFreePartOfOpponentGoalModel;
  const FieldCoverage& theFieldCoverage;
  const GlobalFieldCoverage& theGlobalFieldCoverage;
  const SideConfidence& theSideConfidence;
  const Odometer& theOdometer;
  const OwnSideModel& theOwnSideModel;
  const ObstacleWheel& theObstacleWheel;
  const ObstacleClusters& theObstacleClusters;

  // BehaviorControl
  const ActivationGraph& theActivationGraph;
  const BehaviorControlOutput& theBehaviorControlOutput;
  const BehaviorLEDRequest& theBehaviorLEDRequest;
  const Path& thePath;

  // Sensing
  const FilteredJointData& theFilteredJointData;
  const FilteredSensorData& theFilteredSensorData;
  const InertiaSensorData& theInertiaSensorData;
  const OrientationData& theOrientationData;
  const GroundTruthOrientationData& theGroundTruthOrientationData;
  const TorsoMatrix& theTorsoMatrix;
  const RobotModel& theRobotModel;
  const JointDynamics& theJointDynamics;
  const FutureJointDynamics& theFutureJointDynamics;
  const RobotBalance& theRobotBalance;
  const FsrData& theFsrData;
  const FsrZmp& theFsrZmp;

  // MotionControl
  const ArmMotionEngineOutput& theArmMotionEngineOutput;
  const ArmMotionRequest& theArmMotionRequest;
  const OdometryData& theOdometryData;
  const GroundTruthOdometryData& theGroundTruthOdometryData;
  const GetUpEngineOutput& theGetUpEngineOutput;
  const MotionRequest& theMotionRequest;
  const HeadAngleRequest& theHeadAngleRequest;
  const HeadMotionRequest& theHeadMotionRequest;
  const HeadJointRequest& theHeadJointRequest;
  const MotionSelection& theMotionSelection;
  const SpecialActionsOutput& theSpecialActionsOutput;
  const WalkingEngineOutput& theWalkingEngineOutput;
  const WalkingEngineStandOutput& theWalkingEngineStandOutput;
  const BikeEngineOutput& theBikeEngineOutput;
  const MotionInfo& theMotionInfo;
  const BallTakingOutput& theBallTakingOutput;
  const IndykickEngineOutput& theIndykickEngineOutput;

  static PROCESS_WIDE_STORAGE(Blackboard) theInstance; /**< The only real instance in the current process. */

  /**
  * The method is a dummy that is called to prevent the compiler from certain
  * optimizations in a method generated in Module.h.
  * It is empty, but important, not defined inline.
  */
  static void distract();

private:
  /**
  * Default constructor.
  */
  Blackboard();

public:
  /**
  * Virtual destructor.
  * Required for derivations of this class.
  */
  virtual ~Blackboard() {}

  /**
  * Assignment operator.
  * Note: copies will share all representations.
  * @param other The instance that is cloned.
  */
  void operator=(const Blackboard& other);

  /**
  * The operator allocates a memory block that is zeroed.
  * Therefore, all members of this class are initialized with 0.
  * @attention This operator is only called if this class is instantiated by
  * a separate call to new, i.e. it cannot be created as a part of another class.
  * @param size The size of the block in bytes.
  * @return A pointer to the block.
  */
  static void* operator new(std::size_t);

  /**
  * The operator frees a memory block.
  * @param p The address of the block to free.
  */
  static void operator delete(void* p);

  friend class Process; /**< The class Process can set theInstance. */
  friend class Cognition; /**< The class Cognition can read theInstance. */
  friend class Motion; /**< The class Motion can read theInstance. */
  friend class Framework; /**< The class Framework can set theInstance. */
  friend class CognitionLogger; /**< The cogniton logger needs to read theInstance */
};
