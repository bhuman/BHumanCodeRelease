/**
 * @file MessageIDs.h
 *
 * Declaration of ids for debug messages.
 *
 * @author Martin Lötzsch
 */

#pragma once

#include "Streaming/Enum.h"

/**
 * IDs for debug messages
 *
 * To distinguish debug messages, they all have an id.
 */
ENUM(MessageID,
{,
  idNumOfDataMessageIDs,
  undefined = idNumOfDataMessageIDs,
  idFrameBegin,
  idFrameFinished,

  // Data message IDs used in RobotConsole. Keep them the same over time.
  idActivationGraph,
  idAnnotation,
  idCameraImage,
  idInertialSensorData,
  idJointCalibration,
  idJointRequest,
  idJointSensorData,
  idJPEGImage,
  idMotionRequest,
  idStopwatch,

  // Data message IDs not used in RobotConsole. They can change over time.
  idAlternativeRobotPoseHypothesis,
  idArmMotionRequest,
  idAudioData,
  idBallModel,
  idBallPercept,
  idBallSpots,
  idBehaviorStatus,
  idBodyContour,
  idCameraCalibration,
  idCameraInfo,
  idCameraMatrix,
  idCirclePercept,
  idFallDownState,
  idFieldBoundary,
  idFieldFeatureOverview,
  idFootOffset,
  idFootSupport,
  idFrameInfo,
  idFsrData,
  idFsrSensorData,
  idGameControllerData,
  idGameState,
  idGyroOffset,
  idGyroState,
  idGroundContactState,
  idGroundTruthOdometryData,
  idGroundTruthWorldState,
  idHeadMotionRequest,
  idIMUCalibration,
  idImageCoordinateSystem,
  idInertialData,
  idJointAnglePred,
  idJointAngles,
  idJointLimits,
  idJointPlay,
  idKeypoints,
  idKeyStates,
  idLinesPercept,
  idMotionInfo,
  idObstacleModel,
  idObstaclesFieldPercept,
  idObstaclesImagePercept,
  idOdometer,
  idOdometryData,
  idOdometryDataPreview,
  idOdometryTranslationRequest,
  idPenaltyMarkPercept,
  idReceivedTeamMessages,
  idRefereePercept,
  idRobotDimensions,
  idRobotHealth,
  idRobotPose,
  idRobotStableState,
  idSelfLocalizationHypotheses,
  idSideInformation,
  idSkillRequest,
  idStaticJointPoses,
  idStrategyStatus,
  idSystemSensorData,
  idTeammatesBallModel,
  idTeamData,
  idWalkGenerator,
  idWalkStepData,
  idWalkingEngineOutput,
  idWalkLearner,
  idWhistle,
  numOfDataMessageIDs, /**< everything below this does not belong into log files */

  // infrastructure
  idConsole = numOfDataMessageIDs,
  idDebugDataChangeRequest,
  idDebugDataResponse,
  idDebugDrawing,
  idDebugDrawing3D,
  idDebugImage,
  idDebugRequest,
  idDebugResponse,
  idDrawingManager,
  idDrawingManager3D,
  idLogResponse,
  idModuleRequest,
  idModuleTable,
  idPlot,
  idRobotName,
  idText,
  idThread,
  idTypeInfo,
  idTypeInfoRequest,
});
