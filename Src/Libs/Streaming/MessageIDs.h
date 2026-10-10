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
  idMotorParameters,
  idStopwatch,
  idDepthImage,
  idJPEGDepthImage,

  // Data message IDs not used in RobotConsole. They can change over time.
  idAgentStates,
  idAlternativeRobotPoseHypothesis,
  idArmMotionRequest,
  idAudioData,
  idBallModel,
  idBallPercept,
  idBallSpots,
  idBehaviorStatus,
  idBodyContour,
  idCalibrationRequest,
  idCameraCalibration,
  idCameraCalibrationStatus,
  idCameraInfo,
  idCameraMatrix,
  idCirclePercept,
  idCustomCalibrationLines,
  idEmergencyButtonState,
  idFallDownState,
  idFieldBoundary,
  idFrameInfo,
  idGameControllerData,
  idGameState,
  idGlobalOpponentsModel,
  idGlobalTeammatesModel,
  idGroundTruthOdometryData,
  idGroundTruthWorldState,
  idGyroOffset,
  idHeadMotionInfo,
  idHeadMotionRequest,
  idIMUCalibration,
  idIMUValueState,
  idIndirectKick,
  idInertialData,
  idJointAnglePred,
  idJointAngles,
  idJointLimits,
  idJoystickState,
  idKeyStates,
  idLinesPercept,
  idMotionInfo,
  idMotionRobotHealth,
  idObstacleModel,
  idRobotPercept,
  idOdometer,
  idOdometryData,
  idOdometryDataPreview,
  idPenaltyMarkPercept,
  idReceivedTeamMessages,
  idRobotDimensions,
  idRobotHealth,
  idRobotPose,
  idSelfLocalizationHypotheses,
  idSideInformation,
  idSkillRequest,
  idStaticJointPoses,
  idStrategyStatus,
  idSystemSensorData,
  idTeamData,
  idTeamBallModel,
  idTorsoMatrix,
  idWalkGenerator,
  idWalkLearner,
  idWalkStepData,
  idWalkingEngineOutput,
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
