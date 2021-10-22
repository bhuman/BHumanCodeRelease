/**
 * @file ExpAutonomousCameraCalibrationCard.cpp
 *
 *
 * @author Nele Matschull
 * @author Lukas Plecher
 */

#include "Platform/File.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/CameraCalibrationStatus.h"
#include "Representations/Configuration/CameraIntrinsics.h"
#include "Representations/Configuration/CalibrationRequest.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/HeadMotionInfo.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"

CARD(ExpAutonomousCameraCalibrationCard,
{,
  CALLS(Activity),
  CALLS(CalibrateRobot),
  CALLS(LookAtAngles),
  CALLS(LookAtPoint),
  CALLS(LookForward),
  CALLS(PanAndTiltGrid),
  CALLS(Say),
  CALLS(Stand),
  CALLS(TurnAngle),
  CALLS(WalkToPoint),
  REQUIRES(FieldDimensions),
  REQUIRES(HeadMotionInfo),
  REQUIRES(JointAngles),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotPose),
  USES(CameraCalibration),
  USES(CameraCalibrationStatus),
  USES(CameraIntrinsics),
  DEFINES_PARAMETERS(
  {,
    (float)(0.7) walkSpeed,
    (float)(400) adjustmentRadius, /**< The maximum distance to the calibrationPose for adjustment, in mm. */
    (float)(1200) goalAreaCornerDistance, /**< The distance to the goal area corner in the first calibration phase, in mm.*/
    (float)(1200) penaltyMarkDistance, /**< The distance to the penalty mark in the second calibration phase, in mm. */
    (int)(5000) settleAfterWalkDuration, /**< The duration the robot will wait before recording samples after walking, in milliseconds. */
    (int)(1000) settleAfterHeadMovementDuration, /**< The duration the robot will wait before recording samples after turning its head, in milliseconds. */
    (int)(2500) waitDurationForSamplesWhenRecording, /**< The time we look for samples when recording before adjusting head or position, in milliseconds. */
    (Angle)(10_deg) maxHorizontalAdjustmentAngle, /**< The maximum additional head pan angle for adjustment. */
    (Angle)(5_deg) horizontalAdjustmentAngleStep, /**< The step size by which the head pan angle is increased. */
    (Angle)(5_deg) maxVerticalAdjustmentAngle, /**< The maximum additional head pan angle for adjustment. */
    (Angle)(5_deg) verticalAdjustmentAngleStep, /**< The step size by which the head pan angle is increased. */
    (Angle)(120_deg) headSpeed, /**< The maximum speed with which the head moves between targets (deg/s). */
    (Angle)(0.38f) headTiltUpper, /**< The head tilt angle for recording samples with the upper camera. */
    (Angle)(-.25f) headTiltLower, /**< The head tilt angle for recording samples with the lower camera. */
  }),
});

class ExpAutonomousCameraCalibrationCard : public ExpAutonomousCameraCalibrationCardBase
{
  bool finishedCalibration = false;
  struct SampleConfigurationInfo
  {
    SampleConfigurationRequest request;
    Pose2f calibrationPose;
    Vector2f lookTarget;
  };

  unsigned totalNumOfSamples;
  std::vector<SampleConfigurationInfo> sampleConfigurations;
  std::vector<SampleConfigurationInfo>::iterator currentSampleConfiguration;
  SampleConfigurationInfo* lastSampleConfiguration;
  bool switchedToLower = false;

  std::unique_ptr<Pose2f> adjustmentTarget;
  std::unique_ptr<HeadOrientation> origHeadOrientation;

  CalibrationRequest theCalibrationRequest;

  void preProcess() override
  {
    DECLARE_DEBUG_RESPONSE("behavior:ExpAutonomousCameraCalibrationCard:finishNow");
    DECLARE_DEBUG_DRAWING("behavior:ExpAutonomousCameraCalibrationCard:position", "drawingOnField");
  }

  bool preconditions() const override
  {
    return !finishedCalibration;
  }

  bool postconditions() const override
  {
    return finishedCalibration;
  }

  option
  {
    DECLARED_DEBUG_RESPONSE("behavior:ExpAutonomousCameraCalibrationCard:finishNow")
      finishedCalibration = true;

    COMPLEX_DRAWING("behavior:ExpAutonomousCameraCalibrationCard:position")
    {
      // Draw all calibration poses in orange.
      for(auto it = sampleConfigurations.begin(); it != sampleConfigurations.end(); it++)
      {
        const auto& position = it->calibrationPose.translation;
        const auto& target = it->lookTarget;
        ARROW("behavior:ExpAutonomousCameraCalibrationCard:position", position.x(), position.y(), target.x(), target.y(), 10, Drawings::PenStyle::solidPen, ColorRGBA::orange);
      }
      // Draw the current calibration pose in blue.
      if(currentSampleConfiguration != sampleConfigurations.end())
      {
        const auto& position = currentSampleConfiguration->calibrationPose.translation;
        const auto& target = currentSampleConfiguration->lookTarget;
        ARROW("behavior:ExpAutonomousCameraCalibrationCard:position", position.x(), position.y(), target.x(), target.y(), 10, Drawings::PenStyle::solidPen, ColorRGBA::blue);
      }
    }

    resetSampleConfigurationRequest();

    initial_state(initialize)
    {
      transition
      {
        if(state_time || !sampleConfigurations.empty()) goto walkToCalibrationPose;
      }
      action
      {
        OUTPUT_TEXT("Initializing sample configurations");
        sampleConfigurations.clear();
        adjustmentTarget.reset();
        totalNumOfSamples = 0;

        const Vector2f goalAreaShortLineMiddle = theRobotPose.translation.y() > 0
                                                 ? Vector2f(theFieldDimensions.xPosOwnGroundLine + 0.5f * (theFieldDimensions.xPosOwnGoalArea - theFieldDimensions.xPosOwnGroundLine), theFieldDimensions.yPosLeftGoalArea)
                                                 : Vector2f(theFieldDimensions.xPosOwnGroundLine + 0.5f * (theFieldDimensions.xPosOwnGoalArea - theFieldDimensions.xPosOwnGroundLine), theFieldDimensions.yPosRightGoalArea);

        const Vector2f awayFromGoalAreaCorner = theRobotPose.translation.y() > 0 ? Vector2f(4, 5) : Vector2f(4, -5);
        const Vector2f ownGroundlineCenter(theFieldDimensions.xPosOwnGroundLine, 0);

        const Vector2f firstLookTarget = goalAreaShortLineMiddle;
        Vector2f firstPosition = goalAreaShortLineMiddle + awayFromGoalAreaCorner.normalized() * goalAreaCornerDistance;
        theFieldDimensions.clipToField(firstPosition);

        const Vector2f secondPosition(theFieldDimensions.xPosOwnPenaltyMark + penaltyMarkDistance, 0);
        const Vector2f secondLookTarget(theFieldDimensions.xPosOwnPenaltyMark, 0);

        int index = 0;

        const auto minHeadYaw = -15_deg;
        const auto maxHeadYaw = +15_deg;
        const auto minHeadPitch = -15_deg;
        const auto maxHeadPitch = +15_deg;
        const auto headMoveStep = 10_deg;

        const std::vector<Angle> poseRotations = { 0_deg, -40_deg, +40_deg };
        for(const auto& poseRotation : poseRotations)
        {
          for(auto currentHeadPitch = minHeadPitch; currentHeadPitch <= maxHeadPitch; currentHeadPitch += headMoveStep)
            for(auto currentHeadYaw = minHeadYaw; currentHeadYaw <= maxHeadYaw; currentHeadYaw += headMoveStep)
              addSampleConfiguration(index++, firstPosition, firstLookTarget, poseRotation, CameraInfo::upper, currentHeadYaw, currentHeadPitch, bit(cornerAngle) | bit(parallelAngle) | bit(parallelLinesDistance));

          for(auto currentHeadPitch = minHeadPitch; currentHeadPitch <= maxHeadPitch; currentHeadPitch += headMoveStep)
            for(auto currentHeadYaw = minHeadYaw; currentHeadYaw <= maxHeadYaw; currentHeadYaw += headMoveStep)
              addSampleConfiguration(index++, firstPosition, firstLookTarget, poseRotation, CameraInfo::lower, currentHeadYaw, currentHeadPitch, bit(cornerAngle) | bit(parallelAngle) | bit(parallelLinesDistance));
        }
        ASSERT(!sampleConfigurations.empty());
        currentSampleConfiguration = sampleConfigurations.begin();
        lastSampleConfiguration = nullptr;

        theCalibrationRequest.totalNumOfSamples = totalNumOfSamples;
        theCalibrationRequest.targetState = State::recordSamples;

        theLookForwardSkill();
        theStandSkill();
      }
    }

    state(walkToCalibrationPose)
    {
      transition
      {
        if(switchedToLower)
        {
          switchedToLower = false;
          // Only the head was moved, so we do not need to wait that long.
          goto waitAndSettleAfterHeadMoved;
        }
        if(theWalkToPointSkill.isDone() || theTurnAngleSkill.isDone())
        {
          goto waitAndSettleAfterWalking;
        }
      }
      action
      {
        // Check if we need to walk a different position.
        if(lastSampleConfiguration && lastSampleConfiguration->calibrationPose.translation == currentSampleConfiguration->calibrationPose.translation)
        {
          setHeadMotionRequest(currentSampleConfiguration->request);
          // Check if we need to turn towards a different angle.
          if(lastSampleConfiguration->calibrationPose.rotation == currentSampleConfiguration->calibrationPose.rotation)
          {
            switchedToLower = true;
            theStandSkill();
          }
          else
          {
            theTurnAngleSkill(-(currentSampleConfiguration->calibrationPose.rotation - lastSampleConfiguration->calibrationPose.rotation), 0_deg);
          }
        }
        else
        {
          const Pose2f relativeCalibrationPose = theRobotPose.inversePose * currentSampleConfiguration->calibrationPose;
          theWalkToPointSkill(relativeCalibrationPose, walkSpeed, false, true);

          const auto relativeTarget = theRobotPose.inversePose * currentSampleConfiguration->lookTarget;
          theLookAtPointSkill(Vector3f(relativeTarget.x(), relativeTarget.y(), 0.f), HeadMotionRequest::upperCamera);
        }
      }
    }

    state(waitAndSettleAfterWalking)
    {
      transition
      {
        if(state_time > settleAfterWalkDuration)
          goto recordSamples;
      }
      action
      {
        checkIfCurrentConfigurationCanRecord();
        setHeadMotionRequest(currentSampleConfiguration->request);
        theStandSkill();
      }
    }

    state(waitAndSettleAfterHeadMoved)
    {
      transition
      {
        if(state_time > settleAfterHeadMovementDuration)
          goto recordSamples;
      }
      action
      {
        checkIfCurrentConfigurationCanRecord();
        setHeadMotionRequest(currentSampleConfiguration->request);
        theStandSkill();
      }
    }

    state(recordSamples)
    {
      transition
      {
        const auto& status = theCameraCalibrationStatus.sampleConfigurationStatus;
        if(status == SampleConfigurationStatus::finished || (status != SampleConfigurationStatus::recording && state_time > waitDurationForSamplesWhenRecording))
        {
          if(status == SampleConfigurationStatus::finished)
            OUTPUT_TEXT("+ Recorded samples for configuration.");
          else
            OUTPUT_TEXT("- Skipping configuration.");

          if(hasNextSampleConfiguration())
          {
            resetSampleConfigurationRequest();
            nextSampleConfiguration();
            goto walkToCalibrationPose;
          }
          else
          {
            goto optimize;
          }
        }
      }
      action
      {
        recordCurrentConfiguration();
        setHeadMotionRequest(currentSampleConfiguration->request);
        theStandSkill();
      }
    }

    state(optimize)
    {
      transition
      {
        // Wait for the optimizer to finish.
        if(theCameraCalibrationStatus.state == State::idle && state_time > 100)
        {
          // Save the camera calibration.
          saveCameraCalibration(theCameraCalibration);
          saveCameraInstrinsics(theCameraIntrinsics);
          goto finished;
        }
      }
      action
      {
        theCalibrationRequest.targetState = State::optimize;

        theSaySkill("Optimizing.");
        theLookAtAnglesSkill(0.f, 0.f);
        theStandSkill();
      }
    }

    state(finished)
    {
      transition
      {
      }
      action
      {
        theCalibrationRequest.targetState = State::idle;
        // TODO: reset calibrator completely.

        finishedCalibration = true;
        theLookForwardSkill();
        theStandSkill();
      }
    }

    // FIXME: this is a bit hacky.
    theCalibrateRobotSkill(theCalibrationRequest);
    theActivitySkill(BehaviorStatus::unknown);
  }

  void addSampleConfiguration(unsigned index, const Vector2f& position, const Vector2f& lookTarget, const Angle& poseRotation, CameraInfo::Camera camera, const Angle& headYaw, const Angle& headPitch, unsigned sampleTypes)
  {
    Angle defaultHeadPitch = (camera == CameraInfo::Camera::upper) ? headTiltUpper : headTiltLower;
    SampleConfigurationRequest request;
    request.index = index;
    request.camera = camera;
    request.headPan = poseRotation - headYaw;
    request.headTilt = defaultHeadPitch + headPitch;
    request.sampleTypes = sampleTypes;

    auto pose = poseTowards(position, lookTarget);
    pose.rotate(poseRotation);

    SampleConfigurationInfo config = { request, pose, lookTarget };
    sampleConfigurations.push_back(config);
    FOREACH_ENUM(SampleType, sampleType)
      if(sampleTypes & bit(sampleType))
        ++totalNumOfSamples;
  }

  void recordCurrentConfiguration()
  {
    currentSampleConfiguration->request.doRecord = true;
    setSampleConfigurationRequest(currentSampleConfiguration->request);
  }

  void checkIfCurrentConfigurationCanRecord()
  {
    currentSampleConfiguration->request.doRecord = false;
    setSampleConfigurationRequest(currentSampleConfiguration->request);
  }

  void setSampleConfigurationRequest(const SampleConfigurationRequest& request)
  {
    theCalibrationRequest.totalNumOfSamples = totalNumOfSamples;
    theCalibrationRequest.sampleConfigurationRequest = SampleConfigurationRequest(request);
  }

  void resetSampleConfigurationRequest()
  {
    theCalibrationRequest.sampleConfigurationRequest = std::optional<SampleConfigurationRequest> {};
  }

  bool hasNextSampleConfiguration()
  {
    return std::next(currentSampleConfiguration) != sampleConfigurations.end();
  }

  void nextSampleConfiguration()
  {
    lastSampleConfiguration = &*currentSampleConfiguration;
    currentSampleConfiguration++;
    OUTPUT_TEXT("Prepared configuration #" << currentSampleConfiguration->request.index << " (for " << TypeRegistry::getEnumName(currentSampleConfiguration->request.camera) << ")");
  }

  void setHeadMotionRequest(const SampleConfigurationRequest& request)
  {
    // Set the requested head angles.
    theLookAtAnglesSkill(request.headPan,
                         request.headTilt + theRobotDimensions.getTiltNeckToCamera(false),
                         headSpeed,
                         HeadMotionRequest::upperCamera);
  }

  static void saveCameraCalibration(const CameraCalibration& cameraCalibration)
  {
    std::string name = "cameraCalibration.cfg";
    for(std::string& fullName : File::getFullNames(name))
    {
      File path(fullName, "r", false);
      if(path.exists())
      {
        name = std::move(fullName);
        break;
      }
    }
    OutMapFile stream(name, true);
    stream << cameraCalibration;
  }

  static void saveCameraInstrinsics(const CameraIntrinsics& cameraIntrinsics)
  {
    std::string name = "cameraIntrinsics.cfg";
    for(std::string& fullName : File::getFullNames(name))
    {
      File path(fullName, "r", false);
      if(path.exists())
      {
        name = std::move(fullName);
        break;
      }
    }
    OutMapFile stream(name, true);
    stream << cameraIntrinsics;
  }

  static Pose2f poseTowards(const Vector2f& position, const Vector2f& target)
  {
    const Angle angle = (target - position).angle();
    return Pose2f(angle, position);
  }
};

MAKE_CARD(ExpAutonomousCameraCalibrationCard);
