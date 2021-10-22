/**
 * @file AutonomousCameraCalibrationCard.cpp
 *
 *
 * @author Nele Matschull
 * @author Lukas Plecher
 */

#include "Platform/File.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/CameraCalibrationStatus.h"
#include "Representations/Configuration/CalibrationRequest.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/HeadMotionInfo.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"

CARD(AutonomousCameraCalibrationCard,
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
  DEFINES_PARAMETERS(
  {,
    (float)(0.7) walkSpeed,
    (float)(400) adjustmentRadius, /**< The maximum distance to the calibrationPose for adjustment, in mm. */
    (float)(1200) goalAreaCornerDistance, /**< The distance to the goal area corner in the first calibration phase, in mm.*/
    (float)(1200) penaltyMarkDistance, /**< The distance to the penalty mark in the second calibration phase, in mm. */
    (int)(5000) settleAfterWalkDuration, /**< The duration the robot will wait before recording samples after walking, in milliseconds. */
    (int)(1500) settleAfterHeadMovementDuration, /**< The duration the robot will wait before recording samples after turning its head, in milliseconds. */
    (int)(2500) waitDurationForSamplesWhenRecording, /**< The time we look for samples when recording before adjusting head or position, in milliseconds. */
    (Angle)(10_deg) maxHorizontalAdjustmentAngle, /**< The maximum additional head pan angle for adjustment. */
    (Angle)(5_deg) horizontalAdjustmentAngleStep, /**< The step size by which the head pan angle is increased. */
    (Angle)(5_deg) maxVerticalAdjustmentAngle, /**< The maximum additional head pan angle for adjustment. */
    (Angle)(5_deg) verticalAdjustmentAngleStep, /**< The step size by which the head pan angle is increased. */
    (std::vector<Angle>)({0.f, -0.75f, 0.75f}) headPans, /**< The head pan angles for recording samples. */
    (Angle)(120_deg) headSpeed, /**< The maximum speed with which the head moves between targets (deg/s). */
    (Angle)(0.38f) headTiltUpper, /**< The head tilt angle for recording samples with the upper camera. */
    (Angle)(-.25f) headTiltLower, /**< The head tilt angle for recording samples with the lower camera. */
    (Angle)(2_deg) badCalibrationThresholdX, /**< If the calculated x value is higher than this, the calibration is probably bad and should be done again. */
    (Angle)(5.5_deg) badCalibrationThresholdY, /**< If the calculated y value is higher than this, the calibration is probably bad and should be done again. */
  }),
});

class AutonomousCameraCalibrationCard : public AutonomousCameraCalibrationCardBase
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
    DECLARE_DEBUG_RESPONSE("behavior:AutonomousCameraCalibrationCard:finishNow");
    DECLARE_DEBUG_DRAWING("behavior:AutonomousCameraCalibrationCard:position", "drawingOnField");
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
    DECLARED_DEBUG_RESPONSE("behavior:AutonomousCameraCalibrationCard:finishNow")
      finishedCalibration = true;

    COMPLEX_DRAWING("behavior:AutonomousCameraCalibrationCard:position")
    {
      // Draw all calibration poses in orange.
      for(auto it = sampleConfigurations.begin(); it != sampleConfigurations.end(); it++)
      {
        const auto& position = it->calibrationPose.translation;
        const auto& target = it->lookTarget;
        ARROW("behavior:AutonomousCameraCalibrationCard:position", position.x(), position.y(), target.x(), target.y(), 10, Drawings::PenStyle::solidPen, ColorRGBA::orange);
      }
      // Draw the current calibration pose in blue.
      if(currentSampleConfiguration != sampleConfigurations.end())
      {
        const auto& position = currentSampleConfiguration->calibrationPose.translation;
        const auto& target = currentSampleConfiguration->lookTarget;
        ARROW("behavior:AutonomousCameraCalibrationCard:position", position.x(), position.y(), target.x(), target.y(), 10, Drawings::PenStyle::solidPen, ColorRGBA::blue);
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
        for(auto& headPan : headPans)
        {
          addSampleConfiguration(index++, firstPosition, firstLookTarget, CameraInfo::upper, headPan, bit(cornerAngle) | bit(parallelAngle) | bit(parallelLinesDistance));
          addSampleConfiguration(index++, firstPosition, firstLookTarget, CameraInfo::lower, headPan, bit(cornerAngle) | bit(parallelAngle) | bit(parallelLinesDistance));
        }

        /*for(auto& headPan : headPans)
        {
          addSampleConfiguration(index++, secondPosition, secondLookTarget, CameraInfo::upper, headPan, bit(goalAreaDistance) | bit(groundLineDistance) | bit(parallelAngle));
          addSampleConfiguration(index++, secondPosition, secondLookTarget, CameraInfo::lower, headPan, bit(goalAreaDistance) | bit(groundLineDistance) | bit(parallelAngle));
        }*/

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
            // Use the head pan from the previous sample configuration, in case we had to adjust the head angle to record.
            currentSampleConfiguration->request.headPan = lastSampleConfiguration->request.headPan;
            theStandSkill();
          }
          else
          {
            theTurnAngleSkill(-(currentSampleConfiguration->request.headPan - lastSampleConfiguration->request.headPan), 0_deg);
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

    state(adjustHeadAngle)
    {
      transition
      {
        const auto& status = theCameraCalibrationStatus.sampleConfigurationStatus;
        if(status == SampleConfigurationStatus::visible || status == SampleConfigurationStatus::finished)
        {
          currentSampleConfiguration->request.headPan = theJointAngles.angles[Joints::headYaw];
          currentSampleConfiguration->request.headTilt = theJointAngles.angles[Joints::headPitch];
          origHeadOrientation.reset();
          goto waitAndSettleAfterHeadMoved;
        }
        if(thePanAndTiltGridSkill.isDone())
        {
          origHeadOrientation.reset();
          goto adjustCalibrationPose;
        }
      }
      action
      {
        if(!origHeadOrientation)
        {
          origHeadOrientation = std::make_unique<HeadOrientation>();
          origHeadOrientation->pan = currentSampleConfiguration->request.headPan;
          origHeadOrientation->tilt = currentSampleConfiguration->request.headTilt;
        }
        HeadOrientation max;
        max.pan = maxHorizontalAdjustmentAngle;
        max.tilt = maxVerticalAdjustmentAngle;

        if(!theHeadMotionInfo.moving)
          checkIfCurrentConfigurationCanRecord();
        thePanAndTiltGridSkill(*origHeadOrientation, max, horizontalAdjustmentAngleStep, verticalAdjustmentAngleStep, settleAfterHeadMovementDuration, headSpeed);
        theStandSkill();
      }
    }

    state(adjustCalibrationPose)
    {
      transition
      {
        const auto& status = theCameraCalibrationStatus.sampleConfigurationStatus;
        if(status == SampleConfigurationStatus::visible || status == SampleConfigurationStatus::finished || (adjustmentTarget && theWalkToPointSkill.isDone()))
        {
          // Try to record the samples now.
          adjustmentTarget.reset();
          goto waitAndSettleAfterWalking;
        }
      }
      action
      {
        if(!adjustmentTarget)
        {
          // Random position in adjustmentRadius.
          float r = Random::uniform(0.f, adjustmentRadius);
          float t = Random::uniform(0.f, pi2);
          Vector2f pos = Vector2f::polar(r, t);
          theFieldDimensions.clipToField(pos);

          auto pose = poseTowards(
                        currentSampleConfiguration->calibrationPose.translation + pos,
                        currentSampleConfiguration->lookTarget);
          pose = pose.rotate(-currentSampleConfiguration->request.headPan);

          adjustmentTarget = std::make_unique<Pose2f>(std::move(pose));
        }

        COMPLEX_DRAWING("behavior:AutonomousCameraCalibrationCard:position")
        {
          // Draw the adjustmentTarget.
          const auto& position = adjustmentTarget->translation;
          const auto& target = currentSampleConfiguration->lookTarget;
          ARROW("behavior:AutonomousCameraCalibrationCard:position", position.x(), position.y(), target.x(), target.y(), 10, Drawings::PenStyle::solidPen, ColorRGBA::red);
        }

        theSaySkill("Adjusting position.");
        setHeadMotionRequest(currentSampleConfiguration->request);
        theWalkToPointSkill(theRobotPose.inversePose * *adjustmentTarget, walkSpeed, false, true);
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
        if(status == SampleConfigurationStatus::finished)
        {
          if(hasNextSampleConfiguration())
          {
            resetSampleConfigurationRequest();
            OUTPUT_TEXT("Finished recording current configuration.");
            nextSampleConfiguration();
            goto walkToCalibrationPose;
          }
          else
          {
            goto optimize;
          }
        }
        if(status != SampleConfigurationStatus::recording && state_time > waitDurationForSamplesWhenRecording)
        {
          currentSampleConfiguration->request.doRecord = false;
          goto adjustHeadAngle;
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
          if(std::abs(theCameraCalibration.cameraRotationCorrections[0][0]) > badCalibrationThresholdX
             || std::abs(theCameraCalibration.cameraRotationCorrections[0][1]) > badCalibrationThresholdY
             || std::abs(theCameraCalibration.cameraRotationCorrections[1][0]) > badCalibrationThresholdX
             || std::abs(theCameraCalibration.cameraRotationCorrections[1][1]) > badCalibrationThresholdY)
          {
            OUTPUT_TEXT("The resulting values are very high. A recalibration is suggested.");
          }
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

  void addSampleConfiguration(unsigned index, const Vector2f& position, const Vector2f& lookTarget, CameraInfo::Camera camera, const Angle& headPan, unsigned sampleTypes)
  {
    Angle headTilt = (camera == CameraInfo::Camera::upper) ? headTiltUpper : headTiltLower;
    SampleConfigurationRequest request;
    request.index = index;
    request.camera = camera;
    request.headPan = headPan;
    request.headTilt = headTilt;
    request.sampleTypes = sampleTypes;

    auto pose = poseTowards(position, lookTarget);
    pose = pose.rotate(-headPan);

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
    OUTPUT_TEXT("Prepared sample configuration: " << currentSampleConfiguration->request.index << " (for " << TypeRegistry::getEnumName(currentSampleConfiguration->request.camera) << ")");
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

  static Pose2f poseTowards(const Vector2f& position, const Vector2f& target)
  {
    const Angle angle = (target - position).angle();
    return Pose2f(angle, position);
  }
};

MAKE_CARD(AutonomousCameraCalibrationCard);
