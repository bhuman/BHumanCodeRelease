/**
 * @file AutonomousCameraCalibration.cpp
 * This skill defines the behavior to let the robot walk to predefined positions,
 * to sample images and calibrate the extrinsic camera parameters
 *
 * @author Nele Matschull
 * @author Lukas Plecher
 */

#include "SkillBehaviorControl.h"
#include "Debugging/DebugDrawings.h"
#include "Framework/Settings.h"
#include "Platform/File.h"
#include <filesystem>

namespace AutonomousCameraCalibration
{
  struct SampleConfigurationInfo
  {
    SampleConfigurationRequest request;
    Pose2f calibrationPose;
    Vector2f lookTarget;
  };
}
using namespace AutonomousCameraCalibration;

option((SkillBehaviorControl) AutonomousCameraCalibration,
       defs((Pose2f)(Pose2f(0.7f, 0.7f, 0.7f)) walkSpeed,
            (float)(300) adjustmentRadius, /**< The maximum distance to the calibrationPose for adjustment, in mm. */
            (float)(1500) goalAreaCornerDistance, /**< The distance to the goal area corner in the first calibration phase, in mm.*/
            //(float)(1200) penaltyMarkDistance, /**< The distance to the penalty mark in the second calibration phase, in mm. */
            (int)(7000) settleAfterWalkDuration, /**< The duration the robot will wait before recording samples after walking, in milliseconds. */
            (int)(3000) settleAfterHeadMovementDuration, /**< The duration the robot will wait before recording samples after turning its head, in milliseconds. */
            (int)(1500) panAndTiltGridWaitInPositionTime, /**< While searching for a good head position wait this long between the head movements. */
            (int)(4500) waitDurationForSamplesWhenRecording, /**< The time we look for samples when recording before adjusting head or position, in milliseconds. */
            (int)(2000) walkInPlaceDuration, /**< When stopping to walk, walk in place for this duration. */
            (int)(1000) notVisibleThreshold, /**< Assume not all lines are visible after this time in not visible state. */
            (int)(50) visibleThreshold, /**< Assume visibility of all lines after this time in visible state. */
            (Angle)(10_deg) maxHorizontalAdjustmentAngle, /**< The maximum additional head pan angle for adjustment. */
            (Angle)(9.9_deg) horizontalAdjustmentAngleStep, /**< The step size by which the head pan angle is increased. */
            (Angle)(10_deg) maxVerticalAdjustmentAngle, /**< The maximum additional head tilt angle for adjustment. */
            (Angle)(9.9_deg) verticalAdjustmentAngleStep, /**< The step size by which the head pitch angle is increased. */
            (std::vector<Angle>)({0.f, -0.9f, -2.07f, 0.9f, 2.07f }) headPans, /**< The head pan angles for recording samples. */
            (std::vector<Angle>)({0.34f, 0.26f, 0.26f, 0.26f, 0.26f}) upperHeadTilts,
            (std::vector<Angle>)({-0.25f, -0.34f, -0.33f, -0.34f, -0.33f}) lowerHeadTilts,
            (Angle)(50_deg) headSpeed, /**< The maximum speed with which the head moves between targets (deg/s). */
            (float)(100.f) standStillTranslationThreshold,
            (Angle)(6_deg) standStillOrientationThreshold,
            (Angle)(1.5_deg) turnAngleThreshold,
            (bool)(true) useGoalAreaConnectingLineDistance),
       vars((bool)(false) forceWalkToPoseAfterInterruption,
            (bool)(false) finishedCalibration,
            (unsigned)(0) totalNumOfSamples,
            (std::vector<SampleConfigurationInfo>) sampleConfigurations,
            (std::vector<SampleConfigurationInfo>::iterator) currentSampleConfiguration,
            (SampleConfigurationInfo*)(nullptr) lastSampleConfiguration,
            (bool)(false) switchedToLower,
            (std::unique_ptr<Pose2f>) adjustmentTarget,
            (Pose2f) currentWalkTarget,
            (std::unique_ptr<HeadOrientation>) origHeadOrientation,
            (CalibrationRequest) theCalibrationRequest))
{
  theCalibrationRequest.preciseJointPositions = true;
  const auto poseTowards = [&](const Vector2f& position, const Vector2f& target)
  {
    const Angle angle = (target - position).angle();
    return Pose2f(angle, position);
  };

  const auto addSampleConfiguration = [&](unsigned index, const Vector2f& position, const Vector2f& lookTarget, CameraInfo::Camera camera, const Angle& headPan, const Angle& headTilt, unsigned sampleTypes)
  {
    SampleConfigurationRequest request;
    request.index = index;
    request.camera = camera;
    request.headPan = headPan;
    request.orgHeadPan = headPan;
    request.headTilt = headTilt;
    request.sampleTypes = sampleTypes;

    auto pose = poseTowards(position, lookTarget);
    pose = pose.rotate(-headPan);

    SampleConfigurationInfo config = { request, pose, lookTarget };
    sampleConfigurations.push_back(config);
    FOREACH_ENUM(SampleType, sampleType)
      if(sampleTypes & bit(sampleType))
        ++totalNumOfSamples;
  };

  const auto setSampleConfigurationRequest = [&](const SampleConfigurationRequest& request)
  {
    theCalibrationRequest.totalNumOfSamples = totalNumOfSamples;
    theCalibrationRequest.sampleConfigurationRequest = SampleConfigurationRequest(request);
    CalibrateRobot({.request = theCalibrationRequest});
  };

  const auto recordCurrentConfiguration = [&]
  {
    currentSampleConfiguration->request.doRecord = true;
    setSampleConfigurationRequest(currentSampleConfiguration->request);
  };

  const auto checkIfCurrentConfigurationCanRecord = [&]
  {
    currentSampleConfiguration->request.doRecord = false;
    setSampleConfigurationRequest(currentSampleConfiguration->request);
  };

  const auto resetSampleConfigurationRequest = [&]
  {
    theCalibrationRequest.sampleConfigurationRequest = std::optional<SampleConfigurationRequest> {};
    CalibrateRobot({.request = theCalibrationRequest});
  };

  const auto hasNextSampleConfiguration = [&]
  {
    return std::next(currentSampleConfiguration) != sampleConfigurations.end();
  };

  const auto nextSampleConfiguration = [&]
  {
    lastSampleConfiguration = &* currentSampleConfiguration;
    currentSampleConfiguration++;
    OUTPUT_TEXT("Prepared sample configuration: " << currentSampleConfiguration->request.index << " (for " << TypeRegistry::getEnumName(currentSampleConfiguration->request.camera) << ")");
  };

  const auto setHeadMotionRequest = [&](const SampleConfigurationRequest& request)
  {
    // Set the requested head angles.
    LookAtAngles({.pan = request.headPan,
                  .tilt = request.headTilt + theRobotDimensions.getTiltNeckToCamera(false),
                  .speed = headSpeed,
                  .camera = HeadMotionRequest::upperCamera});
  };

  const auto saveCameraCalibration = [&](const CameraCalibration& cameraCalibration)
  {
    const std::string path = std::string(File::getBHDir()) + "/Config/Robots/" + Global::getSettings().headName + "/" + Global::getSettings().bodyName;
    std::filesystem::create_directories(path);
    OutMapFile stream(path + "/cameraCalibration.cfg", true);
    stream << cameraCalibration;
  };

  DECLARED_DEBUG_RESPONSE("option:AutonomousCameraCalibration:finishNow")
    finishedCalibration = true;

  COMPLEX_DRAWING("option:AutonomousCameraCalibration:position")
  {
    // Draw all calibration poses in orange.
    for(auto it = sampleConfigurations.begin(); it != sampleConfigurations.end(); it++)
    {
      const auto& position = it->calibrationPose.translation;
      const auto& target = it->lookTarget;
      ARROW("option:AutonomousCameraCalibration:position", position.x(), position.y(), target.x(), target.y(), 10, Drawings::PenStyle::solidPen, ColorRGBA::orange);
    }
    // Draw the current calibration pose in blue.
    if(sampleConfigurations.size() > 0 && currentSampleConfiguration != sampleConfigurations.end())
    {
      const auto& position = currentSampleConfiguration->calibrationPose.translation;
      const auto& target = currentSampleConfiguration->lookTarget;
      ARROW("option:AutonomousCameraCalibration:position", position.x(), position.y(), target.x(), target.y(), 10, Drawings::PenStyle::solidPen, ColorRGBA::blue);
    }
  }

  resetSampleConfigurationRequest();

  common_transition
  {
    if(finishedCalibration)
      goto finished;
  }

  initial_state(initialize)
  {
    transition
    {
      if(!sampleConfigurations.empty())
      {
        for(std::size_t i = 0; i < theCameraCalibrationStatus.sampleIndex; i++)
          nextSampleConfiguration();
        forceWalkToPoseAfterInterruption = true;
        goto walkToCalibrationPose;
      }
    }
    action
    {
      OUTPUT_TEXT("Initializing sample configurations");
      sampleConfigurations.clear();
      adjustmentTarget.reset();
      totalNumOfSamples = 0;

      // For head control
      const Vector2f firstLookTarget = useGoalAreaConnectingLineDistance ?
                                       Vector2f(theFieldDimensions.xPosOwnGoal, 0.f) : // look at middle of goal to see all 4 goal area lines
                                       (theRobotPose.translation.y() > 0
                                        ? Vector2f(theFieldDimensions.xPosOwnGoalArea, theFieldDimensions.yPosLeftGoalArea)
                                        : Vector2f(theFieldDimensions.xPosOwnGoalArea, theFieldDimensions.yPosRightGoalArea));

      // For positioning
      const Vector2f goalAreaShortLineMiddle = theRobotPose.translation.y() > 0
                                               ? Vector2f(theFieldDimensions.xPosOwnGoalArea, theFieldDimensions.yPosLeftGoalArea)
                                               : Vector2f(theFieldDimensions.xPosOwnGoalArea, theFieldDimensions.yPosRightGoalArea);

      const Vector2f awayFromGoalAreaCorner = theRobotPose.translation.y() > 0 ? Vector2f(1, 1) : Vector2f(1, -1);

      Vector2f firstPosition = goalAreaShortLineMiddle + awayFromGoalAreaCorner.normalized() * goalAreaCornerDistance;
      theFieldDimensions.clipToField(firstPosition);

      //const Vector2f secondPosition(theFieldDimensions.xPosOwnPenaltyMark + penaltyMarkDistance, 0);
      //const Vector2f secondLookTarget(theFieldDimensions.xPosOwnPenaltyMark, 0);

      int index = 0;
      std::size_t cameraIndex = 0;
      ASSERT(upperHeadTilts.size() == lowerHeadTilts.size());
      ASSERT(upperHeadTilts.size() == headPans.size());
      unsigned sampleTypes = bit(cornerAngle) | bit(parallelAngle) | bit(parallelLinesDistance);
      if(useGoalAreaConnectingLineDistance)
        sampleTypes |= bit(parallelLinesDistanceShort);
      for(auto& headPan : headPans)
      {
        ASSERT(cameraIndex < upperHeadTilts.size());
        addSampleConfiguration(index++, firstPosition, firstLookTarget, CameraInfo::upper, headPan, upperHeadTilts[cameraIndex], sampleTypes);
        addSampleConfiguration(index++, firstPosition, firstLookTarget, CameraInfo::lower, headPan, lowerHeadTilts[cameraIndex++], sampleTypes);
      }

      ASSERT(!sampleConfigurations.empty());
      currentSampleConfiguration = sampleConfigurations.begin();
      lastSampleConfiguration = &*currentSampleConfiguration;

      theCalibrationRequest.totalNumOfSamples = totalNumOfSamples;
      theCalibrationRequest.targetState = CameraCalibrationStatus::State::recordSamples;

      CalibrateRobot({.request = theCalibrationRequest});
      LookForward();
      Stand();
    }
  }

  state(walkToCalibrationPose)
  {
    transition
    {
      if(switchedToLower)
      {
        forceWalkToPoseAfterInterruption = false;
        switchedToLower = false;
        // Only the head was moved, so we do not need to wait that long.
        goto waitAndSettleAfterHeadMoved;
      }

      const Pose2f relativeCalibrationPose = theRobotPose.inverse() * currentSampleConfiguration->calibrationPose;
      if(action_done || (relativeCalibrationPose.translation.squaredNorm() < sqr(standStillTranslationThreshold) && std::abs(relativeCalibrationPose.rotation) < standStillOrientationThreshold))
      {
        forceWalkToPoseAfterInterruption = false;
        goto walkInPlace;
      }
    }
    action
    {
      // Check if we need to walk a different position.
      if(!forceWalkToPoseAfterInterruption && lastSampleConfiguration && lastSampleConfiguration->calibrationPose.translation == currentSampleConfiguration->calibrationPose.translation)
      {
        setHeadMotionRequest(currentSampleConfiguration->request);
        // Check if we need to turn towards a different angle.
        if(lastSampleConfiguration->calibrationPose.rotation == currentSampleConfiguration->calibrationPose.rotation)
        {
          switchedToLower = true;
          // Use the head pan from the previous sample configuration, in case we had to adjust the head angle to record.
          currentSampleConfiguration->request.headPan = lastSampleConfiguration->request.headPan;
          currentSampleConfiguration->request.orgHeadPan = lastSampleConfiguration->request.orgHeadPan;
          Stand();
        }
        else
        {
          TurnAngle({.angle = lastSampleConfiguration->request.headPan - currentSampleConfiguration->request.headPan,
                     .margin = turnAngleThreshold,
                     .reduceWalkSpeedType = ReduceWalkSpeedType::slow});
        }
      }
      else
      {
        const auto relativeTarget = theRobotPose.inverse() * currentSampleConfiguration->lookTarget;
        LookAtPoint({.target = {relativeTarget.x(), relativeTarget.y(), 0.f},
                     .camera = HeadMotionRequest::upperCamera});

        currentWalkTarget = currentSampleConfiguration->calibrationPose;
        const Pose2f relativeCalibrationPose = theRobotPose.inverse() * currentSampleConfiguration->calibrationPose;
        WalkToPoint({.target = relativeCalibrationPose,
                     .speed = walkSpeed,
                     .disableObstacleAvoidance = true});

        forceWalkToPoseAfterInterruption &= std::abs(relativeCalibrationPose.translation.x()) > standStillTranslationThreshold ||
                                            std::abs(relativeCalibrationPose.translation.y()) > standStillTranslationThreshold ||
                                            std::abs(relativeCalibrationPose.rotation) > standStillOrientationThreshold;
      }
    }
  }

  state(adjustHeadAngle)
  {
    transition
    {
      const auto& status = theCameraCalibrationStatus.sampleConfigurationStatus;
      if((status == SampleConfigurationStatus::visible && theFrameInfo.getTimeSince(theCameraCalibrationStatus.inStatusSince) > visibleThreshold) || status == SampleConfigurationStatus::finished)
      {
        currentSampleConfiguration->request.headPan = theJointAngles.angles[Joints::headYaw];
        currentSampleConfiguration->request.headTilt = theJointAngles.angles[Joints::headPitch];
        origHeadOrientation.reset();
        goto waitAndSettleAfterHeadMoved;
      }
      if(action_done)
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
      Stand();
      PanAndTiltGrid({.original = *origHeadOrientation,
                      .maximum = max,
                      .panStep = horizontalAdjustmentAngleStep,
                      .tiltStep = verticalAdjustmentAngleStep,
                      .waitInPosition = panAndTiltGridWaitInPositionTime,
                      .speed = headSpeed});
    }
  }

  state(adjustCalibrationPose)
  {
    transition
    {
      const auto& status = theCameraCalibrationStatus.sampleConfigurationStatus;
      bool walkIsDone = false;
      if(adjustmentTarget)
      {
        const auto poseTarget = theRobotPose.inverse() * *adjustmentTarget;
        walkIsDone = poseTarget.translation.squaredNorm() < sqr(standStillTranslationThreshold) && std::abs(poseTarget.rotation) < standStillOrientationThreshold;
      }

      if(status == SampleConfigurationStatus::finished || (adjustmentTarget && (action_done || walkIsDone)))
      {
        // Try to record the samples now.
        adjustmentTarget.reset();
        goto walkInPlace;
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

        auto pose = poseTowards(
                      currentSampleConfiguration->calibrationPose.translation + pos,
                      currentSampleConfiguration->lookTarget);

        theFieldDimensions.clipToField(pose.translation);
        ASSERT(lastSampleConfiguration != nullptr);
        pose = pose.rotate(-currentSampleConfiguration->request.orgHeadPan);

        adjustmentTarget = std::make_unique<Pose2f>(std::move(pose));

        if(useGoalAreaConnectingLineDistance) // use RobotPose to look look at the middle of the goal
        {
          const Pose2f fieldPose = *adjustmentTarget;
          const Angle targetAngle = (Vector2f(theFieldDimensions.xPosOwnGoal, 0.f) - fieldPose.translation).angle();
          adjustmentTarget->rotation = targetAngle - currentSampleConfiguration->request.orgHeadPan;
        }
      }

      COMPLEX_DRAWING("option:AutonomousCameraCalibration:position")
      {
        // Draw the adjustmentTarget.
        const auto& position = adjustmentTarget->translation;
        const auto& target = currentSampleConfiguration->lookTarget;
        ARROW("option:AutonomousCameraCalibration:position", position.x(), position.y(), target.x(), target.y(), 10, Drawings::PenStyle::solidPen, ColorRGBA::red);
      }

      Say({.text = "Adjusting position."});

      setHeadMotionRequest(currentSampleConfiguration->request);
      WalkToPoint({.target = theRobotPose.inverse() * *adjustmentTarget,
                   .speed = walkSpeed,
                   .disableObstacleAvoidance = true});
    }
  }

  state(walkInPlace)
  {
    transition
    {
      if(state_time > walkInPlaceDuration)
        goto waitAndSettleAfterWalking;
    }
    action
    {
      if(state_time)
        currentWalkTarget = theRobotPose;
      const Angle target = (theRobotPose.inverse() * currentWalkTarget).rotation;
      WalkAtAbsoluteSpeed({.speed = {target, 0.01f, 0.f }});
      setHeadMotionRequest(currentSampleConfiguration->request);
    }
  }

  state(waitAndSettleAfterWalking)
  {
    transition
    {
      if(state_time > settleAfterWalkDuration &&
         theFallDownState.state != FallDownState::pickedUp &&
         theFallDownState.timestampSinceStateSwitch != theFrameInfo.time &&
         theFrameInfo.getTimeSince(theFallDownState.timestampSinceStateSwitch) > settleAfterWalkDuration)
        goto recordSamples;
    }
    action
    {
      checkIfCurrentConfigurationCanRecord();
      setHeadMotionRequest(currentSampleConfiguration->request);
      Stand();
    }
  }

  state(waitAndSettleAfterHeadMoved)
  {
    transition
    {
      if(state_time > settleAfterHeadMovementDuration &&
         theFallDownState.state != FallDownState::pickedUp &&
         theFallDownState.timestampSinceStateSwitch != theFrameInfo.time &&
         theFrameInfo.getTimeSince(theFallDownState.timestampSinceStateSwitch) > settleAfterHeadMovementDuration &&
         theCameraCalibrationStatus.sampleConfigurationStatus != SampleConfigurationStatus::waiting)
        goto recordSamples;
    }
    action
    {
      checkIfCurrentConfigurationCanRecord();
      setHeadMotionRequest(currentSampleConfiguration->request);
      Stand();
    }
  }

  state(recordSamples)
  {
    transition
    {
      const auto& status = theCameraCalibrationStatus.sampleConfigurationStatus;
      if(status == SampleConfigurationStatus::waiting && theFallDownState.state != FallDownState::pickedUp)
        goto waitAndSettleAfterHeadMoved;
      if(status == SampleConfigurationStatus::finished)
      {
        if(hasNextSampleConfiguration())
        {
          OUTPUT_TEXT("Finished recording current configuration.");
          nextSampleConfiguration();
          goto walkToCalibrationPose;
        }
        else
        {
          goto optimize;
        }
      }
      if(theFallDownState.state == FallDownState::pickedUp)
        goto waitAndSettleAfterHeadMoved;
      if(status != SampleConfigurationStatus::recording && status != SampleConfigurationStatus::waiting && state_time > waitDurationForSamplesWhenRecording && theFrameInfo.getTimeSince(theCameraCalibrationStatus.inStatusSince) > notVisibleThreshold)
      {
        currentSampleConfiguration->request.doRecord = false;
        goto adjustHeadAngle;
      }
    }
    action
    {
      recordCurrentConfiguration();
      setHeadMotionRequest(currentSampleConfiguration->request);
      Stand();
    }
  }

  state(optimize)
  {
    transition
    {
      // Wait for the optimizer to finish.
      if(theCameraCalibrationStatus.state == CameraCalibrationStatus::State::idle && state_time > 100)
      {
        // Save the camera calibration.
        saveCameraCalibration(theCameraCalibration);
        goto finished;
      }
    }
    action
    {
      theCalibrationRequest.targetState = CameraCalibrationStatus::State::optimize;

      CalibrateRobot({.request = theCalibrationRequest});
      Say({.text = "Optimizing."});
      LookAtAngles({.pan = 0.f,
                    .tilt = 0.f});
      Stand();
    }
  }

  target_state(finished)
  {
    action
    {
      theCalibrationRequest = {};
      CalibrateRobot({.request = theCalibrationRequest});

      finishedCalibration = true;
      LookForward();
      Stand();
    }
  }
}
