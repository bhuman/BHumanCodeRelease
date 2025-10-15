/**
 * @file ManualCameraCalibration.cpp
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

namespace ManualCameraCalibration
{
  struct SampleConfigurationInfo
  {
    SampleConfigurationRequest request;
  };
}
using namespace ManualCameraCalibration;

option((SkillBehaviorControl)ManualCameraCalibration,
       defs((int)(4500) waitDurationForSamplesWhenRecording, /**< The time we look for samples when recording before adjusting head or position, in milliseconds. */
            (int)(1000) notVisibleThreshold, /**< Assume not all lines are visible after this time in not visible state. */
            (std::vector<Angle>)({ 0.f, -0.9f, -2.07f, 0.9f, 2.07f }) headPans, /**< The head pan angles for recording samples. */
            (std::vector<Angle>)({ 0.34f, 0.26f, 0.26f, 0.26f, 0.26f }) upperHeadTilts,
            (std::vector<Angle>)({ -0.25f, -0.34f, -0.33f, -0.34f, -0.33f }) lowerHeadTilts,
            (Angle)(50_deg) headSpeed, /**< The maximum speed with which the head moves between targets (deg/s). */
            (bool)(true) useGoalAreaConnectingLineDistance),
       vars((bool)(false) finishedCalibration,
            (unsigned)(0) totalNumOfSamples,
            (std::vector<SampleConfigurationInfo>) sampleConfigurations,
            (std::vector<SampleConfigurationInfo>::iterator) currentSampleConfiguration,
            (CalibrationRequest) theCalibrationRequest))
{
  theCalibrationRequest.preciseJointPositions = true;

  const auto addSampleConfiguration = [&](unsigned index, CameraInfo::Camera camera, const Angle& headPan, const Angle& headTilt, unsigned sampleTypes)
  {
    SampleConfigurationRequest request;
    request.index = index;
    request.camera = camera;
    request.headPan = headPan;
    request.orgHeadPan = headPan;
    request.headTilt = headTilt;
    request.sampleTypes = sampleTypes;

    SampleConfigurationInfo config = { request };
    sampleConfigurations.push_back(config);
    FOREACH_ENUM(SampleType, sampleType)
      if(sampleTypes & bit(sampleType))
        ++totalNumOfSamples;
  };

  const auto setSampleConfigurationRequest = [&](const SampleConfigurationRequest& request)
  {
    theCalibrationRequest.totalNumOfSamples = totalNumOfSamples;
    theCalibrationRequest.sampleConfigurationRequest = SampleConfigurationRequest(request);
    CalibrateRobot({ .request = theCalibrationRequest });
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
    theCalibrationRequest.sampleConfigurationRequest = std::optional<SampleConfigurationRequest>{};
    CalibrateRobot({ .request = theCalibrationRequest });
  };

  const auto hasNextSampleConfiguration = [&]
  {
    return std::next(currentSampleConfiguration) != sampleConfigurations.end();
  };

  const auto nextSampleConfiguration = [&]
  {
    currentSampleConfiguration++;
    OUTPUT_TEXT("Prepared sample configuration: " << currentSampleConfiguration->request.index << " (for " << TypeRegistry::getEnumName(currentSampleConfiguration->request.camera) << ")");
  };

  const auto setHeadMotionRequest = [&](const SampleConfigurationRequest& request)
  {
    // Set the requested head angles.
    LookAtAngles({ .pan = request.headPan,
                   .tilt = request.headTilt + theRobotDimensions.getTiltNeckToCamera(false),
                   .speed = headSpeed,
                   .camera = HeadMotionRequest::upperCamera });
  };

  const auto saveCameraCalibration = [&](const CameraCalibration& cameraCalibration)
  {
    const std::string path = std::string(File::getBHDir()) + "/Config/Robots/" + Global::getSettings().headName + "/" + Global::getSettings().bodyName;
    std::filesystem::create_directories(path);
    OutMapFile stream(path + "/cameraCalibration.cfg", true);
    stream << cameraCalibration;
  };

  DECLARED_DEBUG_RESPONSE("option:ManualCameraCalibration:finishNow")
    finishedCalibration = true;

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
        goto wait;
      }
    }
    action
    {
      OUTPUT_TEXT("Initializing sample configurations");
      sampleConfigurations.clear();
      totalNumOfSamples = 0;

      int index = 0;
      std::size_t cameraIndex = 0;
      ASSERT(upperHeadTilts.size() == lowerHeadTilts.size());
      ASSERT(upperHeadTilts.size() == headPans.size());
      unsigned sampleTypes = bit(cornerAngle) | bit(parallelAngle) | bit(parallelLinesDistance);
      if(useGoalAreaConnectingLineDistance)
        sampleTypes |= bit(parallelLinesDistanceShort);

      // 5 head pans & tilts * 2 cameras * 4 sample types = 40 samples
      for(auto& headPan : headPans)
      {
        ASSERT(cameraIndex < upperHeadTilts.size());
        addSampleConfiguration(index++, CameraInfo::upper, headPan, upperHeadTilts[cameraIndex], sampleTypes);
        addSampleConfiguration(index++, CameraInfo::lower, headPan, lowerHeadTilts[cameraIndex++], sampleTypes);
      }

      ASSERT(!sampleConfigurations.empty());
      currentSampleConfiguration = sampleConfigurations.begin();

      theCalibrationRequest.totalNumOfSamples = totalNumOfSamples;
      theCalibrationRequest.targetState = CameraCalibrationStatus::State::recordSamples;

      CalibrateRobot({.request = theCalibrationRequest});
      LookForward();
      Stand();
    }
  }

  state(wait)
  {
    transition
    {
      if(theKeyStates.pressed[KeyStates::lFootLeft])
      {
        goto prepareRecording;
      }
    }
    action
    {
      if(state_time == 0)
      {
        OUTPUT_TEXT("Waiting for the robot to settle.");
        Say({ .text = "Waiting" });
      }
      checkIfCurrentConfigurationCanRecord();
      setHeadMotionRequest(currentSampleConfiguration->request);
      Stand();
    }
  }

  state(prepareRecording)
  {
    transition
    {
      if(state_time >= 2000)
      {
        goto recordSamples;
      }
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
        goto wait;
      if(status == SampleConfigurationStatus::finished)
      {
        OUTPUT_TEXT("Finished recording current configuration.");
        Say({ .text = "Recording finished" });
        if(hasNextSampleConfiguration())
        {
          nextSampleConfiguration();
          goto wait;
        }
        else
        {
          goto optimize;
        }
      }

      if(theFallDownState.state == FallDownState::pickedUp ||
         (status != SampleConfigurationStatus::recording && status != SampleConfigurationStatus::waiting &&
          state_time > waitDurationForSamplesWhenRecording && theFrameInfo.getTimeSince(theCameraCalibrationStatus.inStatusSince) > notVisibleThreshold))
      {
        currentSampleConfiguration->request.doRecord = false;
        goto wait;
      }
    }
    action
    {
      if(state_time == 0)
      {
        OUTPUT_TEXT("Recording current configuration.");
        Say({ .text = "Recording started" });
      }
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
