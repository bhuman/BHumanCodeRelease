/**
 * @file PhotoModeEngine.h
 *
 * @author Ayleen Lührsen
 */

#pragma once

#include "Framework/Module.h"
#include "Math/Range.h"
#include "Platform/SystemCall.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/MotionControl/PhotoModeGenerator.h"
#include "Representations/MotionControl/EnergySaving.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Sensing/InertialData.h"
#include "Tools/BehaviorControl/Cabsl.h"
#include "Tools/Motion/MotionUtilities.h"

using BodyPart = PhotoModeGenerator::BodyPart;
using LEDGroup = PhotoModeGenerator::LEDGroup;
using LampColor = PhotoModeGenerator::LampColor;

MODULE(PhotoModeEngine,
{,
  REQUIRES(EnergySaving),
  REQUIRES(EnhancedKeyStates),
  REQUIRES(FrameInfo),
  REQUIRES(InertialData),
  REQUIRES(JointAngles),
  USES(JointRequest),
  PROVIDES(PhotoModeGenerator),
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) lowpassFilter,
    (int)(75) stiffness,
    (int)(20) unstiffness,
  }),
});

class PhotoModeEngine : public PhotoModeEngineBase
{
public:
  PhotoModeEngine();

  void update(PhotoModeGenerator& photoModeGenerator) override;

  bool wasMotionPhaseSet = false;
  std::size_t selectedLEDGroupIndex = 0;
  std::size_t selectedLEDColorIndex = 0;
  bool selectLEDs = false;
  JointRequest lastPose;
};

struct PhotoModePhase : MotionPhase, public cabsl::Cabsl<PhotoModePhase>
{
  explicit PhotoModePhase(PhotoModeEngine& engine);

  void applyPosingState();

  option(Root)
  {
    common_transition
    {
      Rangea torsoTiltX(-60_deg, 60_deg);
      Rangea torsoTiltY(-60_deg, 60_deg);
      if(!torsoTiltY.isInside(engine.theInertialData.angle.y()) ||
         !torsoTiltX.isInside(engine.theInertialData.angle.x()))
      {
        engine.theEnergySaving.shutDown();
        goto fall;
      }
    }

    initial_state(start)
    {
      // TODO: Wenn vom start aus interpolation, dann eine if-Abfrage ob state_time > 0
      transition
      {
        if(!engine.wasMotionPhaseSet)
          goto posing;
        else
        {
          targetAngles = engine.lastPose;
          targetAngles.stiffnessData.stiffnesses.fill(engine.stiffness);
          startAngles = jointRequestOutput;
          currentTimeStamp = engine.theFrameInfo.time;
          goto interpolation;
        }
      }

      // TODO: braucht action um Start- und Zielgelenkwinkel für ausgangposen speichern und Zeitstempel merken um Interpolation berechnen zu können
    }

    state(interpolation)
    {
      action
      {
        const float ratio = Rangef::ZeroOneRange().limit(engine.theFrameInfo.getTimeSince(currentTimeStamp) / 2000.f);
        MotionUtilities::interpolate(startAngles, targetAngles, ratio, jointRequestOutput, engine.theJointAngles);
      }
      transition
      {
        if(engine.theFrameInfo.getTimeSince(currentTimeStamp) > 2000)
        {
          jointRequestOutput = targetAngles;
          unstiff = false;
          goto posing;
        }
      }
    }

    state(posing)
    {
      action
      {
        if(state_time == 0)
          jointRequestPreHeat = jointRequestOutput;
        applyPosingState();
      }
    }

    state(fall)
    {
      transition
      {
        Rangea torsoTiltX(-20_deg, 20_deg);
        Rangea torsoTiltY(-20_deg, 20_deg);
        if(torsoTiltY.isInside(engine.theInertialData.angle.y()) ||
           torsoTiltX.isInside(engine.theInertialData.angle.x()))
        {
          currentTimeStamp = engine.theFrameInfo.time + static_cast<unsigned>(Constants::motionCycleTime * 1000);
          startAngles.angles = engine.theJointAngles.angles;
          targetAngles.angles = jointRequestOutput.angles;
          targetAngles.stiffnessData.stiffnesses.fill(engine.stiffness);
          goto interpolation;
        }
      }
      action
      {
        jointRequestOutput.stiffnessData.stiffnesses.fill(10);
      }
    }
  }

private:
  void update() override;
  bool isDone(const MotionRequest& motionRequest) const override;
  void calcJoints(const MotionRequest& motionRequest, JointRequest& jointRequest, Pose2f& odometryOffset, MotionInfo& motionInfo) override;
  std::unique_ptr<MotionPhase> createNextPhase(const MotionPhase& defaultPhase) const override;

  PhotoModeEngine& engine;
  JointRequest jointRequestOutput, startAngles, targetAngles, jointRequestPreHeat;
  std::size_t selectedBodyPartIndex = 0;
  Rangei bodyPartToJoints[BodyPart::numOfBodyParts];
  bool unstiff = false;
  std::size_t selectedLEDGroupIndex = 0;
  std::size_t selectedLEDColorIndex = 0;
  bool selectLEDs = false;
  PhotoModeGenerator::LEDGroup ledGroup = PhotoModeGenerator::eyes;
  unsigned lastHeadPressTimestamp = 0;
  unsigned currentTimeStamp = 0;
};
