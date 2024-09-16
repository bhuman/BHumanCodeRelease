/**
 * @file PhotoModeEngine.cpp
 *
 * @author Ayleen Lührsen
 */

#include "PhotoModeEngine.h"
#include "Debugging/Annotation.h"
#include "Debugging/DebugDrawings3D.h"
#include "Platform/File.h"
#include "Math/Eigen.h"
#include "Math/Rotation.h"

MAKE_MODULE(PhotoModeEngine);

PhotoModeEngine::PhotoModeEngine()
{
}

void PhotoModeEngine::update(PhotoModeGenerator& photoModeGenerator)
{
  photoModeGenerator.createPhase = [this](const MotionRequest&, const MotionPhase&)
  {
    lastPose = theJointRequest;
    return std::make_unique<PhotoModePhase>(*this);
  };
  photoModeGenerator.ledGroup = LEDGroup(selectedLEDGroupIndex);
  photoModeGenerator.lamps[LEDGroup(selectedLEDGroupIndex)] = LampColor(selectedLEDColorIndex);
  photoModeGenerator.selectLEDs = selectLEDs;
  photoModeGenerator.isActive = true;
}

PhotoModePhase::PhotoModePhase(PhotoModeEngine& engine) :
  MotionPhase(MotionPhase::photoMode),
  engine(engine)
{
  bodyPartToJoints[BodyPart::head] = { Joints::headYaw, Joints::firstArmJoint };
  bodyPartToJoints[BodyPart::leftArm] = { Joints::firstArmJoint, Joints::firstRightArmJoint };
  bodyPartToJoints[BodyPart::rightArm] = { Joints::firstRightArmJoint, Joints::firstLegJoint };
  bodyPartToJoints[BodyPart::leftLeg] = { Joints::firstLegJoint, Joints::firstRightLegJoint };
  bodyPartToJoints[BodyPart::rightLeg] = { Joints::firstRightLegJoint, Joints::numOfJoints };
  bodyPartToJoints[BodyPart::bothLegs] = { Joints::firstLeftLegJoint, Joints::numOfJoints };

  jointRequestOutput = engine.theJointRequest;
  FOREACH_ENUM(Joints::Joint, joint)
  {
    if(jointRequestOutput.stiffnessData.stiffnesses[joint] == 0 ||
       jointRequestOutput.angles[joint] == JointAngles::off)
      jointRequestOutput.angles[joint] = engine.theJointAngles.angles[joint];
    jointRequestOutput.stiffnessData.stiffnesses[joint] = engine.stiffness;
  }
}

void PhotoModePhase::update()
{
}

bool PhotoModePhase::isDone(const MotionRequest& motionRequest) const
{
  return motionRequest.motion != MotionRequest::photoMode;
}

void PhotoModePhase::calcJoints(const MotionRequest&, JointRequest& jointRequest, Pose2f&, MotionInfo&)
{
  // Execute behavior
  beginFrame(engine.theFrameInfo.time);
  execute("Root");
  endFrame();
  engine.wasMotionPhaseSet = true;
  engine.lastPose = jointRequestOutput;
  jointRequest = jointRequestOutput;
}

void PhotoModePhase::applyPosingState()
{
  // Apply joint values without heat adjustment
  jointRequestOutput = jointRequestPreHeat;
  if(engine.theFrameInfo.getTimeSince(lastHeadPressTimestamp) > 1000)
  {
    if(engine.theEnhancedKeyStates.pressed[KeyStates::headFront] && engine.theEnhancedKeyStates.pressed[KeyStates::headRear])
    {
      selectLEDs = !selectLEDs;
      SystemCall::say(selectLEDs ? "Light me up" : "I am a poser");
      lastHeadPressTimestamp = engine.theFrameInfo.time;
    }
    // Pose the robot if selectLEDs is disabled
    else if(!selectLEDs)
    {
      // Select prior body part
      if(engine.theEnhancedKeyStates.pressed[KeyStates::headRear] && !unstiff)
      {
        selectedBodyPartIndex = --selectedBodyPartIndex >= BodyPart::numOfBodyParts ? BodyPart::numOfBodyParts - 1 : selectedBodyPartIndex; // remember the overflow wink wink

        SystemCall::say(("Selected " + std::string(TypeRegistry::getEnumName(BodyPart(selectedBodyPartIndex)))).c_str());
        lastHeadPressTimestamp = engine.theFrameInfo.time;
      }
      // Select next body part
      else if(engine.theEnhancedKeyStates.pressed[KeyStates::headFront] && !unstiff)
      {
        selectedBodyPartIndex = ++selectedBodyPartIndex >= BodyPart::numOfBodyParts ? 0 : selectedBodyPartIndex;
        SystemCall::say(("Selected " + std::string(TypeRegistry::getEnumName(BodyPart(selectedBodyPartIndex)))).c_str());
        lastHeadPressTimestamp = engine.theFrameInfo.time;
      }

      // Make selected body part unstiff
      if(engine.theEnhancedKeyStates.pressed[KeyStates::headMiddle])
      {
        unstiff = !unstiff;
        SystemCall::say(unstiff ? "unstiff" : "stiff");
        for(int joint = bodyPartToJoints[BodyPart(selectedBodyPartIndex)].min; joint < bodyPartToJoints[BodyPart(selectedBodyPartIndex)].max; joint++)
        {
          jointRequestOutput.stiffnessData.stiffnesses[joint] = unstiff ? engine.unstiffness : engine.stiffness;

          if(!unstiff)
            jointRequestOutput.angles[joint] = engine.theJointAngles.angles[joint];
        }
        lastHeadPressTimestamp = engine.theFrameInfo.time;
      }
    }
    else
    {
      // Change LEDs
      if(engine.theEnhancedKeyStates.pressed[KeyStates::headFront])
      {
        selectedLEDGroupIndex = ++selectedLEDGroupIndex >= PhotoModeGenerator::numOfLEDGroups ? 0 : selectedLEDGroupIndex;
        SystemCall::say(("Selected " + std::string(TypeRegistry::getEnumName(LEDGroup(selectedLEDGroupIndex)))).c_str());
        lastHeadPressTimestamp = engine.theFrameInfo.time;
        ledGroup = LEDGroup(selectedLEDGroupIndex);
      }
      // Aint much and it doesn't work, so we just use forward cycling
      if(engine.theEnhancedKeyStates.pressed[KeyStates::headRear])
      {
        selectedLEDGroupIndex = --selectedLEDGroupIndex >= PhotoModeGenerator::numOfLEDGroups ? PhotoModeGenerator::numOfLEDGroups - 1 : selectedLEDGroupIndex;
        SystemCall::say(("Selected " + std::string(TypeRegistry::getEnumName(LEDGroup(selectedLEDGroupIndex)))).c_str());
        lastHeadPressTimestamp = engine.theFrameInfo.time;
        ledGroup = LEDGroup(selectedLEDGroupIndex);
      }
      if(engine.theEnhancedKeyStates.pressed[KeyStates::headMiddle])
      {
        selectedLEDColorIndex = ++selectedLEDColorIndex >= PhotoModeGenerator::numOfLampColors ? 0 : selectedLEDColorIndex;
        lastHeadPressTimestamp = engine.theFrameInfo.time;
      }
    }

    // low pass filter, so the selected joint will not just fall but obey to gravity in a slow and controlled manner if you don't hold it
    if(unstiff)
    {
      // the energy saving mode can be ignore, as the request is replaced with the measured positions anyway
      for(int joint = bodyPartToJoints[BodyPart(selectedBodyPartIndex)].min; joint < bodyPartToJoints[BodyPart(selectedBodyPartIndex)].max; joint++)
      {
        jointRequestOutput.angles[joint] = (engine.lowpassFilter * jointRequestOutput.angles[joint]) + ((1 - engine.lowpassFilter) * engine.theJointAngles.angles[joint]);
      }
    }

    engine.selectLEDs = selectLEDs;
    engine.selectedLEDColorIndex = selectedLEDColorIndex;
    engine.selectedLEDGroupIndex = selectedLEDGroupIndex;
  }

  // Save joint values from before the heat adjustment
  jointRequestPreHeat = jointRequestOutput;
  // Only apply heat adjustment for the limbs with "stiffness", skip the ones with "unstiffness"
  engine.theEnergySaving.applyHeatAdjustment(jointRequestOutput,
                                             BodyPart(selectedBodyPartIndex) != BodyPart::leftLeg || !unstiff,
                                             BodyPart(selectedBodyPartIndex) != BodyPart::rightLeg || !unstiff,
                                             BodyPart(selectedBodyPartIndex) != BodyPart::leftArm || !unstiff,
                                             BodyPart(selectedBodyPartIndex) != BodyPart::rightArm || !unstiff,
                                             /*standHigh=*/ false, /*accuratePositions=*/ false);
}

std::unique_ptr<MotionPhase> PhotoModePhase::createNextPhase(const MotionPhase&) const
{
  return std::unique_ptr<MotionPhase>();
}
