/**
 * @file JointPlayOffsetController.cpp
 * @author Philip Reichenberg
 */

#include "JointPlayOffsetController.h"

JointPlayOffsetController::JointPlayOffsetController(const JointRequest& request)
{
  this->fill(0);
  lastRequest = request;
}

void JointPlayOffsetController::update(const JointPlayTranslation& jointPlayTranslation, JointRequest& request,
                                       const bool isRightSupport, const bool isStanding, const JointPlayOffsetParameters& params,
                                       const JointAngles& appliedChanged, const JointAngles& appliedGyro, const float ratio)
{
  auto getRatio = [&](const float ratio, const std::size_t joint)
  {
    float baseRatio = 0.f;
    if(joint < Joints::firstLegJoint)
      return baseRatio;
    if(joint == Joints::lHipYawPitch || joint == Joints::rHipYawPitch ||
       joint == Joints::lHipRoll || joint == Joints::rHipRoll)
      baseRatio = std::min(params.maxFactorBase, ratio);
    else
      baseRatio = params.maxFactorBase;
    if(joint == Joints::lAnklePitch || joint == Joints::rAnklePitch)
      baseRatio *= params.applyOffsetFactorAnkle;
    else if(joint == Joints::lHipPitch || joint == Joints::rHipPitch)
      baseRatio *= params.applyOffsetFactorHipPitch;
    else
      baseRatio *= params.applyOffsetFactorGeneral;
    return baseRatio;
  };

  auto applyOffset = [&](const std::size_t joint)
  {
    const Rangea appliedChangeRange(std::min(0_deg, std::min(Angle(appliedGyro.angles[joint] - appliedChanged.angles[joint]), std::min(appliedGyro.angles[joint], -appliedChanged.angles[joint]))),
                                    std::max(0_deg, std::max(Angle(appliedGyro.angles[joint] - appliedChanged.angles[joint]), std::max(appliedGyro.angles[joint], -appliedChanged.angles[joint]))));
    Angle requestChange = request.angles[joint] - lastRequest.angles[joint];
    requestChange -= appliedChangeRange.limit(requestChange);
    this->at(joint) = jointPlayTranslation.jointOffset[joint].limit(requestChange);
    this->at(joint) *= getRatio(ratio, joint);
    request.angles[joint] -= this->at(joint);
  };

  for(std::size_t joint = (isRightSupport && !isStanding ? Joints::firstRightLegJoint : Joints::firstLeftLegJoint) + 1; joint < (isRightSupport || isStanding ? Joints::numOfJoints : Joints::firstRightLegJoint); joint++)
    applyOffset(joint);

  applyOffset(Joints::lHipYawPitch);
  request.angles[Joints::rHipYawPitch] -= this->at(Joints::lHipYawPitch);

  lastRequest = request;
}
