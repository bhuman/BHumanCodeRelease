/**
 * @file FieldFeatureCheck.h
 * The file declares macros to assist a fast check-function declaration field features.
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Tools/Debugging/Annotation.h"
#include "Tools/Module/Blackboard.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/RobotPose.h"

/**
 * A marco for creating an pose to GroundTruthRobotPose check within the check-function of a STREAMABLE.
 *
 * @param name, the ANNOTATION name
 * @param pose, the pose variable to check
 * @param paramDist, threshold of allowed translation distance
 * @param paramRot, threshold of allowed rotation distance
 */
#ifdef TARGET_ROBOT
#define CHECK_POSE(name, pose, paramDist, paramRot) ;
#else
#define CHECK_POSE(name, pose, paramDist, paramRot)  \
  void verify() const \
  { \
    if(Blackboard::getInstance().exists("GroundTruthRobotPose")) \
    { \
      const GroundTruthRobotPose& theGroundTruthRobotPose = static_cast<const GroundTruthRobotPose&>(Blackboard::getInstance()["GroundTruthRobotPose"]); \
      const Pose2f diffPose = theGroundTruthRobotPose - pose; \
      if(diffPose.translation.norm() > paramDist) \
        ANNOTATION(name, "CHECK FAILED of CHECK_POSE: translation distance was" << diffPose.translation.norm() << " allowed: " << paramDist << " GTPOSE was " << theGroundTruthRobotPose.rotation << theGroundTruthRobotPose.translation.x() << "x " << theGroundTruthRobotPose.translation.y() << "y"); \
      if(std::abs(diffPose.rotation) > paramRot) \
        ANNOTATION(name, "CHECK FAILED of CHECK_POSE: rotation distance was" << diffPose.rotation << " allowed: " << paramRot << " GTPOSE was " << theGroundTruthRobotPose.rotation << theGroundTruthRobotPose.translation.x() << "x " << theGroundTruthRobotPose.translation.y() << "y"); \
    } \
  }
#endif

/**
 * Same as CHECK_POSE but explicit for REPRESENTATIONS that inherits FieldFeature.
 */
#ifdef TARGET_ROBOT
#define CHECK_FIELD_FEATURE_POSE(name, paramDist, paramRot) ;
#else
#define CHECK_FIELD_FEATURE_POSE(name, paramDist, paramRot)  \
  void verify() const \
  { \
    if(!this->isValid) return; \
    if(Blackboard::getInstance().exists("GroundTruthRobotPose")) \
    { \
      const GroundTruthRobotPose& theGroundTruthRobotPose = static_cast<const GroundTruthRobotPose&>(Blackboard::getInstance()["GroundTruthRobotPose"]); \
      const FieldFeature::RobotPoseToFF poses = this->getGlobalRobotPosition(); \
      const Pose2f pose = (poses.pos1 - theGroundTruthRobotPose).translation.squaredNorm() < (poses.pos2 - theGroundTruthRobotPose).translation.squaredNorm() ? poses.pos1 : poses.pos2; \
      const Pose2f diffPose = theGroundTruthRobotPose - pose; \
      if(diffPose.translation.norm() > paramDist) \
        ANNOTATION(name, "CHECK FAILED of CHECK_POSE: translation distance was" << diffPose.translation.norm() << " allowed: " << paramDist << " GTPOSE was " << theGroundTruthRobotPose.rotation << theGroundTruthRobotPose.translation.x() << "x " << theGroundTruthRobotPose.translation.y() << "y"); \
      if(std::abs(diffPose.rotation) > paramRot) \
        ANNOTATION(name, "CHECK FAILED of CHECK_POSE: rotation distance was" << diffPose.rotation << " allowed: " << paramRot << " GTPOSE was " << theGroundTruthRobotPose.rotation << theGroundTruthRobotPose.translation.x() << "x " << theGroundTruthRobotPose.translation.y() << "y"); \
    } \
  }
#endif
