/**
 * @file RefereeGestureDetection.cpp
 *
 * This file implements a module that detects referee gestures.
 *
 * @author <a href="mailto:aylu@uni-bremen.de">Ayleen Lührsen</a>
 */

#include "RefereeGestureDetection.h"
#include "Debugging/DebugDrawings.h"

MAKE_MODULE(RefereeGestureDetection, perception);

void RefereeGestureDetection::update(RefereePercept& theRefereePercept)
{
  /** This module detects poses of a human referee by taking the output moveNet and checking whether the arms and joints  fit certain contraints concerning the positions relative to each other. For that, we use a bitmask, by assigning a power of 2 to both arms. Also, we use a buffer to reduce the impact of falsely recognized keypoints due to noise or a pose that was just  below the threshold. This way, the process of deciding which pose the referee has taken will take a few frames, but it's accuracy is a lot better. */

  // make an array that can take in all relevant joints (and only those)
  Vector2f joints[Keypoints::rightWrist - Keypoints::leftShoulder + 1];
  // go over the relevant keypoints in the output of the net, and
  for(int i = Keypoints::leftShoulder; i <= Keypoints::rightWrist; i++)
  {
    const Keypoints::Point& point = theKeypoints.points[i];
    // if you reach one of the wanted joints, save them into the percept
    if(point.valid)
      joints[i - Keypoints::leftShoulder] = point.position;
    else
      joints[i - Keypoints::leftShoulder] = Vector2f(-1.f, -1.f);
  }

  // Decide whether the keypoints are left or right, since MoveNet likes to swap them but we REALLY need the correct sides
  Vector2f orderedJoints[7]; // shoulder, elbow, and wrist for each side plus an index that can be used for a vertical line

  // Swap sides for each joint side if necessary
  for(int i = 0; i < 6; i += 2)
  {
    // One or both joints missing
    if(joints[i].y() < 0 || joints[i].y() < 0)
    {
      // Both joints missing, just give them the corresponding indices
      if(joints[i].y() < 0 && joints[i + 1].y() < 0)
      {
        orderedJoints[i] = joints[i];
        orderedJoints[i + 1] = joints[i + 1];
      }
      // left joint missing, decide whether the other on is left or right based on their position to the center line of the image
      else if(joints[i].y() < 0)
      {
        orderedJoints[i] = joints[i + 1].x() < (theCameraInfo.width / 2) ? joints[i + 1] : joints[i];
        orderedJoints[i + 1] = joints[i + 1].x() >= (theCameraInfo.width / 2) ? joints[i + 1] : joints[i];
      }
      // left joint missing, decide whether the other on is left or right based on their position to the center line of the image
      else if(joints[i + 1].y() < 0)
      {
        orderedJoints[i] = joints[i].x() < (theCameraInfo.width / 2) ? joints[i + 1] : joints[i];
        orderedJoints[i + 1] = joints[i].x() >= (theCameraInfo.width / 2) ? joints[i + 1] : joints[i];
      }
    }
    // go over the joints to decide which is the left/right one based on their x-position
    else
    {
      if(joints[i].x() < joints[i + 1].x())
      {
        orderedJoints[i] = joints[i];
        orderedJoints[i + 1] = joints[i + 1];
      }
      else
      {
        orderedJoints[i] = joints[i + 1];
        orderedJoints[i + 1] = joints[i];
      }
    }
  }

  // Vector that is used as a reference for the angles towards the vertical
  orderedJoints[6] = Vector2f(0, 20);

  // Assign the bones which can be used for debug drawings
  Vector2f bones[4];
  for(int i = 0; i < 4; i++)
  {
    bones[i] = orderedJoints[i + 2] - orderedJoints[i];
  }

  /** Variables for Pose calculation */
  // Default Value for the Arm Poses, so even if we dont get all Keypoints from the Net, we can guess the pose due to the other arm
  leftArmPose = RefArmPose::none;
  rightArmPose = RefArmPose::none;
  // Ints to use Bit Shifts on, which are like a Bool Array, but with the option of switch-case-Statements instead of loops
  leftArmPoseInt = 0;
  rightArmPoseInt = 0;
  // calculate the relative distance from shoulder to wrist in relation to the Shoulder-Hip-Line
  // Use the middle of both sides to be a bit more save from false keypoints
  const Vector2f middleOfShoulders = (theKeypoints.points[theKeypoints.leftShoulder].position + 0.5f * (theKeypoints.points[theKeypoints.rightShoulder].position - theKeypoints.points[theKeypoints.leftShoulder].position));
  const Vector2f middleOfHips = (theKeypoints.points[theKeypoints.leftHip].position + 0.5f * (theKeypoints.points[theKeypoints.rightHip].position - theKeypoints.points[theKeypoints.leftHip].position));
  // length of the line from middle of shoulders to middle of hips
  float shoulderHip = (middleOfHips - middleOfShoulders).norm(); 
  // Left Arm length in relation to height
  float armBodyRatioL = (bones[0].stableNorm() + bones[2].stableNorm()) / shoulderHip;
  // Right Arm length in relation to height
  float armBodyRatioR = (bones[1].stableNorm() + bones[3].stableNorm()) / shoulderHip;

  /** Check whether a keypoint wasn't found or was below the minimal confidence threshold, we dont want to use those and ignore them in the calculation if that is the case */
  // default  is that every keypoint has a valid value
  bool leftJointMissing = false;
  bool rightJointMissing = false;
  // go over all joints
  for(int i = 0; i < 6; i++)
  {
    // invalid keypoint variables are set to (-1,-1) which will never occur naturally, so we can check for that
    if((i % 2) == 0 && !leftJointMissing)
    {
      leftJointMissing = orderedJoints[i].x() < 0;
    }
    else if((i % 2) == 1 && !rightJointMissing)
    {
      rightJointMissing = orderedJoints[i].x() < 0;
    }
  }

  /** Iterate over all valid arm posesand check whether their constraints are fullfilled by the keypoints */
  FOREACH_ENUM(RefArmPose, j)
  {
    // Left Arm
    if(!leftJointMissing // If a joint is missing, we dont want to consider this arm
       && constraints[j].angleBodyToUpperArm.isInside(bones[0].angleTo(orderedJoints[6])) // Angle Between Upperarm and Backbone
       && constraints[j].angleUpperToLower.isInside(bones[2].angleTo(bones[0])) // Angle of the Elbow (Upper Arm - Lower Arm)
       && constraints[j].distanceRatio.contains(armBodyRatioL)) // Relative joint distance
    {
      leftArmPoseInt = bit(j);
      leftArmPose = j;
    }
    // Right Arm
    if(!rightJointMissing // If a joint is missing, we dont want to consider this arm
       && constraints[j].angleBodyToUpperArm.isInside(bones[1].angleTo(orderedJoints[6]))  // Angle Between Upperarm and Backbone
       && constraints[j].angleUpperToLower.isInside(bones[3].angleTo(bones[1])) // Angle of the Elbow (Upper Arm - Lower Arm)
       && constraints[j].distanceRatio.contains(armBodyRatioR)) // Relative joint distance
    {
      rightArmPoseInt = bit(j) << numOfRefArmPoses;
      rightArmPose = j;
    }
  }

  // decrement the number of appearances for the element that will be deleted from the ringbuffer next round
  if(!poseMemory.empty() && poseMemory.size() == poseMemory.capacity()) // but only if the ringbuffer is fully filled
  {
    poseAppearances[poseMemory.back()]--;
  }

  // Decide a position based on one or both arms. NOTE: this only increments the appearance counter, but is NOT directly assigned to the Referee in this statement, because 
  // Also note: there 3 Poses involving an arm that is 90deg sideways. When testing, we found that forward is the arm position that is the least recognized, so if one arm doesn't fits any constraints while the other is 90 degrees, we will assume this arm is in forward position, which produced only in rare cases false positives, but reduced the number of false negatives by a huge amount. 
  // There are also other cases in which we assume a false detection and chose the most plausible pose, since false positives are likely to be ignored anyways due to the buffer
  switch(leftArmPoseInt + rightArmPoseInt)
  {
    // KICK IN
    // One Arm down, one arm 90 deg sideways
    // Blue Team
    case bit(degree90) | bit(down) << numOfRefArmPoses:
      poseMemory.push_front(RefereePercept::kickInBlue);
      poseAppearances[RefereePercept::kickInBlue]++;
      break;
    // Red Team
    case bit(down) | bit(degree90) << numOfRefArmPoses:
      poseMemory.push_front(RefereePercept::kickInRed);
      poseAppearances[RefereePercept::kickInRed]++;
      break;
    // GOAL KICK
    // One Arm up, one arm down
    // Blue Team
    case bit(up) | bit(down) << numOfRefArmPoses:
      poseMemory.push_front(RefereePercept::goalKickBlue);
      poseAppearances[RefereePercept::goalKickBlue]++;
      break;
    // Red Team
    case bit(down) | bit(up) << numOfRefArmPoses:
      poseMemory.push_front(RefereePercept::goalKickRed);
      poseAppearances[RefereePercept::goalKickRed]++;
      break;
    // CORNER KICK
    // one arm sideways 45 deg, one down
    // Blue Team
    case bit(degree45) | bit(down) << numOfRefArmPoses:
      poseMemory.push_front(RefereePercept::cornerKickBlue);
      poseAppearances[RefereePercept::cornerKickBlue]++;
      break;
    // Red Team
    case bit(down) | bit(degree45) << numOfRefArmPoses:
      poseMemory.push_front(RefereePercept::cornerKickRed);
      poseAppearances[RefereePercept::cornerKickRed]++;
      break;
    // GOAL
    // one Arm sideways 90 deg, one arm pointing towards the middle circle
    // Blue Team
    case bit(degree45):
    case bit(degree90):
    case bit(up):
    case bit(degree45) | bit(forward) << numOfRefArmPoses:
    case bit(degree90) | bit(forward) << numOfRefArmPoses:
    case bit(up) | bit(forward) << numOfRefArmPoses:
      poseMemory.push_front(RefereePercept::goalBlue);
      poseAppearances[RefereePercept::goalBlue]++;
      break;
    // Red Team
    case bit(degree45) << numOfRefArmPoses:
    case bit(degree90) << numOfRefArmPoses:
    case bit(up) << numOfRefArmPoses:
    case bit(forward) | bit(degree45) << numOfRefArmPoses:
    case bit(forward) | bit(degree90) << numOfRefArmPoses:
    case bit(forward) | bit(up) << numOfRefArmPoses:
      poseMemory.push_front(RefereePercept::goalRed);
      poseAppearances[RefereePercept::goalRed]++;
      break;
    // PUSHING FREE KICK
    // One Arm pointing sideways 90 deg, the other folded across the chest
    // Blue Team
    case bit(degree45) | bit(folded) << numOfRefArmPoses:
    case bit(degree90) | bit(folded) << numOfRefArmPoses:
    case bit(up) | bit(folded) << numOfRefArmPoses:
      poseMemory.push_front(RefereePercept::pushingFreeKickBlue);
      poseAppearances[RefereePercept::pushingFreeKickBlue]++;
      break;
    // Red Team
    case bit(folded) | bit(degree45) << numOfRefArmPoses:
    case bit(folded) | bit(degree90) << numOfRefArmPoses:
    case bit(folded) | bit(up) << numOfRefArmPoses:
      poseMemory.push_front(RefereePercept::pushingFreeKickRed);
      poseAppearances[RefereePercept::pushingFreeKickRed]++;
      break;
    // FULL TIME
    // Both Arms moving inwards and outwards from folded to 90 deg spread
    case bit(degree90) | bit(degree90) << numOfRefArmPoses:
    case bit(wing) | bit(wing) << numOfRefArmPoses:
    case bit(spreadwing) | bit(spreadwing) << numOfRefArmPoses:
      poseMemory.push_front(RefereePercept::Gesture::fullTime);
      poseAppearances[RefereePercept::fullTime]++;
      break;
    default:
      poseMemory.push_front(RefereePercept::none);
      poseAppearances[RefereePercept::none]++;
      break;
  }
  // Our poseCandidate is none to begin with
  RefereePercept::Gesture poseCandidate = RefereePercept::Gesture::none;
  // based on the number of appearances, find the pose that was estimated the most times over the last frames
  poseCandidate = static_cast<RefereePercept::Gesture>(std::max_element(poseAppearances.begin(), poseAppearances.end()) - poseAppearances.begin());
  // only set it as referee pose, if the appearance counter is bigger than a set portion of the buffer size. This way, we can hopefully eliminate poses that were only
  if(poseAppearances[poseCandidate] > static_cast<int>(poseMemory.capacity() / posePortion))
  {
    theRefereePercept.gesture = poseCandidate;
  }

  /*Draw the Lines for the arms, but only if both keypoints were detected with a confidence over the specified value*/
  DEBUG_DRAWING("module:RefereeGestureDetection:arms", "drawingOnImage")
  {
    for(int i = 0; i < 4; i++)
    {
      if(orderedJoints[i].y() > 0 && orderedJoints[i + 2].y() > 0)
      {
        ColorRGBA col = i % 2 == 0 ? ColorRGBA::blue : ColorRGBA::green;
        // Draw bones of the arms (blue left, green right)
        LINE("module:RefereeGestureDetection:arms", orderedJoints[i].x(), orderedJoints[i].y(), orderedJoints[i + 2].x(), orderedJoints[i + 2].y(), 5, Drawings::solidPen, col);
        // Draw the angles
        if(i < 2)
        {
          DRAW_TEXT("module:RefereeGestureDetection:arms", orderedJoints[i].x(), orderedJoints[i].y(), 5, ColorRGBA::white, Angle((bones[i].angleTo(orderedJoints[6]))).toDegrees());
        }
        else
        {
          DRAW_TEXT("module:RefereeGestureDetection:arms", orderedJoints[i].x(), orderedJoints[i].y(), 5, ColorRGBA::red, Angle((bones[i - 2].angleTo(bones[i]))).toDegrees());
        }
      }
      // Draw the length of the bone
      DRAW_TEXT("module:RefereeGestureDetection:arms", orderedJoints[i].x(), orderedJoints[i].y() + 10, 5, ColorRGBA::yellow, bones[i].stableNorm());
    }
    // Write the detected arm pose both with name and as Integer Value (bit mask)
    DRAW_TEXT("module:RefereeGestureDetection:arms", orderedJoints[2].x() - 30, orderedJoints[2].y(), 7, ColorRGBA::red, TypeRegistry::getEnumName(leftArmPose));
    DRAW_TEXT("module:RefereeGestureDetection:arms", orderedJoints[2].x() - 30, orderedJoints[2].y() + 10, 7, ColorRGBA::red, leftArmPoseInt);
    DRAW_TEXT("module:RefereeGestureDetection:arms", orderedJoints[3].x() + 30, orderedJoints[3].y(), 7, ColorRGBA::red, TypeRegistry::getEnumName(rightArmPose));
    DRAW_TEXT("module:RefereeGestureDetection:arms", orderedJoints[3].x() + 30, orderedJoints[3].y() + 10, 7, ColorRGBA::red, rightArmPoseInt);
  }

  /** Draw a line from shoulder to wrist alongside the angle in degrees to a vertical line and the length relative to the shoulder-hip-line we calculated earlier */
  DEBUG_DRAWING("module:RefereeGestureDetection:shoulderToWrist", "drawingOnImage")
  {
    // Left arm
    if(orderedJoints[0].y() > 0 && orderedJoints[4].y() > 0) // check whether all joints are valid
    {
      // The vector to draw
      Vector2f leftLine = orderedJoints[4] - orderedJoints[0];
      // draw the vector
      LINE("module:RefereeGestureDetection:shoulderToWrist", orderedJoints[0].x(), orderedJoints[0].y(), orderedJoints[4].x(), orderedJoints[4].y(), 2, Drawings::solidPen, ColorRGBA::red);
      // print the angle to the body in degrees
      DRAW_TEXT("module:RefereeGestureDetection:shoulderToWrist", orderedJoints[0].x() - 50, orderedJoints[0].y() + 50, 5, ColorRGBA::red, Angle((leftLine.angleTo(orderedJoints[6]))).toDegrees());
      // below the angle, also print the length relative to the body height
      DRAW_TEXT("module:RefereeGestureDetection:shoulderToWrist", orderedJoints[0].x() - 50, orderedJoints[0].y() + 60, 5, ColorRGBA::magenta,  armBodyRatioL);
    }
    // Right arm
    if(orderedJoints[1].y() > 0 && orderedJoints[5].y() > 0) // check whether all joints are valid
    {
      // The vector to draw
      Vector2f rightLine = orderedJoints[5] - orderedJoints[1]; 
      // draw the vector
      LINE("module:RefereeGestureDetection:shoulderToWrist", orderedJoints[1].x(), orderedJoints[1].y(), orderedJoints[5].x(), orderedJoints[5].y(), 2, Drawings::solidPen, ColorRGBA::red);
      // print the angle to the body in degrees
      DRAW_TEXT("module:RefereeGestureDetection:shoulderToWrist", orderedJoints[0].x() + 50, orderedJoints[0].y() + 50, 5, ColorRGBA::red, Angle((rightLine.angleTo(orderedJoints[6]))).toDegrees());
      // below the angle, also print the length relative to the body height
      DRAW_TEXT("module:RefereeGestureDetection:shoulderToWrist", orderedJoints[0].x() + 50, orderedJoints[0].y() + 60, 5, ColorRGBA::magenta, armBodyRatioR);
    }
  }
}
