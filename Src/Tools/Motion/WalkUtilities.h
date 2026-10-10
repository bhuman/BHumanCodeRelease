#include "Math/Pose2f.h"

struct MotionPhase;
struct WalkGenerator;
struct WalkingEngineOutput;
struct WalkStepData;

namespace WalkUtilities
{
  /**
   * Allowed angle range that the targetOfInterest is allowed to have relative to the robot
   */
  const Rangea maxTargetFocusAngle = Rangea(-40_deg, 40_deg); // Keep the target of interest in a cone in front of the robot
  const Angle maxDiagonalAngle = 30_deg;
  const Angle extraFocusShiftIfBallNotInVision = 0;
  constexpr float maxSideStepDiagonalWalk = 35.f; // Max allowed side step size during diagonal walking

  /**
   *
   * Calculate the orientation to walk to a target sideways, which keeps the targetOfInterest in vision
   * @param theWalkGenerator The walkGenerator representation
   * @param walkOutput The walkingEngineOutput representation
   * @param walkStepData The walk speed of last step
   * @param walkSpeed The max allowed walking speed
   * @param targetInSCS The target pose
   * @param modTargetInSCS The modified pose
   * @param targetOfInterest The target of interest, which shall remain inside of maxTargetFocusAngle
   * @param isLeftPhase Is the next walking step a left phase?
   * @param isFastWalk Is fast walk allowed, e.g. more translation in rotation steps
   * @param lastPhase The last motion phase
   * @param useModTarget Shall the modTargetInSCS be used to calculate the walk direction
   */
  void calcSideWalk(const WalkGenerator& theWalkGenerator, const WalkingEngineOutput& walkOutput,
                    const WalkStepData& walkStepData, const Pose2f& walkSpeed,
                    const Pose2f& targetInSCS, Pose2f& modTargetInSCS,
                    const Vector2f& targetOfInterest, const bool isLeftPhase, const bool isFastWalk,
                    const MotionPhase& lastPhase, const bool useModTarget);

  /**
   * Calculate the orientation to walk to a target, which keeps the targetOfInterest in vision
   * and makes use of the fact, that we can to larger steps when walking diagonal
   * @param walkStepData The walk speed of last step
   * @param modTargetInSCS The modified pose
   * @param targetOfInterest The target of interest, which shall remain inside of maxTargetFocusAngle
   * @param isFastWalk Is fast walk allowed, e.g. more translation in rotation steps
   */
  void calcDiagonal(const WalkStepData& walkStepData, Pose2f& modTargetInSCS, const Vector2f& targetOfInterest, const bool isFastWalk);
}
