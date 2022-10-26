/**
 * @file WalkStepAdjustment.h
 * This file declares our Walk Step Adjustment. See our paper <Step Adjustment for a Robust Humanoid Walk> for more details.
 * @author Philip Reichenberg
 */

#include "WalkStepAdjustment.h"
#include "Debugging/Annotation.h"
#include "Math/Geometry.h"
#include "Math/Rotation.h"
#include <cmath>

WalkStepAdjustment::WalkStepAdjustment()
{
  lastLeftAdjustmentX = 0.f;
  lastRightAdjustmentX = 0.f;
  highestNegativeAdjustmentX = 0.f;
  highestAdjustmentX = 0.f;
  kneeHipBalanceCounter = 0.f;
  hipBalanceIsSafeBackward = 0.f;
  hipBalanceIsSafeForward = 0.f;
  backwardsWalkingRotationCompensationFactor = 0.f;
  forwardsWalkingRotationCompensationFactor = 1.f;
  swingFootXTranslationChange = 0;
  lastLargeBackwardStep = 0;
  lastNormalStep = 0;
  reduceWalkingSpeed = 0;
}

//todo for small movements (like standing), there needs to be some filtering
RotationMatrix WalkStepAdjustment::predictRotation(RotationMatrix rotationMatrix, const RobotModel& robotModel, const FootOffset& footOffset, const bool isLeftPhase, const bool prediction)
{
  // pre calculation. Get current CoM position and init last values. Also calculate the support polygon
  Vector3f lastComInFoot = lastMeasuredComInFoot;
  Vector3f lastComInRobot = lastMeasuredCom;
  lastMeasuredCom = rotationMatrix.inverse() * robotModel.centerOfMass;
  RotationMatrix lastRotationMatrix = lastMeasuredRotationMatrix;
  lastMeasuredRotationMatrix = rotationMatrix;
  calcSupportPolygon(robotModel, footOffset, isLeftPhase);
  for(int i = 0; i < (prediction ? updateSteps : 1); i++)
  {
    // 1. calc current com relative to foot plane
    if(lastRotationMatrix == RotationMatrix())
      lastRotationMatrix = rotationMatrix; // robot just started to walk
    const RotationMatrix currentRotationMatrix = rotationMatrix;
    const Vector3f currentComInRobot = rotationMatrix.inverse() * robotModel.centerOfMass;

    const Vector3f currentComInFoot = rotationMatrix * (Vector3f() << currentComInRobot.head<2>(), (rotationMatrix.inverse() * (isLeftPhase ? robotModel.soleRight : robotModel.soleLeft).translation).z()).finished();
    if(i == 0)
      lastMeasuredComInFoot = currentComInFoot;
    if(lastComInFoot == Vector3f::Zero()) // robot just started to walk
      return rotationMatrix;
    // 2. calc tilting edge
    Vector3f tiltingEdge;
    if(!getTiltingPoint(currentComInFoot, lastComInFoot, tiltingEdge))
      return rotationMatrix;

    // 3. convert into robot coordinates
    const Vector3f tiltingEdgeInCurrentRobot = rotationMatrix.inverse() * tiltingEdge;
    const Vector3f tiltingEdgeInOldRobot = lastRotationMatrix.inverse() * tiltingEdge;

    // 4. Calc Angles of the LIPM
    // cos(angle) = adjacent side / hypo = height / length
    // THIS IS NOT EVEN REMOTELY A LIPM!!! - A.H. 7.1.2022
    //calc old angle
    const Vector3f oldDiff = lastComInRobot - tiltingEdgeInOldRobot;
    const Vector2a oldAngle(-sgnPos(oldDiff.y()) * std::acos(oldDiff.z() / oldDiff.tail<2>().norm()),
                            sgnPos(oldDiff.x()) * std::acos(oldDiff.z() / Vector2f(oldDiff.x(), oldDiff.z()).norm()));

    //calc current angle
    const Vector3f currentDiff = currentComInRobot - tiltingEdgeInCurrentRobot;
    const Vector2a angle(-sgnPos(currentDiff.y()) * std::acos(currentDiff.z() / currentDiff.tail<2>().norm()),
                         sgnPos(currentDiff.x()) * std::acos(currentDiff.z() / Vector2f(currentDiff.x(), currentDiff.z()).norm()));

    // 5. calc acc
    const float length = currentDiff.tail<2>().norm();
    // THIS IS JUST COMPLETELY WRONG!!! - A.H. 7.1.2022
    const Vector2a acc(Angle::fromDegrees(-Constants::g / length * angle.x() * Constants::motionCycleTime),
                       Angle::fromDegrees(-Constants::g / length * angle.y() * Constants::motionCycleTime));

    // 6. calc velocity
    const Vector2a vel = (angle - oldAngle) - acc;

    // 7. update rotationMatrix
    rotationMatrix.rotateX(-vel.x());
    rotationMatrix.rotateY(-vel.y());

    // 8. save values for next iteration
    lastComInFoot = currentComInFoot;
    lastComInRobot = currentComInRobot;
    lastRotationMatrix = currentRotationMatrix;
  }
  return rotationMatrix;
}

bool WalkStepAdjustment::getTiltingPoint(const Vector3f& currentCom, const Vector3f& lastCom, Vector3f& intersection3D)
{
  //code copied from FallDownStateProvider. direction is calculated different, rest is the same.
  const Vector2f direction = (currentCom - lastCom).head<2>();
  const Geometry::Line fallDirectionLine = Geometry::Line(supportFootCenter, direction.normalized());

  CROSS3D("module:WalkStepAdjustment:footMid", currentCom.x(), currentCom.y(), supportPolygon[0].z(), 3, 3, ColorRGBA::violet);
  LINE3D("module:WalkStepAdjustment:footMid", currentCom.x(), currentCom.y(), supportPolygon[0].z(), currentCom.x() + 10.f * direction.x(), currentCom.y() + 10.f * direction.y(), supportPolygon[0].z(), 3, ColorRGBA::violet);

  // search for the intersection with the support foot polygon and the line
  for(size_t i = 0; i < supportPolygon.size(); ++i)
  {
    // check if the line intersect a line from the two points of the polygon
    Vector2f intersection2D;
    const Vector3f& p1 = supportPolygon[i];
    const Vector3f& p2 = supportPolygon[(i + 1) % supportPolygon.size()];
    const Vector2f& base = p1.head<2>();
    const Vector2f dir = p2.head<2>() - base;
    const Geometry::Line polygonLine(base, dir.normalized());

    if(Geometry::isPointLeftOfLine(fallDirectionLine.base, fallDirectionLine.base + fallDirectionLine.direction, base)
       != Geometry::isPointLeftOfLine(fallDirectionLine.base, fallDirectionLine.base + fallDirectionLine.direction, p2.head<2>())
       && Geometry::getIntersectionOfLines(fallDirectionLine, polygonLine, intersection2D)
       && (supportFootCenter - intersection2D).norm() > (supportFootCenter + fallDirectionLine.direction - intersection2D).norm())
    {
      LINE3D("module:WalkStepAdjustment:footMid", p1.x(), p1.y(), p1.z(), p2.x(), p2.y(), p2.z(), 3, ColorRGBA::blue);
      const float scalar = (intersection2D - base).norm() / dir.norm();
      intersection3D = p1 + scalar * (p2 - p1);
      CROSS3D("module:WalkStepAdjustment:footMid", intersection3D.x(), intersection3D.y(), supportPolygon[0].z(), 3, 3, ColorRGBA::orange);

      return true;
    }
    LINE3D("module:WalkStepAdjustment:footMid", p1.x(), p1.y(), p1.z(), p2.x(), p2.y(), p2.z(), 3, ColorRGBA::green);
  }
  return false;
}

void WalkStepAdjustment::calcSupportPolygon(const RobotModel& robotModel, const FootOffset& footOffset, const bool isLeftPhase)
{
  supportPolygon = std::vector<Vector3f>();
  const Pose3f& useFoot = !isLeftPhase ? robotModel.soleLeft : robotModel.soleRight;
  const float left = !isLeftPhase ? footOffset.leftFoot.left : footOffset.rightFoot.left;
  const float right = !isLeftPhase ? footOffset.leftFoot.right : footOffset.rightFoot.right;
  supportPolygon.push_back(((useFoot + Vector3f(footOffset.forward, left, 0.f))).translation);
  supportPolygon.push_back(((useFoot + Vector3f(-footOffset.backward, left, 0.f))).translation);
  supportPolygon.push_back(((useFoot + Vector3f(-footOffset.backward, -right, 0.f))).translation);
  supportPolygon.push_back(((useFoot + Vector3f(footOffset.forward, -right, 0.f))).translation);

  // calculate centroid of the convex, counter-clockwise ordered polygon
  const size_t size = supportPolygon.size();
  float area = 0.f, x = 0.f, y = 0.f;
  for(size_t i = 0, j = size - 1; i < size; j = i++)
  {
    const Vector3f& p1 = supportPolygon[i];
    const Vector3f& p2 = supportPolygon[j];
    float f = p1.x() * p2.y() - p2.x() * p1.y();
    area += f;
    x += (p1.x() + p2.x()) * f;
    y += (p1.y() + p2.y()) * f;
  }
  area *= 3.f;
  supportFootCenter << x / area, y / area;
}

void WalkStepAdjustment::addBalance(Pose3f& left, Pose3f& right, const float stepTime, const Vector2f& com, const FootOffset& footOffset,
                                    const Rangef& clipForwardPosition, const bool isLeftPhase, const FootSupport& footSupport,
                                    const FsrData& fsrData, const FrameInfo& frameInfo, const Angle hipRot, const bool isStepAdjustmentAllowed,
                                    const int reduceWalkingSpeedStepAdjustmentSteps, const Vector2f& ball, const float clipAtBallDistanceX,
                                    const float& theJointPlayFactor, const bool groundContact, const WalkStepAdjustmentParams& walkStepParams)
{
  // If
  // - the current support foot is not the correct one,
  // - the feet have no ground contact,
  // - the swing foot has ground contact and the step did not just started
  // then the feet should not get balanced.
  // current support foot and the swing foot pressure are checked, because the swing foot could be the support foot, but with very low pressure.
  // Otherwise the feet are unnecessary balanced (with robot is hold in the air by a human) or the chance, the robot might fall, is even higher

  ///////////////////////////////////
  // 1. Set allowed movement speed //
  ///////////////////////////////////

  const float comX = com.x();
  float useMaxVelX = walkStepParams.maxVelX * Constants::motionCycleTime;
  float useMinVelX = walkStepParams.minVelX * Constants::motionCycleTime;
  float useRemoveSpeedX = walkStepParams.removeSpeedX * Constants::motionCycleTime;
  const bool wrongSupportFoot = footSupport.support < 0.f != isLeftPhase;
  const bool swingFootHasGroundContact = (isLeftPhase && fsrData.legInfo[Legs::left].hasPressure == frameInfo.time) || (!isLeftPhase && fsrData.legInfo[Legs::right].hasPressure == frameInfo.time);
  if(!groundContact)
  {
    useMaxVelX = 0.f;
    useMinVelX = 0.f;
    useRemoveSpeedX = 0.f;
  }
  if(swingFootHasGroundContact && !wrongSupportFoot)
  {
    useMaxVelX = stepTime < 0.2f ? useMaxVelX : 0.f;
    useMinVelX = stepTime < 0.2f ? useMinVelX : 0.f;
    useRemoveSpeedX = stepTime < 0.2f ? useRemoveSpeedX : 0.f;
  }

  if(lastLeft == Pose3f())
  {
    lastLeft = left;
    lastRight = right;
  }

  //////////////////////////////////////////
  // Start with step adjustment algorithm //
  //////////////////////////////////////////

  // 3. copy for later
  const Pose3f originalSwing = isLeftPhase ? left : right;
  const Pose3f originalSupport = !isLeftPhase ? left : right;

  // 4. Get references to swing and support foot
  // swingAdjustment = last[Left|Right]Adjustment = delta{w_t-1} - w_t-1
  float& swingAdjustment = isLeftPhase ? lastLeftAdjustmentX : lastRightAdjustmentX;
  float& supportAdjustment = !isLeftPhase ? lastLeftAdjustmentX : lastRightAdjustmentX;
  // w_t and u_t
  Pose3f& swingFoot = isLeftPhase ? left : right;
  Pose3f& supportFoot = !isLeftPhase ? left : right;
  // bar{w_t-1} and bar{u_t-1}
  // w_t-1 abd u_t-1 are equal to bar{w_t-1} and bar{u_t-1}, when swingAdjustment is subtracted
  const Pose3f& lastSwingFoot = isLeftPhase ? lastLeft : lastRight;

  // 5. Calculate the tolerance range
  // bar{w_t-1}
  const float lastSwingX = lastSwingFoot.translation.x();

  // change of the swing foot based on the planned walk
  const float plannedChange = (swingFoot.translation.x() - (lastSwingFoot.translation.x() - swingAdjustment));

  // deltaReduced = delta{hat{w_t-1}}
  const Rangef adjustmentClip(-useMaxVelX - useRemoveSpeedX + std::min(-plannedChange, 0.f), useMaxVelX + useRemoveSpeedX + std::max(-plannedChange, 0.f));
  const float deltaReduced = -adjustmentClip.limit(swingAdjustment);

  // 2. low pass filter if feet are already adjusted, to reduce damage from noise
  if(std::abs(swingAdjustment + deltaReduced) > 0.1f)
    lowPassFilteredComX = walkStepParams.comLowPassRatio * lowPassFilteredComX + (1.f - walkStepParams.comLowPassRatio) * comX;
  else
    lowPassFilteredComX = comX;

  if(!groundContact)
    lowPassFilteredComX = 0.f;

  // currentPlannedSwingX = hat{w_t}
  const float currentPlannedSwingX = lastSwingX + plannedChange + deltaReduced;

  // Current feet area
  const Rangef toleranceSpecification = Rangef((footOffset.backward + footOffset.forward) * walkStepParams.desiredFootArea.min - footOffset.backward, (footOffset.backward + footOffset.forward) * walkStepParams.desiredFootArea.max - footOffset.backward);

  // toleranceLowerLimit = Delta_min; toleranceUpperLimit = Delta_max;
  const float toleranceLowerLimit = std::min(currentPlannedSwingX, supportFoot.translation.x() - (swingAdjustment + deltaReduced) / 2.f) + toleranceSpecification.min;
  const float toleranceUpperLimit = std::max(currentPlannedSwingX, supportFoot.translation.x() - (swingAdjustment + deltaReduced) / 2.f) + toleranceSpecification.max;

  // when the arms are on the back, the upper body is tilted forward.
  // if hipRot == 0, nothing changes
  const float walkHeight = std::max(supportFoot.translation.z(), swingFoot.translation.z());
  const float deltaMin = Vector2f(toleranceLowerLimit, walkHeight).rotate(-hipRot).x();
  const float deltaMax = Vector2f(toleranceUpperLimit, walkHeight).rotate(-hipRot).x();

  // 6. delta
  delta = isStepAdjustmentAllowed ? std::min(lowPassFilteredComX - deltaMin, 0.f) + std::max(lowPassFilteredComX - deltaMax, 0.f) : 0.f;

  // hat{p_max}; hat{p_min}
  const float pMin = std::min(-useMinVelX, adjustmentClip.min - plannedChange) - std::max(deltaReduced, 0.f);
  const float pMax = std::max(useMinVelX, adjustmentClip.max - plannedChange) - std::min(deltaReduced, 0.f);

  // 7. Apply adjustment
  // Apply swing foot adjustment
  swingAdjustment = (currentPlannedSwingX + std::max(pMin, std::min(pMax, delta))) - swingFoot.translation.x();

  if(std::abs(swingAdjustment) < 0.1f)
    swingAdjustment = 0.f;

  swingFoot.translation.x() += swingAdjustment;

  // clip to make sure the feet pose are reachable
  swingFoot.translation.x() = clipForwardPosition.limit(swingFoot.translation.x());
  swingAdjustment = swingFoot.translation.x() - originalSwing.translation.x();

  // Apply support foot adjustment
  supportAdjustment += std::min(-pMin + useRemoveSpeedX, std::max(-pMax - useRemoveSpeedX, -swingAdjustment / 2.f - supportAdjustment));
  supportFoot.translation.x() += supportAdjustment;

  ////////////////////////////////////////////////////////////////
  // clip to make sure we do not unintentionally touch the ball //
  ////////////////////////////////////////////////////////////////
  // but if the robot would fall, do it anyway
  if(delta < 25.f && !allowTouchingTheBallForBalancing) // Todo 25.f should be a parameter, and maybe change code to still clip, instead of allowing full adjustment after exceeding the 25 mm mark
  {
    const float clipAdjustment = std::max((supportFoot.translation.x() + std::max(ball.x() - clipAtBallDistanceX, 0.f)) - (swingFoot.translation.x() - swingAdjustment), 0.f) / 1.5f;
    const float clippedAdjustment = std::min(clipAdjustment - swingAdjustment, 0.f);
    swingAdjustment += clippedAdjustment;
    supportAdjustment -= clippedAdjustment / 2.f;
    swingFoot.translation.x() += clippedAdjustment;
    supportFoot.translation.x() -= clippedAdjustment / 2.f;
  }
  else
    allowTouchingTheBallForBalancing = true;

  // Save highest adjustments for later
  highestAdjustmentX = (swingAdjustment < 0.f ? -1.f : (swingAdjustment > 0.f ? 1.f : 0.f)) * std::max(std::abs(highestAdjustmentX), std::abs(swingAdjustment)); // save the highest value for the current step
  highestNegativeAdjustmentX = std::min(highestNegativeAdjustmentX, highestAdjustmentX);

  ///////////////////////////////////////////
  // Additional balancing over engineering //
  ///////////////////////////////////////////

  // 8. shall the hip pitch be used to balance?
  Rangef offsetsHipBalance = Rangef((footOffset.backward + footOffset.forward) * (walkStepParams.desiredFootArea.min + walkStepParams.hipBalanceBackwardFootArea) - footOffset.backward, (footOffset.backward + footOffset.forward) * (walkStepParams.desiredFootArea.max - walkStepParams.hipBalanceBackwardFootArea) - footOffset.backward);
  const float toleranceLowerLimitSmall = std::min(currentPlannedSwingX - deltaReduced, supportFoot.translation.x() + (swingAdjustment + deltaReduced) / 2.f) + offsetsHipBalance.min;
  const float toleranceUpperLimitSmall = std::max(currentPlannedSwingX - deltaReduced, supportFoot.translation.x() + (swingAdjustment + deltaReduced) / 2.f) + offsetsHipBalance.max;

  float smallMax = Vector2f(toleranceLowerLimitSmall, walkHeight).rotate(-hipRot).x();
  float smallMin = Vector2f(toleranceUpperLimitSmall, walkHeight).rotate(-hipRot).x();
  hipBalanceIsSafeBackward = 1.f - Rangef::ZeroOneRange().limit((lowPassFilteredComX - smallMax) / (deltaMax - smallMax));
  hipBalanceIsSafeForward = Rangef::ZeroOneRange().limit((lowPassFilteredComX - deltaMin) / (smallMin - deltaMin)) * Rangef::ZeroOneRange().limit((ball.x() - clipAtBallDistanceX) / 20.f); // TODO Parameter

  const Rangef toleranceSide(left.translation.y() - footOffset.leftFoot.right, right.translation.y() + footOffset.leftFoot.left);
  sideBalanceFactor = Rangef::ZeroOneRange().limit((com.y() - toleranceSide.limit(com.y())) / 50.f * (isLeftPhase ? -1.f : 1.f)); // TODO Parameter/RobotDimensions

  // Really worn out robots need a earlier trigger
  const float heel = std::min(currentPlannedSwingX - deltaReduced - footOffset.backward, supportFoot.translation.x() + (swingAdjustment + deltaReduced) / 2.f - footOffset.backward);
  const float toe = std::max(currentPlannedSwingX - deltaReduced + footOffset.forward, supportFoot.translation.x() + (swingAdjustment + deltaReduced) / 2.f + footOffset.forward);

  // Dynamic % threshold based on joint play. The higher the play, the less further back the com must be, to start the joint play gyro balancing
  // With this balancing lever a really bad robot should be able to walk as fast as a good robot
  const float magicBalancingParameter = 0.20f + 0.11f * (1.f - theJointPlayFactor); // TODO move into config and scale with size of (toe - heel)
  if(highestNegativeAdjustmentX < walkStepParams.unstableBackWalkThreshold ||
     (toe - heel) * magicBalancingParameter + heel > comX)
  {
    kneeHipBalanceCounter = 4;
    isForwardBalance = true;
    forwardBalanceWasActive = false;
    balanceComIsForward = 1.f;
  }
  else if(toe - (toe - heel) * magicBalancingParameter < comX)
  {
    kneeHipBalanceCounter = 4;
    backwardBalanceWasActive = false;
    isBackwardBalance = true;
  }
  if(swingAdjustment > 0.f)
  {
    kneeHipBalanceCounter = 4;
    isForwardBalance = true;
    forwardBalanceWasActive = false;
    balanceComIsForward = Rangef::ZeroOneRange().limit(balanceComIsForward + 0.25f); // Todo should be a parameter
  }

  if(isLeftPhase)
    swingFootXTranslationChange = left.translation.x() - lastLeft.translation.x();
  else
    swingFootXTranslationChange = right.translation.x() - lastRight.translation.x();

  // Too much backward adjustment. Reduce walking speed
  if((highestNegativeAdjustmentX < -walkStepParams.unstableWalkThreshold && // threshold check
      frameInfo.getTimeSince(lastNormalStep) < walkStepParams.reduceWalkingSpeedTimeWindow && // only reduce if we adjusted two times backwards in the last time frame
      frameInfo.getTimeSince(lastLargeBackwardStep) < walkStepParams.reduceWalkingSpeedTimeWindow) ||
     (highestAdjustmentX > walkStepParams.unstableWalkThreshold && std::abs(highestAdjustmentX) != highestNegativeAdjustmentX)) // reduce walk speed if we adjusted a lot forward
  {
    if(reduceWalkingSpeed != reduceWalkingSpeedStepAdjustmentSteps)
      ANNOTATION("WalkingEngine", "Reduced Walking Speed!");
    reduceWalkingSpeed = reduceWalkingSpeedStepAdjustmentSteps;
  }

  lastLeft = left;
  lastRight = right;

  ///////////////////
  // Debug Drawings //
  ///////////////////
  COMPLEX_DRAWING3D("module:WalkStepAdjustment:balance")
  {
    Pose3f leftDrawing = left;
    Pose3f rightDrawing = right;
    Pose3f originLeftDrawing = isLeftPhase ? originalSwing : originalSupport;
    Pose3f originRightDrawing = !isLeftPhase ? originalSwing : originalSupport;
    const Vector2f leftShift = Vector2f(left.translation.x(), left.translation.z()).rotate(-hipRot);
    const Vector2f rightShift = Vector2f(right.translation.x(), right.translation.z()).rotate(-hipRot);
    const Vector2f leftOriginShift = Vector2f(originLeftDrawing.translation.x(), originLeftDrawing.translation.z()).rotate(-hipRot);
    const Vector2f rightOriginShift = Vector2f(originRightDrawing.translation.x(), originRightDrawing.translation.z()).rotate(-hipRot);
    leftDrawing.translation.x() = leftShift.x();
    leftDrawing.translation.z() = leftShift.y();
    rightDrawing.translation.x() = rightShift.x();
    rightDrawing.translation.z() = rightShift.y();
    originLeftDrawing.translation.x() = leftOriginShift.x();
    originLeftDrawing.translation.z() = leftOriginShift.y();
    originRightDrawing.translation.x() = rightOriginShift.x();
    originRightDrawing.translation.z() = rightOriginShift.y();

    // Debug Drawings
    const Pose3f maxLeft = (leftDrawing + Vector3f(toleranceSpecification.max, 0.f, 0.f));
    const Pose3f minLeft = (leftDrawing + Vector3f(toleranceSpecification.min, 0.f, 0.f));
    const Pose3f maxRight = (rightDrawing + Vector3f(toleranceSpecification.max, 0.f, 0.f));
    const Pose3f minRight = (rightDrawing + Vector3f(toleranceSpecification.min, 0.f, 0.f));

    const Pose3f maxLeft2 = (leftDrawing + Vector3f(offsetsHipBalance.max, 0.f, 0.f));
    const Pose3f minLeft2 = (leftDrawing + Vector3f(offsetsHipBalance.min, 0.f, 0.f));
    const Pose3f maxRight2 = (rightDrawing + Vector3f(offsetsHipBalance.max, 0.f, 0.f));
    const Pose3f minRight2 = (rightDrawing + Vector3f(offsetsHipBalance.min, 0.f, 0.f));
    LINE3D("module:WalkStepAdjustment:balance", maxLeft.translation.x(), maxLeft.translation.y() - 30.f, maxLeft.translation.z(), maxLeft.translation.x(), maxLeft.translation.y() + 30.f, maxLeft.translation.z(), 3, ColorRGBA::magenta);
    LINE3D("module:WalkStepAdjustment:balance", minLeft.translation.x(), minLeft.translation.y() - 30.f, minLeft.translation.z(), minLeft.translation.x(), minLeft.translation.y() + 30.f, minLeft.translation.z(), 3, ColorRGBA::magenta);
    LINE3D("module:WalkStepAdjustment:balance", maxRight.translation.x(), maxRight.translation.y() - 30.f, maxRight.translation.z(), maxRight.translation.x(), maxRight.translation.y() + 30.f, maxRight.translation.z(), 3, ColorRGBA::magenta);
    LINE3D("module:WalkStepAdjustment:balance", minRight.translation.x(), minRight.translation.y() - 30.f, minRight.translation.z(), minRight.translation.x(), minRight.translation.y() + 30.f, minRight.translation.z(), 3, ColorRGBA::magenta);

    LINE3D("module:WalkStepAdjustment:balance", maxLeft2.translation.x(), maxLeft2.translation.y() - 30.f, maxLeft2.translation.z(), maxLeft2.translation.x(), maxLeft2.translation.y() + 30.f, maxLeft2.translation.z(), 3, ColorRGBA::blue);
    LINE3D("module:WalkStepAdjustment:balance", minLeft2.translation.x(), minLeft2.translation.y() - 30.f, minLeft2.translation.z(), minLeft2.translation.x(), minLeft2.translation.y() + 30.f, minLeft2.translation.z(), 3, ColorRGBA::blue);
    LINE3D("module:WalkStepAdjustment:balance", maxRight2.translation.x(), maxRight2.translation.y() - 30.f, maxRight2.translation.z(), maxRight2.translation.x(), maxRight2.translation.y() + 30.f, maxRight2.translation.z(), 3, ColorRGBA::blue);
    LINE3D("module:WalkStepAdjustment:balance", minRight2.translation.x(), minRight2.translation.y() - 30.f, minRight2.translation.z(), minRight2.translation.x(), minRight2.translation.y() + 30.f, minRight2.translation.z(), 3, ColorRGBA::blue);

    LINE3D("module:WalkStepAdjustment:balance", leftDrawing.translation.x(), leftDrawing.translation.y(), leftDrawing.translation.z(), originLeftDrawing.translation.x(), originLeftDrawing.translation.y(), originLeftDrawing.translation.z(), 2, ColorRGBA::orange);
    LINE3D("module:WalkStepAdjustment:balance", rightDrawing.translation.x(), rightDrawing.translation.y(), rightDrawing.translation.z(), originRightDrawing.translation.x(), originRightDrawing.translation.y(), originRightDrawing.translation.z(), 2, ColorRGBA::orange);
    CROSS3D("module:WalkStepAdjustment:balance", leftDrawing.translation.x(), leftDrawing.translation.y(), leftDrawing.translation.z(), 3, 3, ColorRGBA::black);
    CROSS3D("module:WalkStepAdjustment:balance", rightDrawing.translation.x(), rightDrawing.translation.y(), rightDrawing.translation.z(), 3, 3, ColorRGBA::black);
  }

  PLOT("module:WalkStepAdjustment:Data:leftAdjustment", lastLeftAdjustmentX);
  PLOT("module:WalkStepAdjustment:Data:rightAdjustment", lastRightAdjustmentX);
}

void WalkStepAdjustment::init(const WalkStepAdjustment& walkData, const bool reset, const FrameInfo& frameInfo,
                              const float unstableWalkThreshold)
{
  // For the LIPM save values from the last motion frame
  lastMeasuredCom = walkData.lastMeasuredCom;
  lastMeasuredComInFoot = walkData.lastMeasuredComInFoot;
  lastMeasuredRotationMatrix = walkData.lastMeasuredRotationMatrix;

  if(reset)
  {
    kneeHipBalanceCounter = 0;
    delta = 0.f;
    return;
  }

  // For balancing
  lastLeft = walkData.lastLeft;
  lastRight = walkData.lastRight;
  lowPassFilteredComX = walkData.lowPassFilteredComX;
  previousHighestAdjustmentX = walkData.highestAdjustmentX;
  kneeHipBalanceCounter = walkData.kneeHipBalanceCounter;
  isForwardBalance = walkData.isForwardBalance;
  isBackwardBalance = walkData.isBackwardBalance;
  reduceWalkingSpeed = walkData.reduceWalkingSpeed;
  backwardsWalkingRotationCompensationFactor = walkData.backwardsWalkingRotationCompensationFactor;
  forwardsWalkingRotationCompensationFactor = walkData.forwardsWalkingRotationCompensationFactor;
  forwardBalanceWasActive = walkData.forwardBalanceWasActive;
  backwardBalanceWasActive = walkData.backwardBalanceWasActive;
  kneeHipBalanceCounter--;
  reduceWalkingSpeed--;
  delta = walkData.delta;

  // For walk speed reduction
  lastLargeBackwardStep = walkData.lastLargeBackwardStep;
  lastNormalStep = walkData.lastNormalStep;

  if(previousHighestAdjustmentX < -unstableWalkThreshold)
    lastLargeBackwardStep = frameInfo.time;
  if(previousHighestAdjustmentX >= 0.f)
    lastNormalStep = frameInfo.time;

  // make sure this is still low pass filtered, when balancing (otherwise for one frame the newest value is used and not the filtered one)
  lastLeftAdjustmentX = walkData.lastLeftAdjustmentX != 0.f ? (walkData.lastLeftAdjustmentX > 0.f ? 0.001f : -0.001f) : 0.f;
  lastRightAdjustmentX = walkData.lastRightAdjustmentX != 0.f ? (walkData.lastRightAdjustmentX > 0.f ? 0.001f : -0.001f) : 0.f;
  backwardsWalkingRotationCompensationFactor *= 0.5f;
  forwardsWalkingRotationCompensationFactor *= 0.5f;
}

void WalkStepAdjustment::modifySwingFootRotation(const Pose3f& supportSole, const Pose3f& requestedSupportSole, Angle& swingRotation, const Angle oldSwingRotation,
                                                 const Angle torsoRotation, const float stepRatio, const Angle soleRotationOffsetSpeed,
                                                 const bool useTorsoAngle, const Angle hipRotation,
                                                 const SoleRotationParameter& soleRotParams, const Pose3f& swingSole)
{
  ASSERT(soleRotParams.minTorsoRotation > 0_deg);
  ASSERT(soleRotParams.minSoleRotation > 0_deg);
  forwardsWalkingRotationCompensationFactor = Rangef::ZeroOneRange().limit(swingFootXTranslationChange <= 0.f ? forwardsWalkingRotationCompensationFactor - soleRotParams.soleCompensationReduction : forwardsWalkingRotationCompensationFactor + soleRotParams.soleCompensationIncreasement);
  backwardsWalkingRotationCompensationFactor = Rangef::ZeroOneRange().limit(swingFootXTranslationChange >= 0.f ? backwardsWalkingRotationCompensationFactor - soleRotParams.soleCompensationReduction : backwardsWalkingRotationCompensationFactor + soleRotParams.soleCompensationIncreasement);
  const float useBackwardScaling = (1.f - Rangef::ZeroOneRange().limit((requestedSupportSole.translation.x() - (swingSole.translation.x() - 20.f)) / -20.f)) * backwardsWalkingRotationCompensationFactor;
  const Angle supportSoleRotError = supportSole.rotation.getYAngle() - (requestedSupportSole.rotation.getYAngle() + hipRotation);
  if(torsoRotation + (hipRotation - std::min(supportSoleRotError, 0_deg)) > 0_deg)
  {
    const float torsoFactor = Rangef::ZeroOneRange().limit((torsoRotation + hipRotation - std::min(supportSoleRotError, 0_deg)) / soleRotParams. minTorsoRotation); // torso angle clip
    const Rangea limitAnkleOffsetFast(std::min(0.f, oldSwingRotation - swingRotation) - soleRotationOffsetSpeed * Constants::motionCycleTime * 2.f, soleRotationOffsetSpeed * Constants::motionCycleTime); // allowed angular speed
    Angle supportSoleRotation = std::min(supportSoleRotError, 0_deg); // y sole rotation error
    if(useTorsoAngle)  // only use torso, if the robot is fully calibrated
      supportSoleRotation = std::min(Angle(((torsoRotation + hipRotation)) * -0.5f), supportSoleRotation);
    supportSoleRotation *= Rangef::ZeroOneRange().limit(sqr(std::min(0.f, supportSoleRotation / soleRotParams.minSoleRotation))); // rotation clip
    supportSoleRotation *= stepRatio < soleRotParams.soleForwardCompensationReturnZeroRation ? 1.f : 1.f - std::min(1.f, stepRatio); // time scaling
    supportSoleRotation *= torsoFactor;
    supportSoleRotation *= forwardsWalkingRotationCompensationFactor;
    PLOT("module:WalkingEngine:Data:supportSoleRotationY", Angle(supportSole.rotation.getYAngle()).toDegrees());
    swingRotation += limitAnkleOffsetFast.limit(supportSoleRotation - swingRotation);  // apply correction
  }
  else
  {
    const float torsoFactor = Rangef::ZeroOneRange().limit(sqr((torsoRotation + hipRotation) / -soleRotParams.minTorsoRotation)); // torso angle clip
    const Rangea limitAnkleOffsetFast(-soleRotationOffsetSpeed * Constants::motionCycleTime * (1.f + torsoFactor * 3.f), std::max(0.f, oldSwingRotation - swingRotation) + soleRotationOffsetSpeed * Constants::motionCycleTime * (2.f + torsoFactor * 4.f));  // allowed angular speed
    // Use the max rotation error from the torso and the sole
    Angle torsoAngleCompensation = std::max(supportSoleRotError, 0_deg);
    if(useTorsoAngle) // only use torso, if the robot is fully calibrated
      torsoAngleCompensation = std::min((torsoRotation + hipRotation) * -soleRotParams.soleBackwardsCompensationTorsoFactor, soleRotParams.maxBackCompensation + torsoAngleCompensation);
    torsoAngleCompensation *= stepRatio < soleRotParams.soleBackwardsCompensationReturnZeroRatio ? 1.f : 1.f - std::min(1.f, (stepRatio - soleRotParams.soleBackwardsCompensationReturnZeroRatio) * 2.f); // time scaling
    torsoAngleCompensation *= torsoFactor;
    torsoAngleCompensation *= useBackwardScaling;
    PLOT("module:WalkingEngine:Data:supportSoleRotationY", Angle(supportSole.rotation.getYAngle()).toDegrees());
    swingRotation += limitAnkleOffsetFast.limit(torsoAngleCompensation - swingRotation);  // apply correction
  }

  PLOT("module:WalkingEngine:Data:supportSoleRotationCompensationY", swingRotation.toDegrees());
}
