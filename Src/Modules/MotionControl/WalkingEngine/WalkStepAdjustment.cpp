/*
 * @file WalkStepAdjustment.cpp
 * This file implements something
 * @author Philip Reichenberg
 */

#include "WalkStepAdjustment.h"
#include "Tools/Debugging/Annotation.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Rotation.h"
#include <cmath>

WalkStepAdjustment::WalkStepAdjustment()
{
  lastLeftAdjustmentX = 0.f;
  lastRightAdjustmentX = 0.f;
  highestNegativeAdjustmentX = 0.f;
  highestAdjustmentX = 0.f;
  hipBalanceCounter = 100;
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
Pose3f WalkStepAdjustment::predictRotation(Pose3f& rotationMatrix, const RobotModel& robotModel, const FootOffset& footOffset, const bool isLeftPhase, const bool prediction)
{
  // pre calculation. Get current CoM position and init last values. Also calculate the support polygon
  Pose3f lastComInFoot = lastMeasuredComInFoot;
  Pose3f lastCom = lastMeasuredCom;
  lastMeasuredCom = rotationMatrix.inverse() * robotModel.centerOfMass;
  Pose3f lastRotationMatrix = lastMeasuredRotationMatrix;
  lastMeasuredRotationMatrix = rotationMatrix;
  calcSupportPolygon(robotModel, footOffset, isLeftPhase);
  for(int i = 0; i < (prediction ? updateSteps : 1); i++)
  {
    // 1. calc current com relative to foot plane
    if(lastRotationMatrix == Pose3f())
      lastRotationMatrix = rotationMatrix; // robot just started to walk
    const Pose3f currentRotationMatrix = rotationMatrix;
    const Pose3f currentCom = rotationMatrix.inverse() * robotModel.centerOfMass;
    Pose3f currentComInFoot = currentCom;
    currentComInFoot.translation.z() = isLeftPhase ? (rotationMatrix.inverse() * robotModel.soleRight).translation.z() : (rotationMatrix.inverse() * robotModel.soleLeft).translation.z();
    currentComInFoot = rotationMatrix.rotation * currentComInFoot;
    if(lastComInFoot == Pose3f()) // robot just started to walk
    {
      lastComInFoot = currentComInFoot;
      lastMeasuredComInFoot = currentComInFoot;
    }
    // 2. calc tilting edge
    Vector3f tiltingEdge = getTiltingPoint(currentComInFoot, lastComInFoot);
    if(std::isnan(tiltingEdge.x()) || std::isnan(tiltingEdge.y()) || std::isnan(tiltingEdge.z()))
      return rotationMatrix;
    if(i == 0)
      lastMeasuredComInFoot = currentComInFoot;

    // 3. convert into robot coordinates
    const Vector3f tiltingEdgeInCurrentRobot = rotationMatrix.inverse() * tiltingEdge;
    const Vector3f tiltingEdgeInOldRobot = lastRotationMatrix.inverse() * tiltingEdge;
    const Vector3f currentComInRobot = currentCom.translation;
    const Vector3f lastComInRobot = lastCom.translation;

    // 4. Calc Angles of the LIPM
    // cos(angle) = adjacent side / hypo = height / length
    //calc old angle
    Vector2a oldAngle;
    oldAngle.x() = std::acos((lastComInRobot.z() - tiltingEdgeInOldRobot.z()) / (Vector2f(lastComInRobot.y(), lastComInRobot.z()) - Vector2f(tiltingEdgeInOldRobot.y(), tiltingEdgeInOldRobot.z())).norm());
    oldAngle.x() *= tiltingEdgeInOldRobot.y() - lastComInRobot.y() > 0 ? 1.f : -1.f;
    oldAngle.y() = std::acos((lastComInRobot.z() - tiltingEdgeInOldRobot.z()) / (Vector2f(lastComInRobot.x(), lastComInRobot.z()) - Vector2f(tiltingEdgeInOldRobot.x(), tiltingEdgeInOldRobot.z())).norm());
    oldAngle.y() *= tiltingEdgeInOldRobot.x() - lastComInRobot.x() > 0 ? -1.f : 1.f;

    //calc current angle
    Vector2a angle;
    angle.x() = std::acos((currentComInRobot.z() - tiltingEdgeInCurrentRobot.z()) / (Vector2f(currentComInRobot.y(), currentComInRobot.z()) - Vector2f(tiltingEdgeInCurrentRobot.y(), tiltingEdgeInCurrentRobot.z())).norm());
    angle.x() *= tiltingEdgeInCurrentRobot.y() - currentComInRobot.y() > 0 ? 1.f : -1.f;
    angle.y() = std::acos((currentComInRobot.z() - tiltingEdgeInCurrentRobot.z()) / (Vector2f(currentComInRobot.x(), currentComInRobot.z()) - Vector2f(tiltingEdgeInCurrentRobot.x(), tiltingEdgeInCurrentRobot.z())).norm());
    angle.y() *= tiltingEdgeInCurrentRobot.x() - currentComInRobot.x() > 0 ? -1.f : 1.f;

    // 5. calc velocity
    Vector2a vel = angle - oldAngle;

    // 6. calc acc
    const float length = (Vector2f(currentComInRobot.y(), currentComInRobot.z()) - Vector2f(tiltingEdgeInCurrentRobot.y(), tiltingEdgeInCurrentRobot.z())).norm();
    Vector2a acc;
    acc.x() = Angle::fromDegrees(-Constants::g / length * angle.x() * Constants::motionCycleTime);
    acc.y() = Angle::fromDegrees(-Constants::g / length * angle.y() * Constants::motionCycleTime);

    //update velocity
    vel -= acc;

    // 7. update rotationMatrix
    rotationMatrix.rotateX(-vel.x());
    rotationMatrix.rotateY(-vel.y());

    // 8. save values for next iteration
    lastComInFoot = currentComInFoot;
    lastCom = currentCom;
    lastRotationMatrix = currentRotationMatrix;
  }
  return rotationMatrix;
}

Vector3f WalkStepAdjustment::getTiltingPoint(const Pose3f& currentCom, const Pose3f& lastCom)
{
  //code copied from FallDownStateProvider. direction is calculated different, rest is the same.
  const Vector2f direction = (currentCom.translation - lastCom.translation).head<2>();
  const Geometry::Line fallDirectionLine = Geometry::Line(Vector2f(supportFootCenter), direction.normalized());

  CROSS3D("module:WalkStepAdjustment:footMid", currentCom.translation.x(), currentCom.translation.y(), supportPolygon[0].z(), 3, 3, ColorRGBA::violet);
  LINE3D("module:WalkStepAdjustment:footMid", currentCom.translation.x(), currentCom.translation.y(), supportPolygon[0].z(), currentCom.translation.x() + 10.f * direction.x(), currentCom.translation.y() + 10.f * direction.y(), supportPolygon[0].z(), 3, ColorRGBA::violet);

  // search for the intersection with the support foot polygon and the line
  Vector3f intersection3D;
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
      float scalar = (intersection2D - base).norm() / dir.norm();
      intersection3D = p1 + scalar * (p2 - p1);
      CROSS3D("module:WalkStepAdjustment:footMid", intersection3D.x(), intersection3D.y(), supportPolygon[0].z(), 3, 3, ColorRGBA::orange);

      break;
    }
    LINE3D("module:WalkStepAdjustment:footMid", p1.x(), p1.y(), p1.z(), p2.x(), p2.y(), p2.z(), 3, ColorRGBA::green);
  }
  return intersection3D;
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

void WalkStepAdjustment::addBalance(Pose3f& left, Pose3f& right, const float stepTime, const float comX, const float unstableBackWalkThreshold,
                                    const Rangef& footArea, const float hipBalanceBackwardFootArea, const FootOffset& footOffset,
                                    const float maxVelX, const float minVelX, const float removeSpeedX,
                                    const float comLowPassRatio, const Rangef& clipForwardPosition, const bool isLeftPhase,
                                    const FootSupport& footSupport, const FrameInfo& frameInfo, const Angle hipRot, const bool isStepAdjustmentAllowed,
                                    const float unstableWalkThreshold, const int reduceWalkingSpeedTimeWindow, const int reduceWalkingSpeedStepAdjustmentSteps,
                                    const Vector2f& ball, const float clipAtBallDistanceX)
{
  // If
  // - the current support foot is not the correct one,
  // - the feet have no ground contact,
  // - the swing foot has ground contact and the step did not just started
  // then the feet should not get balanced.
  // current support foot and the swing foot pressure are checked, because the swing foot could be the support foot, but with very low pressure.
  // Otherwise the feet are unnecessary balanced (with robot is hold in the air by a human) or the chance, the robot might fall, is even higher

  // 1. Set allowed movement speed
  float useMaxVelX = maxVelX * Constants::motionCycleTime;
  float useMinVelX = minVelX * Constants::motionCycleTime;
  float useRemoveSpeedX = removeSpeedX * Constants::motionCycleTime;
  const bool wrongSupportFoot = footSupport.support < 0.f != isLeftPhase;
  const bool swingFootHasGroundContact = (isLeftPhase && footSupport.footPressure[Legs::left].hasPressure == frameInfo.time) || (!isLeftPhase && footSupport.footPressure[Legs::right].hasPressure == frameInfo.time);
  const bool noGroundContact = std::max(footSupport.footPressure[Legs::left].hasPressure, footSupport.footPressure[Legs::right].hasPressure) != frameInfo.time;
  if(wrongSupportFoot || noGroundContact)
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

  // 2. copy for later
  const Pose3f originalSwing = isLeftPhase ? left : right;
  const Pose3f originalSupport = !isLeftPhase ? left : right;

  // 3. low pass filter if feet are already adjusted, to reduce damage from noise
  if(lastLeftAdjustmentX != 0.f || lastRightAdjustmentX != 0.f)
    lowPassFilteredComX = comLowPassRatio * lowPassFilteredComX + (1.f - comLowPassRatio) * comX;
  else
    lowPassFilteredComX = comX;

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

  // currentPlannedSwingX = hat{w_t}
  const float currentPlannedSwingX = lastSwingX + plannedChange + deltaReduced;

  // Current feet area
  const Rangef toleranceSpecification = Rangef((footOffset.backward + footOffset.forward) * footArea.min - footOffset.backward, (footOffset.backward + footOffset.forward) * footArea.max - footOffset.backward);

  // toleranceLowerLimit = Delta_min; toleranceUpperLimit = Delta_max;
  const float toleranceLowerLimit = std::min(currentPlannedSwingX, supportFoot.translation.x() - (swingAdjustment + deltaReduced) / 2.f) + toleranceSpecification.min;
  const float toleranceUpperLimit = std::max(currentPlannedSwingX, supportFoot.translation.x() - (swingAdjustment + deltaReduced) / 2.f) + toleranceSpecification.max;

  // when the arms are on the back, the upper body is tilted forward.
  // if hipRot == 0, nothing changes
  const float walkHeight = std::max(supportFoot.translation.z(), swingFoot.translation.z());
  const float deltaMin = Vector2f(toleranceLowerLimit, walkHeight).rotate(-hipRot).x();
  const float deltaMax = Vector2f(toleranceUpperLimit, walkHeight).rotate(-hipRot).x();

  // 6. delta
  const float delta = isStepAdjustmentAllowed ? std::min(lowPassFilteredComX - deltaMin, 0.f) + std::max(lowPassFilteredComX - deltaMax, 0.f) : 0.f;

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

  // clip to make sure we do not unintentionally touch the ball
  const float clipAdjustment = std::max((supportFoot.translation.x() + std::max(ball.x() - clipAtBallDistanceX, 0.f) / 1.5f) - swingFoot.translation.x(), 0.f);
  const float clippedAdjustment = std::min(clipAdjustment - swingAdjustment, 0.f);
  swingAdjustment += clippedAdjustment;
  swingFoot.translation.x() += clippedAdjustment;

  // Apply support foot adjustment
  supportAdjustment = -swingAdjustment / 2.f;
  supportFoot.translation.x() += supportAdjustment;

  // Save highest adjustments for later
  highestAdjustmentX = (swingAdjustment < 0.f ? -1.f : (swingAdjustment > 0.f ? 1.f : 0.f)) * std::max(std::abs(highestAdjustmentX), std::abs(swingAdjustment)); // save the highest value for the current step
  highestNegativeAdjustmentX = std::min(highestNegativeAdjustmentX, highestAdjustmentX);

  // 8. shall the hip pitch be used to balance?
  Rangef offsetsHipBalance = Rangef((footOffset.backward + footOffset.forward) * (footArea.min + hipBalanceBackwardFootArea) - footOffset.backward, (footOffset.backward + footOffset.forward) * (footArea.min - hipBalanceBackwardFootArea) - footOffset.backward);
  const float toleranceLowerLimitSmall = std::min(currentPlannedSwingX - deltaReduced, supportFoot.translation.x() + (swingAdjustment + deltaReduced) / 2.f) + offsetsHipBalance.min;
  const float toleranceUpperLimitSmall = std::max(currentPlannedSwingX - deltaReduced, supportFoot.translation.x() + (swingAdjustment + deltaReduced) / 2.f) + offsetsHipBalance.max;

  float smallMax = Vector2f(toleranceLowerLimitSmall, walkHeight).rotate(-hipRot).x();
  float smallMin = Vector2f(toleranceUpperLimitSmall, walkHeight).rotate(-hipRot).x();
  hipBalanceIsSafeBackward = 1.f - Rangef::ZeroOneRange().limit((lowPassFilteredComX - smallMax) / (deltaMax - smallMax));
  hipBalanceIsSafeForward = Rangef::ZeroOneRange().limit((lowPassFilteredComX - deltaMin) / (smallMin - deltaMin));

  // Really worn out robots need a earlier trigger
  const float heel = std::min(currentPlannedSwingX - deltaReduced - footOffset.backward, supportFoot.translation.x() + (swingAdjustment + deltaReduced) / 2.f - footOffset.backward);
  const float toe = std::max(currentPlannedSwingX - deltaReduced + footOffset.forward, supportFoot.translation.x() + (swingAdjustment + deltaReduced) / 2.f + footOffset.forward);

  if((highestNegativeAdjustmentX < unstableBackWalkThreshold) ||
     ((toe - heel) * 0.20f + heel > comX))
    kneeHipBalanceCounter = 4;

  // save for next computation
  if(isLeftPhase)
    swingFootXTranslationChange = left.translation.x() - lastLeft.translation.x();
  else
    swingFootXTranslationChange = right.translation.x() - lastRight.translation.x();

  // Too much backward adjustment. Reduce walking speed
  if(highestNegativeAdjustmentX < -unstableWalkThreshold && lastNormalStep > lastLargeBackwardStep && frameInfo.getTimeSince(lastNormalStep) < reduceWalkingSpeedTimeWindow && frameInfo.getTimeSince(lastLargeBackwardStep) < reduceWalkingSpeedTimeWindow)
  {
    if(reduceWalkingSpeed != reduceWalkingSpeedStepAdjustmentSteps)
      ANNOTATION("WalkingEngine", "Reduced Walking Speed!");
    reduceWalkingSpeed = reduceWalkingSpeedStepAdjustmentSteps;
  }

  lastLeft = left;
  lastRight = right;

  // debug drawing
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

    LINE3D("module:WalkStepAdjustment:balance", maxLeft.translation.x(), maxLeft.translation.y() - 30.f, maxLeft.translation.z(), maxLeft.translation.x(), maxLeft.translation.y() + 30.f, maxLeft.translation.z(), 3, ColorRGBA::magenta);
    LINE3D("module:WalkStepAdjustment:balance", minLeft.translation.x(), minLeft.translation.y() - 30.f, minLeft.translation.z(), minLeft.translation.x(), minLeft.translation.y() + 30.f, minLeft.translation.z(), 3, ColorRGBA::magenta);
    LINE3D("module:WalkStepAdjustment:balance", maxRight.translation.x(), maxRight.translation.y() - 30.f, maxRight.translation.z(), maxRight.translation.x(), maxRight.translation.y() + 30.f, maxRight.translation.z(), 3, ColorRGBA::magenta);
    LINE3D("module:WalkStepAdjustment:balance", minRight.translation.x(), minRight.translation.y() - 30.f, minRight.translation.z(), minRight.translation.x(), minRight.translation.y() + 30.f, minRight.translation.z(), 3, ColorRGBA::magenta);

    LINE3D("module:WalkStepAdjustment:balance", leftDrawing.translation.x(), leftDrawing.translation.y(), leftDrawing.translation.z(), originLeftDrawing.translation.x(), originLeftDrawing.translation.y(), originLeftDrawing.translation.z(), 2, ColorRGBA::orange);
    LINE3D("module:WalkStepAdjustment:balance", rightDrawing.translation.x(), rightDrawing.translation.y(), rightDrawing.translation.z(), originRightDrawing.translation.x(), originRightDrawing.translation.y(), originRightDrawing.translation.z(), 2, ColorRGBA::orange);
    CROSS3D("module:WalkStepAdjustment:balance", leftDrawing.translation.x(), leftDrawing.translation.y(), leftDrawing.translation.z(), 3, 3, ColorRGBA::black);
    CROSS3D("module:WalkStepAdjustment:balance", rightDrawing.translation.x(), rightDrawing.translation.y(), rightDrawing.translation.z(), 3, 3, ColorRGBA::black);
  }

  PLOT("module:WalkStepAdjustment:Data:leftAdjustment", lastLeftAdjustmentX);
  PLOT("module:WalkStepAdjustment:Data:rightAdjustment", lastRightAdjustmentX);
}

void WalkStepAdjustment::init(const WalkStepAdjustment& walkData, HipGyroBalanceState& hipBalance,
                              const int hipBalanceSteps, const bool reset, const FrameInfo& frameInfo,
                              const float unstableWalkThreshold)
{
  // For the LIPM save values from the last motion frame
  lastMeasuredCom = walkData.lastMeasuredCom;
  lastMeasuredComInFoot = walkData.lastMeasuredComInFoot;
  lastMeasuredRotationMatrix = walkData.lastMeasuredRotationMatrix;

  if(reset)
  {
    hipBalance = noHipBalance;
    hipBalanceCounter = 100;
    kneeHipBalanceCounter = 0;
    return;
  }

  // For balancing
  lastLeft = walkData.lastLeft;
  lastRight = walkData.lastRight;
  lowPassFilteredComX = walkData.lowPassFilteredComX;
  previousHighestAdjustmentX = walkData.highestAdjustmentX;
  hipBalanceCounter = walkData.hipBalanceCounter;
  kneeHipBalanceCounter = walkData.kneeHipBalanceCounter;
  reduceWalkingSpeed = walkData.reduceWalkingSpeed;
  kneeHipBalanceCounter--;
  hipBalanceCounter++;
  reduceWalkingSpeed--;

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

  const float lastAdjustment = std::abs(walkData.lastLeftAdjustmentX) > std::abs(walkData.lastRightAdjustmentX) ? walkData.lastLeftAdjustmentX : walkData.lastRightAdjustmentX;
  if(lastAdjustment < 0.f)
    backwardsWalkingRotationCompensationFactor = 1.f;

  // for the hip balancing
  if(walkData.highestAdjustmentX * walkData.previousHighestAdjustmentX > 0.f)
  {
    if(walkData.highestAdjustmentX > 0.f && walkData.previousHighestAdjustmentX > 0.f)
    {
      hipBalanceCounter = 0;
      hipBalance = backwardHipBalance;
    }
  }
  if(hipBalanceCounter >= hipBalanceSteps)
    hipBalance = noHipBalance;
}

void WalkStepAdjustment::modifySwingFootRotation(const Pose3f& supportSole, const Pose3f& requestedSupportSole, Angle& swingRotation, const Angle oldSwingRotation,
                                                 const Angle torsoRotation, const float stepRatio, const Angle soleRotationOffsetSpeed,
                                                 const Angle minTorsoRotation, const Angle minSoleRotation, const bool useTorsoAngle, const bool isLeftPhase,
                                                 const float soleBackwardsCompensationTorsoFactor, const float soleForwardCompensationReturnZeroRation,
                                                 const float soleBackwardsCompensationReturnZeroRatio, const float soleBackwardsCompensationFeetXDifference,
                                                 const float soleBackwardsCompensationFeetShift, const float soleCompensationReduction, const float soleCompensationIncreasement)
{
  ASSERT(minTorsoRotation > 0_deg);
  ASSERT(minSoleRotation > 0_deg);
  forwardsWalkingRotationCompensationFactor = Rangef::ZeroOneRange().limit(swingFootXTranslationChange < 0.f ? forwardsWalkingRotationCompensationFactor - soleCompensationReduction : forwardsWalkingRotationCompensationFactor + soleCompensationIncreasement);
  backwardsWalkingRotationCompensationFactor = Rangef::ZeroOneRange().limit(swingFootXTranslationChange > 0.f ? backwardsWalkingRotationCompensationFactor - soleCompensationReduction : backwardsWalkingRotationCompensationFactor + soleCompensationIncreasement);

  if(torsoRotation > 0_deg)
  {
    const float torsoFactor = Rangef::ZeroOneRange().limit(torsoRotation / minTorsoRotation); // torso angle clip
    const Rangea limitAnkleOffsetFast(std::min(0.f, oldSwingRotation - swingRotation) - soleRotationOffsetSpeed * Constants::motionCycleTime * 2.f, soleRotationOffsetSpeed * Constants::motionCycleTime); // allowed angular speed
    Angle supportSoleRotation = std::min(supportSole.rotation.getYAngle() - requestedSupportSole.rotation.getYAngle(), 0.f); // y sole rotation error
    supportSoleRotation *= Rangef::ZeroOneRange().limit(sqr(std::min(0.f, supportSoleRotation / minSoleRotation))); // rotation clip
    supportSoleRotation *= stepRatio < soleForwardCompensationReturnZeroRation ? 1.f : 1.f - std::min(1.f, stepRatio); // time scaling
    supportSoleRotation *= torsoFactor;
    supportSoleRotation *= forwardsWalkingRotationCompensationFactor;
    PLOT("module:WalkStepAdjustment:Data:supportSoleRotationY", Angle(supportSole.rotation.getYAngle()).toDegrees());
    swingRotation += limitAnkleOffsetFast.limit(supportSoleRotation - swingRotation); // apply correction
  }
  else
  {
    const float torsoFactor = Rangef::ZeroOneRange().limit(sqr(torsoRotation / -minTorsoRotation)); // torso angle clip
    const Rangea limitAnkleOffsetFast(- soleRotationOffsetSpeed * Constants::motionCycleTime, std::max(0.f, oldSwingRotation - swingRotation) + soleRotationOffsetSpeed * Constants::motionCycleTime * 2.f); // allowed angular speed
    // Use the max rotation error from the torso and the sole
    Angle torsoAngleCompensation = std::max(supportSole.rotation.getYAngle() - requestedSupportSole.rotation.getYAngle(), 0.f);
    if(useTorsoAngle) // only use torso, if the robot is fully calibrated
      torsoAngleCompensation = std::max(Angle((torsoRotation + std::max(requestedSupportSole.rotation.getYAngle(), 0.f)) * -soleBackwardsCompensationTorsoFactor), torsoAngleCompensation);
    torsoAngleCompensation *= stepRatio < soleBackwardsCompensationReturnZeroRatio ? 1.f : 1.f - std::min(1.f, stepRatio); // time scaling
    torsoAngleCompensation *= torsoFactor;
    torsoAngleCompensation *= backwardsWalkingRotationCompensationFactor;
    const float swingFactor = 1.f - Rangef::ZeroOneRange().limit(((isLeftPhase ? lastLeft.translation.x() - lastRight.translation.x() : -lastLeft.translation.x() + lastRight.translation.x()) + soleBackwardsCompensationFeetShift) / soleBackwardsCompensationFeetXDifference);
    torsoAngleCompensation *= swingFactor;
    PLOT("module:WalkStepAdjustment:Data:supportSoleRotationY", Angle(supportSole.rotation.getYAngle()).toDegrees());
    swingRotation += limitAnkleOffsetFast.limit(torsoAngleCompensation - swingRotation); // apply correction
  }

  PLOT("module:WalkStepAdjustment:Data:supportSoleRotationCompensationY", swingRotation.toDegrees());
}
