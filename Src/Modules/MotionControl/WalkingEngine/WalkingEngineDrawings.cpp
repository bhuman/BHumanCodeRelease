/**
* @file WalkingEngine.cpp
* Implementation of a module that creates the walking motions
* @author Colin Graf
*/

#include "WalkingEngine.h"
#include "Tools/Debugging/DebugDrawings3D.h"

using namespace std;

void WalkingEngine::declareDrawings(const WalkingEngineOutput& walkingEngineOutput) const
{
  DECLARE_DEBUG_DRAWING3D("module:WalkingEngine:W", "field", drawW(); );
  DECLARE_DEBUG_DRAWING3D("module:WalkingEngine:P", "robot", drawP(); );
  DECLARE_DEBUG_DRAWING3D("module:WalkingEngine:Q", "robot", drawQ(walkingEngineOutput); );

  /*
  DECLARE_DEBUG_DRAWING3D("module:WalkingEngine:footTrajectory", "robot", drawFootTrajectory(walkingEngineOutput); );
  DECLARE_DEBUG_DRAWING3D("module:WalkingEngine:skeleton", "robot", drawSkeleton(walkingEngineOutput); );
  DECLARE_DEBUG_DRAWING3D("module:WalkingEngine:skeleton2", "robot", drawSkeleton2(walkingEngineOutput); );
  DECLARE_DEBUG_DRAWING3D("module:WalkingEngine:joints", "robot", drawJoints(walkingEngineOutput); );
  DECLARE_DEBUG_DRAWING3D("module:WalkingEngine:joints2", "robot", drawJoints2(walkingEngineOutput); );
  DECLARE_DEBUG_DRAWING3D("module:WalkingEngine:pendulum", "robot", drawPendulum(walkingEngineOutput); );
  DECLARE_DEBUG_DRAWING3D("module:WalkingEngine:invKin1", "robot", drawInvKin1(walkingEngineOutput); );
  DECLARE_DEBUG_DRAWING3D("module:WalkingEngine:invKin2", "robot", drawInvKin2(walkingEngineOutput); );
  DECLARE_DEBUG_DRAWING3D("module:WalkingEngine:invKin3", "robot", drawInvKin3(walkingEngineOutput); );

  WalkRequest::KickType previewKickType = WalkRequest::none;
  MODIFY_ENUM("module:WalkingEngine:previewKickType", previewKickType, WalkRequest);
  DECLARE_DEBUG_DRAWING3D("module:WalkingEngine:preview", "robot", drawPreview(previewKickType, walkingEngineOutput); );

  DECLARE_DEBUG_DRAWING3D("module:WalkingEngine:com", "robot", drawShowcaseCenterOfMass(walkingEngineOutput); );
  DECLARE_DEBUG_DRAWING3D("module:WalkingEngine:contactAreaLeft", "LFoot", drawShowcaseContactAreaLeft(); );
  DECLARE_DEBUG_DRAWING3D("module:WalkingEngine:contactAreaRight", "RFoot", drawShowcaseContactAreaRight(); );
  DECLARE_DEBUG_DRAWING3D("module:WalkingEngine:supportAreaLeft", "LFoot", drawShowcaseSupportAreaLeft(); );
  DECLARE_DEBUG_DRAWING3D("module:WalkingEngine:supportAreaBoth", "LFoot", drawShowcaseSupportAreaBoth(walkingEngineOutput); );
  DECLARE_DEBUG_DRAWING3D("module:WalkingEngine:ipTrajectory", "robot", drawShowcaseIpTrajectory(walkingEngineOutput); );
  DECLARE_DEBUG_DRAWING3D("module:WalkingEngine:ip", "robot", drawShowcaseIp(walkingEngineOutput); );
  DECLARE_DEBUG_DRAWING3D("module:WalkingEngine:ipSphere", "robot", drawShowcaseIpSphere(walkingEngineOutput); );
  DECLARE_DEBUG_DRAWING3D("module:WalkingEngine:3dlipmPlane", "robot", drawShowcase3dlipmPlane(walkingEngineOutput); );
  DECLARE_DEBUG_DRAWING3D("module:WalkingEngine:simpleIp", "field", drawShowcaseSimpleIp(walkingEngineOutput); );
  DECLARE_DEBUG_DRAWING3D("module:WalkingEngine:zmp", "robot", drawShowcaseZmp(walkingEngineOutput); );
  DECLARE_DEBUG_DRAWING3D("module:WalkingEngine:floorGrid", "robot", drawShowcaseFloorGrid(theTorsoMatrix); );
  */
 }

void WalkingEngine::drawW() const
{
  const float scale = 8.f;
  Vector3<> p0 = Vector3<>(0.f, 0.f, 0.f);
  Vector3<> px2 = Vector3<>(200.f * scale * 0.75f, 0.f, 0.f);
  Vector3<> py2 = Vector3<>(0.f, 200.f * scale * 0.75f, 0.f);
  Vector3<> pz2 = Vector3<>(0.f, 0.f, 200.f * scale * 0.75f);
  CYLINDERARROW3D("module:WalkingEngine:W", p0, Vector3<>(px2.x, px2.y, px2.z), 2 * scale, 20 * scale, 10 * scale, ColorRGBA(255, 0, 0));
  CYLINDERARROW3D("module:WalkingEngine:W", p0, Vector3<>(py2.x, py2.y, py2.z), 2 * scale, 20 * scale, 10 * scale, ColorRGBA(0, 255, 0));
  CYLINDERARROW3D("module:WalkingEngine:W", p0, Vector3<>(pz2.x, pz2.y, pz2.z), 2 * scale, 20 * scale, 10 * scale, ColorRGBA(0, 0, 255));
}

void WalkingEngine::drawP() const
{
  Vector3<> p0 = Vector3<>(0.f, 0.f, 85.f);
  Vector3<> px2 = Vector3<>(200.f * 0.75f, 0.f, 0.f);
  Vector3<> py2 = Vector3<>(0.f, 200.f * 0.75f, 0.f);
  Vector3<> pz2 = Vector3<>(0.f, 0.f, 200.f * 0.75f);
  CYLINDERARROW3D("module:WalkingEngine:P", p0, Vector3<>(px2.x, px2.y, px2.z + 85.f), 2, 20, 10, ColorRGBA(255, 0, 0));
  CYLINDERARROW3D("module:WalkingEngine:P", p0, Vector3<>(py2.x, py2.y, py2.z + 85.f), 2, 20, 10, ColorRGBA(0, 255, 0));
  CYLINDERARROW3D("module:WalkingEngine:P", p0, Vector3<>(pz2.x, pz2.y, pz2.z + 85.f), 2, 20, 10, ColorRGBA(0, 0, 255));
}

void WalkingEngine::drawQ(const WalkingEngineOutput& walkingEngineOutput) const
{
  RobotModel robotModel(walkingEngineOutput, theRobotDimensions, theMassCalibration);
  Pose3D originToQ = robotModel.limbs[predictedPendulumPlayer.phase.type == leftSupportPhase ? MassCalibration::footLeft : MassCalibration::footRight];
  originToQ.translate(0, 0, -theRobotDimensions.heightLeg5Joint);
  originToQ.conc(predictedPendulumPlayer.phase.type == leftSupportPhase ? targetPosture.leftOriginToFoot.invert() : targetPosture.rightOriginToFoot.invert());

  Vector3<> px2 = originToQ * Vector3<>(200.f * 0.75f, 0.f, 0.f);
  Vector3<> p0 = originToQ * Vector3<>(0.f, 0.f, 0.f);
  Vector3<> py2 = originToQ * Vector3<>(0.f, 200.f * 0.75f, 0.f);
  Vector3<> pz2 = originToQ * Vector3<>(0.f, 0.f, 200.f * 0.75f);
  CYLINDERARROW3D("module:WalkingEngine:Q", p0, Vector3<>(px2.x, px2.y, px2.z), 2, 20, 10, ColorRGBA(255, 0, 0));
  CYLINDERARROW3D("module:WalkingEngine:Q", p0, Vector3<>(py2.x, py2.y, py2.z), 2, 20, 10, ColorRGBA(0, 255, 0));
  CYLINDERARROW3D("module:WalkingEngine:Q", p0, Vector3<>(pz2.x, pz2.y, pz2.z), 2, 20, 10, ColorRGBA(0, 0, 255));
}


/*

void WalkingEngine::drawFootTrajectory(const WalkingEngineOutput& walkingEngineOutput) const
{
  RobotModel robotModel(walkingEngineOutput, theRobotDimensions, theMassCalibration);
  Pose3D originToQ = robotModel.limbs[predictedPendulumPlayer.phase.type == leftSupportPhase ? MassCalibration::footLeft : MassCalibration::footRight];
  originToQ.translate(0, 0, -theRobotDimensions.heightLeg5Joint);
  originToQ.conc(predictedPendulumPlayer.phase.type == leftSupportPhase ? targetStance.leftOriginToFoot.invert() : targetStance.rightOriginToFoot.invert());
  Pose3D originToNextQ = Pose3D(originToQ).conc(predictedPendulumPlayer.nextPhase.s.translation); //.rotateZ(predictedPendulumPlayer.nextPhase.s.rotation);

  // paint air foot trajectory
  Vector3<> lastP;
  LegStance tempStance;
  PendulumPlayer pendulumPlayer = this->predictedPendulumPlayer;
  for(float t = 0.f; t <= predictedPendulumPlayer.phase.duration; t += 0.005f)
  {
    pendulumPlayer.seekTo(t);
    pendulumPlayer.getStance(tempStance, 0, 0, 0);
    Pose3D& nextOriginToFoot = (predictedPendulumPlayer.phase.type == leftSupportPhase) ? tempStance.rightOriginToFoot : tempStance.leftOriginToFoot;
    Vector3<> point = originToNextQ * nextOriginToFoot.translation;
    if(t > 0.f)
      LINE3D("module:WalkingEngine:footTrajectory", lastP.x, lastP.y, lastP.z, point.x, point.y, point.z, 3, ColorRGBA(127, 0, 0));
    lastP = point;
  }
}

void WalkingEngine::drawPreview(WalkRequest::KickType previewKickType, const WalkingEngineOutput& walkingEngineOutput) const
{
  RobotModel robotModel(walkingEngineOutput, theRobotDimensions, theMassCalibration);
  Pose3D originToQ = robotModel.limbs[predictedPendulumPlayer.phase.type == leftSupportPhase ? MassCalibration::footLeft : MassCalibration::footRight];
  originToQ.translate(0, 0, -theRobotDimensions.heightLeg5Joint);
  originToQ.conc(predictedPendulumPlayer.phase.type == leftSupportPhase ? targetStance.leftOriginToFoot.invert() : targetStance.rightOriginToFoot.invert());
  Pose3D originToNextQ = Pose3D(originToQ).conc(predictedPendulumPlayer.nextPhase.s.translation); //.rotateZ(predictedPendulumPlayer.nextPhase.s.rotation);

  // paint kick trajectory
  if(previewKickType != WalkRequest::none)
  {
    WalkingEngineKickPlayer kickPlayer;
    bool mirrored = kicks.isKickMirrored(previewKickType);
    kickPlayer.start(*kicks.getKick(previewKickType), mirrored);
    StepSize s, ns;
    kicks.getKickPreStepSize(previewKickType, s.rotation, s.translation);
    kicks.getKickStepSize(previewKickType, ns.rotation, ns.translation);
    float duration = kicks.getKickStepDuration(previewKickType);
    if(duration == 0.f)
      duration = 1.f;
    Pose3D stepOffset, leftOriginToFoot, rightOriginToFoot;
    const float sign = kicks.isKickMirrored(previewKickType) ? 1.f : -1.f;
    Vector3<> lastP;
    for(float t = 0.f; t <= duration; t += 0.005f)
    {
      ASSERT(duration != 0.f);
      kickPlayer.seekTo(t * kickPlayer.getLength() / duration);

      const float ratio = t / duration;
      const float swingMoveFadeIn = ratio < walkMovePhase.start ? 0.f : ratio > walkMovePhase.start + walkMovePhase.duration ? 1.f : blend((ratio - walkMovePhase.start) / walkMovePhase.duration);
      const float swingLift = ratio < walkLiftPhase.start || ratio > walkLiftPhase.start + walkLiftPhase.duration ? 0.f : blend((ratio - walkLiftPhase.start) / walkLiftPhase.duration * 2.f);
      leftOriginToFoot = targetStance.leftOriginToFoot;
      rightOriginToFoot = targetStance.rightOriginToFoot;
      Pose3D &nextOriginToFoot = mirrored ? rightOriginToFoot : leftOriginToFoot;
      nextOriginToFoot.translation.y -= (ns.translation.y + s.translation.y);
      nextOriginToFoot.translation += (ns.translation + s.translation) * swingMoveFadeIn;
      nextOriginToFoot.translation += Vector3<>(walkLiftOffset.x, walkLiftOffset.y * sign, walkLiftOffset.z) * swingLift;
      nextOriginToFoot.rotateZ((ns.rotation + s.rotation) * swingMoveFadeIn);
      kickPlayer.applyFoot(leftOriginToFoot, rightOriginToFoot);
      Vector3<> point = originToNextQ * nextOriginToFoot.translation;
      if(t > 0.f)
        LINE3D("module:WalkingEngine:preview", lastP.x, lastP.y, lastP.z, point.x, point.y, point.z, 3, ColorRGBA(127, 0, 0));
      lastP = point;
    }
  }
}

void WalkingEngine::drawSkeleton(const WalkingEngineOutput& walkingEngineOutput) const
{
  const float boneLineWidth = 5.f;
  ColorRGBA boneColor(0xaa, 0, 0);
  const float jointRadius = 5.f;

  RobotModel robotModel(walkingEngineOutput, theRobotDimensions, theMassCalibration);

  for(int i = 0; i < 2; ++i)
  {
    int firstJoint = i == 0 ? MassCalibration::pelvisLeft : MassCalibration::pelvisRight;
    Pose3D last;
    for(int i = 0; i < (MassCalibration::footLeft + 1) - MassCalibration::pelvisLeft; ++i)
    {
      Pose3D& next = robotModel.limbs[firstJoint + i];
      LINE3D("module:WalkingEngine:skeleton", last.translation.x, last.translation.y, last.translation.z, next.translation.x, next.translation.y, next.translation.z, boneLineWidth, boneColor);
      SPHERE3D("module:WalkingEngine:skeleton", next.translation.x, next.translation.y, next.translation.z, jointRadius, ColorRGBA(0, 0, 0));

      last = next;
    }
    Pose3D next = Pose3D(last).translate(0, 0, -theRobotDimensions.heightLeg5Joint);
    LINE3D("module:WalkingEngine:skeleton", last.translation.x, last.translation.y, last.translation.z, next.translation.x, next.translation.y, next.translation.z, boneLineWidth, boneColor);
  }
  Vector3<> armCenter(0.f, 0.f, theRobotDimensions.armOffset.z);
  LINE3D("module:WalkingEngine:skeleton", 0, 0, 0, armCenter.x, armCenter.y, armCenter.z, boneLineWidth, boneColor);
  for(int i = 0; i < 2; ++i)
  {
    int firstJoint = i == 0 ? MassCalibration::shoulderLeft : MassCalibration::shoulderRight;
    Pose3D last(armCenter);
    for(int i = 0; i < (MassCalibration::foreArmLeft + 1) - MassCalibration::shoulderLeft; ++i)
    {
      Pose3D& next = robotModel.limbs[firstJoint + i];
      if(i == 2)
      {
        Pose3D p = last;
        p.translate(0.f, firstJoint == MassCalibration::shoulderLeft ? theRobotDimensions.yElbowShoulder : -theRobotDimensions.yElbowShoulder, 0.f);
        LINE3D("module:WalkingEngine:skeleton", last.translation.x, last.translation.y, last.translation.z, p.translation.x, p.translation.y, p.translation.z, boneLineWidth, boneColor);
        LINE3D("module:WalkingEngine:skeleton", p.translation.x, p.translation.y, p.translation.z, next.translation.x, next.translation.y, next.translation.z, boneLineWidth, boneColor);
      }
      else
        LINE3D("module:WalkingEngine:skeleton", last.translation.x, last.translation.y, last.translation.z, next.translation.x, next.translation.y, next.translation.z, boneLineWidth, boneColor);
      SPHERE3D("module:WalkingEngine:skeleton", next.translation.x, next.translation.y, next.translation.z, jointRadius, ColorRGBA(0, 0, 0));

      last = next;
    }
    Pose3D next = Pose3D(last).translate(theRobotDimensions.lowerArmLength, 0, 0);
    LINE3D("module:WalkingEngine:skeleton", last.translation.x, last.translation.y, last.translation.z, next.translation.x, next.translation.y, next.translation.z, boneLineWidth, boneColor);
  }

  Pose3D last(armCenter);
  for(int i = 0; i < (MassCalibration::head + 1) - MassCalibration::neck; ++i)
  {
    Pose3D& next = robotModel.limbs[MassCalibration::neck + i];
    LINE3D("module:WalkingEngine:skeleton", last.translation.x, last.translation.y, last.translation.z, next.translation.x, next.translation.y, next.translation.z, boneLineWidth, boneColor);
    SPHERE3D("module:WalkingEngine:skeleton", next.translation.x, next.translation.y, next.translation.z, jointRadius, ColorRGBA(0, 0, 0));

    last = next;
  }
  Pose3D next = Pose3D(last).translate(0, 0, 100.f);
  LINE3D("module:WalkingEngine:skeleton", last.translation.x, last.translation.y, last.translation.z, next.translation.x, next.translation.y, next.translation.z, boneLineWidth, boneColor);
}

void WalkingEngine::drawSkeleton2(const WalkingEngineOutput& walkingEngineOutput) const
{
  const float boneLineWidth = 5.f;
  ColorRGBA boneColor(0xaa, 0, 0);
  const float jointRadius = 5.f;

  RobotModel robotModel(walkingEngineOutput, theRobotDimensions, theMassCalibration);

  for(int i = 0; i < 2; ++i)
  {
    int firstJoint = i == 0 ? MassCalibration::pelvisLeft : MassCalibration::pelvisRight;
    Pose3D last;
    for(int i = 0; i < (MassCalibration::footLeft + 1) - MassCalibration::pelvisLeft; ++i)
    {
      Pose3D& next = robotModel.limbs[firstJoint + i];
      LINE3D("module:WalkingEngine:skeleton2", last.translation.x, last.translation.y, last.translation.z, next.translation.x, next.translation.y, next.translation.z, boneLineWidth, boneColor);
      SPHERE3D("module:WalkingEngine:skeleton2", next.translation.x, next.translation.y, next.translation.z, jointRadius, ColorRGBA(0, 0, 0));

      last = next;
    }
    Pose3D next = Pose3D(last).translate(0, 0, -theRobotDimensions.heightLeg5Joint);
    LINE3D("module:WalkingEngine:skeleton2", last.translation.x, last.translation.y, last.translation.z, next.translation.x, next.translation.y, next.translation.z, boneLineWidth, boneColor);
  }

  for(int i = 0; i < 2; ++i)
  {
    int firstJoint = i == 0 ? MassCalibration::shoulderLeft : MassCalibration::shoulderRight;
    Pose3D last;
    for(int i = 0; i < (MassCalibration::foreArmLeft + 1) - MassCalibration::shoulderLeft; ++i)
    {
      Pose3D& next = robotModel.limbs[firstJoint + i];
      LINE3D("module:WalkingEngine:skeleton2", last.translation.x, last.translation.y, last.translation.z, next.translation.x, next.translation.y, next.translation.z, boneLineWidth, boneColor);
      SPHERE3D("module:WalkingEngine:skeleton2", next.translation.x, next.translation.y, next.translation.z, jointRadius, ColorRGBA(0, 0, 0));

      last = next;
    }
    Pose3D next = Pose3D(last).translate(theRobotDimensions.lowerArmLength, 0, 0);
    LINE3D("module:WalkingEngine:skeleton2", last.translation.x, last.translation.y, last.translation.z, next.translation.x, next.translation.y, next.translation.z, boneLineWidth, boneColor);
  }

  Pose3D last;
  for(int i = 0; i < (MassCalibration::head + 1) - MassCalibration::neck; ++i)
  {
    Pose3D& next = robotModel.limbs[MassCalibration::neck + i];
    LINE3D("module:WalkingEngine:skeleton2", last.translation.x, last.translation.y, last.translation.z, next.translation.x, next.translation.y, next.translation.z, boneLineWidth, boneColor);
    SPHERE3D("module:WalkingEngine:skeleton2", next.translation.x, next.translation.y, next.translation.z, jointRadius, ColorRGBA(0, 0, 0));

    last = next;
  }
  Pose3D next = Pose3D(last).translate(0, 0, 100.f);
  LINE3D("module:WalkingEngine:skeleton2", last.translation.x, last.translation.y, last.translation.z, next.translation.x, next.translation.y, next.translation.z, boneLineWidth, boneColor);
}

void WalkingEngine::drawJoints(const WalkingEngineOutput& walkingEngineOutput) const
{
  const float axisLineWidth = 3.f;

  RobotModel robotModel(walkingEngineOutput, theRobotDimensions, theMassCalibration);

  for(int i = 0; i < 2; ++i)
  {
    int firstJoint = i == 0 ? MassCalibration::pelvisLeft : MassCalibration::pelvisRight;
    Pose3D last;
    for(int i = 0; i < (MassCalibration::footLeft + 1) - MassCalibration::pelvisLeft; ++i)
    {
      Pose3D& next = robotModel.limbs[firstJoint + i];
      SPHERE3D("module:WalkingEngine:joints", next.translation.x, next.translation.y, next.translation.z, 3, ColorRGBA(0, 0, 0));

      Vector3<> axis  = i == 0 ? Vector3<>(0.f, firstJoint == MassCalibration::pelvisLeft ? -50.f : 50.f, 50.f).normalize(50.f) : i == 1 || i == 5 ? Vector3<>(50.f, 0.f, 0.f) : Vector3<>(0.f, 50.f, 0.f);
      ColorRGBA color = i == 0 ? ColorRGBA(0, 0, 255)      : i == 1 || i == 5 ? ColorRGBA(255, 0, 0)      : ColorRGBA(0, 255, 0);
      Vector3<> p = next * axis;
      LINE3D("module:WalkingEngine:joints", next.translation.x, next.translation.y, next.translation.z, p.x, p.y, p.z, axisLineWidth, color);

      last = next;
    }
  }
  Vector3<> armCenter(0.f, 0.f, theRobotDimensions.armOffset.z);
  for(int i = 0; i < 2; ++i)
  {
    int firstJoint = i == 0 ? MassCalibration::shoulderLeft : MassCalibration::shoulderRight;
    Pose3D last(armCenter);
    for(int i = 0; i < (MassCalibration::foreArmLeft + 1) - MassCalibration::shoulderLeft; ++i)
    {
      Pose3D& next = robotModel.limbs[firstJoint + i];
      SPHERE3D("module:WalkingEngine:joints", next.translation.x, next.translation.y, next.translation.z, 3, ColorRGBA(0, 0, 0));

      Vector3<> axis  = i == 1 || i == 3 ? Vector3<>(0.f, 0.f, 50.f) : i == 2 ? Vector3<>(50.f, 0.f, 0.f) : Vector3<>(0.f, 50.f, 0.f);
      ColorRGBA color = i == 1 || i == 3 ? ColorRGBA(0, 0, 255)      : i == 2 ? ColorRGBA(255, 0, 0)      : ColorRGBA(0, 255, 0);
      Vector3<> p = next * axis;
      LINE3D("module:WalkingEngine:joints", next.translation.x, next.translation.y, next.translation.z, p.x, p.y, p.z, axisLineWidth, color);

      last = next;
    }
  }

  Pose3D last(armCenter);
  for(int i = 0; i < (MassCalibration::head + 1) - MassCalibration::neck; ++i)
  {
    Pose3D& next = robotModel.limbs[MassCalibration::neck + i];
    SPHERE3D("module:WalkingEngine:joints", next.translation.x, next.translation.y, next.translation.z, 3, ColorRGBA(0, 0, 0));

    Vector3<> axis  = i == 0 ? Vector3<>(0.f, 0.f, 50.f) : Vector3<>(0.f, 50.f, 0.f);
    ColorRGBA color = i == 0 ? ColorRGBA(0, 0, 255)      : ColorRGBA(0, 255, 0);
    Vector3<> p = next * axis;
    LINE3D("module:WalkingEngine:joints", next.translation.x, next.translation.y, next.translation.z, p.x, p.y, p.z, axisLineWidth, color);

    last = next;
  }
}

void WalkingEngine::drawJoints2(const WalkingEngineOutput& walkingEngineOutput) const
{
  const float boneLineWidth = 5.f;
  ColorRGBA boneColor(0, 0, 0);
  const float cylinderRadius = 4.f;
  const float axisLength = 20.f;

  SPHERE3D("module:WalkingEngine:joints2", 0.f, 0.f, 85.f, 3, ColorRGBA(0, 0, 0));

  RobotModel robotModel(walkingEngineOutput, theRobotDimensions, theMassCalibration);

  for(int i = 0; i < 2; ++i)
  {
    int firstJoint = i == 0 ? MassCalibration::pelvisLeft : MassCalibration::pelvisRight;
    Pose3D last;
    for(int i = 0; i < (MassCalibration::footLeft + 1) - MassCalibration::pelvisLeft; ++i)
    {
      Pose3D& next = robotModel.limbs[firstJoint + i];
      LINE3D("module:WalkingEngine:joints2", last.translation.x, last.translation.y, last.translation.z, next.translation.x, next.translation.y, next.translation.z, boneLineWidth, boneColor);

      Vector3<> axis  = i == 0 ? Vector3<>(0.f, firstJoint == MassCalibration::pelvisLeft ? -50.f : 50.f, 50.f).normalize(axisLength) : i == 1 || i == 5 ? Vector3<>(axisLength, 0.f, 0.f) : Vector3<>(0.f, axisLength, 0.f);
      ColorRGBA color = i == 0 ? ColorRGBA(0, 0, 255)      : i == 1 || i == 5 ? ColorRGBA(255, 0, 0)      : ColorRGBA(0, 255, 0);
      Vector3<> p = next * axis;
      Vector3<> p2 = next * (-axis);
      CYLINDERLINE3D("module:WalkingEngine:joints2", p2, p, cylinderRadius, color);

      last = next;
    }
    Pose3D next = Pose3D(last).translate(0, 0, -theRobotDimensions.heightLeg5Joint);
    LINE3D("module:WalkingEngine:joints2", last.translation.x, last.translation.y, last.translation.z, next.translation.x, next.translation.y, next.translation.z, boneLineWidth, boneColor);
  }
  Vector3<> armCenter(0.f, 0.f, theRobotDimensions.armOffset.z);
  LINE3D("module:WalkingEngine:joints2", 0, 0, 0, armCenter.x, armCenter.y, armCenter.z, boneLineWidth, boneColor);
  for(int i = 0; i < 2; ++i)
  {
    int firstJoint = i == 0 ? MassCalibration::shoulderLeft : MassCalibration::shoulderRight;
    Pose3D last(armCenter);
    for(int i = 0; i < (MassCalibration::foreArmLeft + 1) - MassCalibration::shoulderLeft; ++i)
    {
      Pose3D& next = robotModel.limbs[firstJoint + i];
      if(i == 2)
      {
        Pose3D p = last;
        p.translate(0.f, firstJoint == MassCalibration::shoulderLeft ? theRobotDimensions.yElbowShoulder : -theRobotDimensions.yElbowShoulder, 0.f);
        LINE3D("module:WalkingEngine:joints2", last.translation.x, last.translation.y, last.translation.z, p.translation.x, p.translation.y, p.translation.z, boneLineWidth, boneColor);
        LINE3D("module:WalkingEngine:joints2", p.translation.x, p.translation.y, p.translation.z, next.translation.x, next.translation.y, next.translation.z, boneLineWidth, boneColor);
      }
      else
        LINE3D("module:WalkingEngine:joints2", last.translation.x, last.translation.y, last.translation.z, next.translation.x, next.translation.y, next.translation.z, boneLineWidth, boneColor);

      Vector3<> axis  = i == 1 || i == 3 ? Vector3<>(0.f, 0.f, axisLength) : i == 2 ? Vector3<>(axisLength, 0.f, 0.f) : Vector3<>(0.f, axisLength, 0.f);
      ColorRGBA color = i == 1 || i == 3 ? ColorRGBA(0, 0, 255)      : i == 2 ? ColorRGBA(255, 0, 0)      : ColorRGBA(0, 255, 0);
      Vector3<> p = next * axis;
      Vector3<> p2 = next * (-axis);
      CYLINDERLINE3D("module:WalkingEngine:joints2", p2, p, cylinderRadius, color);

      last = next;
    }
    //Pose3D next = Pose3D(last).translate(theRobotDimensions.lowerArmLength, 0, 0);
    //LINE3D("module:WalkingEngine:joints2", last.translation.x, last.translation.y, last.translation.z, next.translation.x, next.translation.y, next.translation.z, boneLineWidth, boneColor);
  }

  Pose3D last(armCenter);
  for(int i = 0; i < (MassCalibration::head + 1) - MassCalibration::neck; ++i)
  {
    Pose3D& next = robotModel.limbs[MassCalibration::neck + i];
    LINE3D("module:WalkingEngine:joints2", last.translation.x, last.translation.y, last.translation.z, next.translation.x, next.translation.y, next.translation.z, boneLineWidth, boneColor);

    Vector3<> axis  = i == 0 ? Vector3<>(0.f, 0.f, axisLength) : Vector3<>(0.f, axisLength, 0.f);
    ColorRGBA color = i == 0 ? ColorRGBA(0, 0, 255)      : ColorRGBA(0, 255, 0);
    Vector3<> p = next * axis;
    Vector3<> p2 = next * (-axis);
    CYLINDERLINE3D("module:WalkingEngine:joints2", p2, p, cylinderRadius, color);

    last = next;
  }
  //Pose3D next = Pose3D(last).translate(0, 0, 100.f);
  //LINE3D("module:WalkingEngine:joints2", last.translation.x, last.translation.y, last.translation.z, next.translation.x, next.translation.y, next.translation.z, boneLineWidth, boneColor);
}

void WalkingEngine::drawPendulum(const WalkingEngineOutput& walkingEngineOutput) const
{
  RobotModel robotModel(walkingEngineOutput, theRobotDimensions, theMassCalibration);
  Pose3D originToQ = robotModel.limbs[predictedPendulumPlayer.phase.type == leftSupportPhase ? MassCalibration::footLeft : MassCalibration::footRight];
  originToQ.translate(0, 0, -theRobotDimensions.heightLeg5Joint);
  originToQ.conc(predictedPendulumPlayer.phase.type == leftSupportPhase ? targetStance.leftOriginToFoot.invert() : targetStance.rightOriginToFoot.invert());

  Pose3D originToPendulum = Pose3D(originToQ).translate(Vector3<>(0.f, predictedPendulumPlayer.phase.type == leftSupportPhase ? standComPosition.y : -standComPosition.y, 0.f));

  CYLINDERLINE3D("module:WalkingEngine:pendulum", robotModel.centerOfMass, originToPendulum.translation, 1, ColorRGBA(0, 0, 0));
  SPHERE3D("module:WalkingEngine:pendulum", robotModel.centerOfMass.x, robotModel.centerOfMass.y, robotModel.centerOfMass.z, 7, ColorRGBA(0, 0, 0));
  //SPHERE3D("module:WalkingEngine:pendulum", originToPendulum.translation.x, originToPendulum.translation.y, originToPendulum.translation.z, 3, ColorRGBA(0, 0, 127));
}

#define ARC3D(id, center, start, end, width, color) do \
  { \
    Vector3<> startDir = start - center; \
    Vector3<> startDirNorm = startDir; \
    Vector3<> endDirNorm = end - center; \
    startDirNorm.normalize(); \
    endDirNorm.normalize(); \
    float angle = acos(startDirNorm * endDirNorm); \
    Vector3<> axis = startDirNorm ^ endDirNorm; \
    Vector3<> p = start; \
    for(float i = 0.f; i < angle; i += 0.1f) \
    { \
      Vector3<> pNew = RotationMatrix(axis, i) * startDir + center; \
      LINE3D(id, p.x, p.y, p.z, pNew.x, pNew.y, pNew.z, width, color); \
      p = pNew; \
    } \
    Vector3<> pNew = (end - center).normalize(startDir.abs()) + center; \
    LINE3D(id, p.x, p.y, p.z, pNew.x, pNew.y, pNew.z, width, color); \
  } while(false)

void WalkingEngine::drawInvKin1(const WalkingEngineOutput& walkingEngineOutput) const
{
  const float arcLineWidth = 2.f;
  ColorRGBA arcColor(0, 0, 0);
  const float boneLineWidth = 4.f;
  ColorRGBA boneColor(0, 0, 0);
  const float jointRadius = 2.f;

  RobotModel robotModel(walkingEngineOutput, theRobotDimensions, theMassCalibration);

  Pose3D& thighLeft = robotModel.limbs[MassCalibration::thighLeft];
  Pose3D& tibiaLeft = robotModel.limbs[MassCalibration::tibiaLeft];
  Pose3D& ankleLeft = robotModel.limbs[MassCalibration::ankleLeft];

  SPHERE3D("module:WalkingEngine:invKin1", thighLeft.translation.x, thighLeft.translation.y, thighLeft.translation.z, jointRadius, ColorRGBA(0, 0, 0xaa));
  SPHERE3D("module:WalkingEngine:invKin1", tibiaLeft.translation.x, tibiaLeft.translation.y, tibiaLeft.translation.z, jointRadius, ColorRGBA(0, 0, 0xaa));
  SPHERE3D("module:WalkingEngine:invKin1", ankleLeft.translation.x, ankleLeft.translation.y, ankleLeft.translation.z, jointRadius, ColorRGBA(0, 0, 0xaa));

  Vector3<> p1 = tibiaLeft.translation + (thighLeft.translation - tibiaLeft.translation).normalize((thighLeft.translation - tibiaLeft.translation).abs() + 45.f);
  LINE3D("module:WalkingEngine:invKin1", p1.x, p1.y, p1.z, tibiaLeft.translation.x, tibiaLeft.translation.y, tibiaLeft.translation.z, boneLineWidth, boneColor);

  Vector3<> p2 = ankleLeft.translation + (tibiaLeft.translation - ankleLeft.translation).normalize((tibiaLeft.translation - ankleLeft.translation).abs() + 45.f);
  LINE3D("module:WalkingEngine:invKin1", p2.x, p2.y, p2.z, ankleLeft.translation.x, ankleLeft.translation.y, ankleLeft.translation.z, boneLineWidth, boneColor);

  Vector3<> p3 = ankleLeft.translation + (thighLeft.translation - ankleLeft.translation).normalize((thighLeft.translation - ankleLeft.translation).abs() + 55.f);
  LINE3D("module:WalkingEngine:invKin1", p3.x, p3.y, p3.z, ankleLeft.translation.x, ankleLeft.translation.y, ankleLeft.translation.z, boneLineWidth, boneColor);

  p1 = tibiaLeft.translation + (thighLeft.translation - tibiaLeft.translation).normalize((thighLeft.translation - tibiaLeft.translation).abs() + 40.f);
  ARC3D("module:WalkingEngine:invKin1", thighLeft.translation, p1, p3, arcLineWidth, arcColor);
  p2 = ankleLeft.translation + (tibiaLeft.translation - ankleLeft.translation).normalize((tibiaLeft.translation - ankleLeft.translation).abs() + 40.f);
  ARC3D("module:WalkingEngine:invKin1", tibiaLeft.translation, p2, thighLeft.translation, arcLineWidth, arcColor);
  p3 = ankleLeft.translation + (tibiaLeft.translation - ankleLeft.translation).normalize(40.f);
  ARC3D("module:WalkingEngine:invKin1", ankleLeft.translation, p3, thighLeft.translation, arcLineWidth, arcColor);

  for(int i = 0; i < 1; ++i)
  {
    int firstJoint = i == 0 ? MassCalibration::pelvisLeft : MassCalibration::pelvisRight;
    Pose3D last;
    for(int i = 0; i < (MassCalibration::footLeft + 1) - MassCalibration::pelvisLeft; ++i)
    {
      Pose3D& next = robotModel.limbs[firstJoint + i];
      if(i > 0)
        LINE3D("module:WalkingEngine:invKin1", last.translation.x, last.translation.y, last.translation.z, next.translation.x, next.translation.y, next.translation.z, boneLineWidth, boneColor);
      SPHERE3D("module:WalkingEngine:invKin1", next.translation.x, next.translation.y, next.translation.z, jointRadius, ColorRGBA(0, 0, 0xaa));

      last = next;
    }
  }

}

void WalkingEngine::drawInvKin2(const WalkingEngineOutput& walkingEngineOutput) const
{
  const float arcLineWidth = 2.f;
  ColorRGBA arcColor(0, 0, 0);
  const float boneLineWidth = 4.f;
  ColorRGBA boneColor(0, 0, 0);
  ColorRGBA boneColor2(50, 50, 50);
  const float jointRadius = 2.f;
  const float axisLineWidth = 3.f;

  RobotModel robotModel(walkingEngineOutput, theRobotDimensions, theMassCalibration);

  Pose3D& thighLeft = robotModel.limbs[MassCalibration::thighLeft];
  Pose3D& footLeft = robotModel.limbs[MassCalibration::footLeft];
  Vector3<> up(0.f, 0.f, (thighLeft.translation - footLeft.translation).abs());

  LINE3D("module:WalkingEngine:invKin2", footLeft.translation.x, footLeft.translation.y, footLeft.translation.z, thighLeft.translation.x, thighLeft.translation.y, thighLeft.translation.z, boneLineWidth, boneColor);

  Vector3<> p1 = footLeft * up;
  LINE3D("module:WalkingEngine:invKin2", footLeft.translation.x, footLeft.translation.y, footLeft.translation.z, p1.x, p1.y, p1.z, boneLineWidth, boneColor2);

  Vector3<> fromFootToHip = footLeft.invert().conc(thighLeft).translation;
  float angleX = atan2(fromFootToHip.y, fromFootToHip.z);
  //float angleY = -atan2(fromFootToHip.x, sqrt(sqr(fromFootToHip.y) + sqr(fromFootToHip.z)));

  Vector3<> p2 = footLeft * (RotationMatrix::fromRotationX(-angleX) * up);
  LINE3D("module:WalkingEngine:invKin2", footLeft.translation.x, footLeft.translation.y, footLeft.translation.z, p2.x, p2.y, p2.z, boneLineWidth, boneColor2);
  Vector3<> p3 = thighLeft.translation;

  Vector3<> p1s = (p1 - footLeft.translation).normalize(40.f) + footLeft.translation;
  ARC3D("module:WalkingEngine:invKin2", footLeft.translation, p1s, p2, arcLineWidth, arcColor);
  Vector3<> p1q = ((p2 - footLeft.translation) ^ (p1 - footLeft.translation)).normalize(50.f) + footLeft.translation;
  LINE3D("module:WalkingEngine:invKin2", p1q.x, p1q.y, p1q.z, footLeft.translation.x, footLeft.translation.y, footLeft.translation.z, axisLineWidth, ColorRGBA(255, 0, 0));

  Vector3<> p2s = (p2 - footLeft.translation).normalize(40.f) + footLeft.translation;
  ARC3D("module:WalkingEngine:invKin2", footLeft.translation, p2s, thighLeft.translation, arcLineWidth, arcColor);
  Vector3<> p2q = ((p3 - footLeft.translation) ^ (p2 - footLeft.translation)).normalize(-50.f) + footLeft.translation;
  LINE3D("module:WalkingEngine:invKin2", p2q.x, p2q.y, p2q.z, footLeft.translation.x, footLeft.translation.y, footLeft.translation.z, axisLineWidth, ColorRGBA(0, 255, 0));

  SPHERE3D("module:WalkingEngine:invKin2", footLeft.translation.x, footLeft.translation.y, footLeft.translation.z, jointRadius, ColorRGBA(0, 0, 0xaa));
  SPHERE3D("module:WalkingEngine:invKin2", thighLeft.translation.x, thighLeft.translation.y, thighLeft.translation.z, jointRadius, ColorRGBA(0, 0, 0xaa));

  {
    ColorRGBA boneColor(0, 0, 0, 107);
    ColorRGBA jointColor(0, 0, 0xaa, 107);

    Pose3D& thighLeft = robotModel.limbs[MassCalibration::thighLeft];
    Pose3D& tibiaLeft = robotModel.limbs[MassCalibration::tibiaLeft];
    Pose3D& ankleLeft = robotModel.limbs[MassCalibration::ankleLeft];

    //SPHERE3D("module:WalkingEngine:invKin2", thighLeft.translation.x, thighLeft.translation.y, thighLeft.translation.z, jointRadius, jointColor);
    SPHERE3D("module:WalkingEngine:invKin2", tibiaLeft.translation.x, tibiaLeft.translation.y, tibiaLeft.translation.z, jointRadius, jointColor);
    //SPHERE3D("module:WalkingEngine:invKin2", ankleLeft.translation.x, ankleLeft.translation.y, ankleLeft.translation.z, jointRadius, jointColor);

    Vector3<> p1 = thighLeft.translation;
    LINE3D("module:WalkingEngine:invKin2", p1.x, p1.y, p1.z, tibiaLeft.translation.x, tibiaLeft.translation.y, tibiaLeft.translation.z, boneLineWidth, boneColor);

    Vector3<> p2 = tibiaLeft.translation;
    LINE3D("module:WalkingEngine:invKin2", p2.x, p2.y, p2.z, ankleLeft.translation.x, ankleLeft.translation.y, ankleLeft.translation.z, boneLineWidth, boneColor);

    Vector3<> p3 = thighLeft.translation;
    LINE3D("module:WalkingEngine:invKin2", p3.x, p3.y, p3.z, ankleLeft.translation.x, ankleLeft.translation.y, ankleLeft.translation.z, boneLineWidth, boneColor);
  }
}

void WalkingEngine::drawInvKin3(const WalkingEngineOutput& walkingEngineOutput) const
{
  const float axisLineWidth = 3.f;

  RobotModel robotModel(walkingEngineOutput, theRobotDimensions, theMassCalibration);
  Vector3<> hip = robotModel.limbs[MassCalibration::thighLeft].translation;

  RotationMatrix destRotX = RotationMatrix::fromRotationX(pi_4);
  Vector3<> p = hip + destRotX * Vector3<>(50.f, 0.f, 0.f);
  LINE3D("module:WalkingEngine:invKin3", hip.x, hip.y, hip.z, p.x, p.y, p.z, axisLineWidth, ColorRGBA(255, 0, 0));
  p = hip + destRotX * Vector3<>(0.f, 50.f, 0.f);
  LINE3D("module:WalkingEngine:invKin3", hip.x, hip.y, hip.z, p.x, p.y, p.z, axisLineWidth, ColorRGBA(0, 255, 0));
  p = hip + destRotX * Vector3<>(0.f, 0.f, 50.f);
  LINE3D("module:WalkingEngine:invKin3", hip.x, hip.y, hip.z, p.x, p.y, p.z, axisLineWidth, ColorRGBA(0, 0, 255));

  Pose3D& footLeft = robotModel.limbs[MassCalibration::footLeft];
  Pose3D& thighLeft = robotModel.limbs[MassCalibration::thighLeft];
  Vector3<> fromFootToHip = footLeft.invert().conc(thighLeft).translation;
  float angleX = atan2(fromFootToHip.y, fromFootToHip.z);
  float angleY = -atan2(fromFootToHip.x, sqrt(sqr(fromFootToHip.y) + sqr(fromFootToHip.z)));

  RotationMatrix curRotX = Pose3D(robotModel.limbs[MassCalibration::footLeft]).rotateX(-angleX).rotateY(-angleY).rotation;

  p = hip + curRotX * Vector3<>(50.f, 0.f, 0.f);
  LINE3D("module:WalkingEngine:invKin3", hip.x, hip.y, hip.z, p.x, p.y, p.z, axisLineWidth, ColorRGBA(255, 0, 0, 120));
  p = hip + curRotX * Vector3<>(0.f, 50.f, 0.f);
  LINE3D("module:WalkingEngine:invKin3", hip.x, hip.y, hip.z, p.x, p.y, p.z, axisLineWidth, ColorRGBA(0, 255, 0, 120));
  p = hip + curRotX * Vector3<>(0.f, 0.f, 50.f);
  LINE3D("module:WalkingEngine:invKin3", hip.x, hip.y, hip.z, p.x, p.y, p.z, axisLineWidth, ColorRGBA(0, 0, 255, 120));
  //Vector3<> px = hip + (hip - robotModel.limbs[MassCalibration::tibiaLeft].translation).normalize(50.f);
  //LINE3D("module:WalkingEngine:invKin3", hip.x, hip.y, hip.z, px.x, px.y, px.z, axisLineWidth, ColorRGBA(0, 0, 255, 120));

  {
    Pose3D& thighLeft = robotModel.limbs[MassCalibration::thighLeft];
    Pose3D& footLeft = robotModel.limbs[MassCalibration::footLeft];
    Vector3<> up(0.f, 0.f, (thighLeft.translation - footLeft.translation).abs());

    //LINE3D("module:WalkingEngine:invKin3", footLeft.translation.x, footLeft.translation.y, footLeft.translation.z, thighLeft.translation.x, thighLeft.translation.y, thighLeft.translation.z, boneLineWidth, boneColor);

    Vector3<> p1 = footLeft * up;
    //LINE3D("module:WalkingEngine:invKin3", footLeft.translation.x, footLeft.translation.y, footLeft.translation.z, p1.x, p1.y, p1.z, boneLineWidth, boneColor2);

    Vector3<> fromFootToHip = footLeft.invert().conc(thighLeft).translation;
    float angleX = atan2(fromFootToHip.y, fromFootToHip.z);
    //float angleY = -atan2(fromFootToHip.x, sqrt(sqr(fromFootToHip.y) + sqr(fromFootToHip.z)));

    Vector3<> p2 = footLeft * (RotationMatrix::fromRotationX(-angleX) * up);
    //LINE3D("module:WalkingEngine:invKin3", footLeft.translation.x, footLeft.translation.y, footLeft.translation.z, p2.x, p2.y, p2.z, boneLineWidth, boneColor2);
    Vector3<> p3 = thighLeft.translation;

    Vector3<> p1s = (p1 - footLeft.translation).normalize(40.f) + footLeft.translation;
    //ARC3D("module:WalkingEngine:invKin3", footLeft.translation, p1s, p2, arcLineWidth, arcColor);
    Vector3<> p1q = ((p2 - footLeft.translation) ^ (p1 - footLeft.translation)).normalize(50.f) + footLeft.translation;
    LINE3D("module:WalkingEngine:invKin3", p1q.x, p1q.y, p1q.z, footLeft.translation.x, footLeft.translation.y, footLeft.translation.z, axisLineWidth, ColorRGBA(255, 0, 0));

    Vector3<> p2s = (p2 - footLeft.translation).normalize(40.f) + footLeft.translation;
    //ARC3D("module:WalkingEngine:invKin3", footLeft.translation, p2s, thighLeft.translation, arcLineWidth, arcColor);
    Vector3<> p2q = ((p3 - footLeft.translation) ^ (p2 - footLeft.translation)).normalize(-50.f) + footLeft.translation;
    LINE3D("module:WalkingEngine:invKin3", p2q.x, p2q.y, p2q.z, footLeft.translation.x, footLeft.translation.y, footLeft.translation.z, axisLineWidth, ColorRGBA(0, 255, 0));
  }


  {
    ColorRGBA boneColor(0, 0, 0, 107);
    ColorRGBA jointColor(0, 0, 0xaa, 107);
    const float boneLineWidth = 4.f;
    const float jointRadius = 2.f;

    Pose3D& thighLeft = robotModel.limbs[MassCalibration::thighLeft];
    Pose3D& tibiaLeft = robotModel.limbs[MassCalibration::tibiaLeft];
    Pose3D& ankleLeft = robotModel.limbs[MassCalibration::ankleLeft];

    SPHERE3D("module:WalkingEngine:invKin3", thighLeft.translation.x, thighLeft.translation.y, thighLeft.translation.z, jointRadius, jointColor);
    SPHERE3D("module:WalkingEngine:invKin3", tibiaLeft.translation.x, tibiaLeft.translation.y, tibiaLeft.translation.z, jointRadius, jointColor);
    SPHERE3D("module:WalkingEngine:invKin3", ankleLeft.translation.x, ankleLeft.translation.y, ankleLeft.translation.z, jointRadius, jointColor);

    Vector3<> p1 = thighLeft.translation;
    LINE3D("module:WalkingEngine:invKin3", p1.x, p1.y, p1.z, tibiaLeft.translation.x, tibiaLeft.translation.y, tibiaLeft.translation.z, boneLineWidth, boneColor);

    Vector3<> p2 = tibiaLeft.translation;
    LINE3D("module:WalkingEngine:invKin3", p2.x, p2.y, p2.z, ankleLeft.translation.x, ankleLeft.translation.y, ankleLeft.translation.z, boneLineWidth, boneColor);

    Vector3<> p3 = thighLeft.translation;
    LINE3D("module:WalkingEngine:invKin3", p3.x, p3.y, p3.z, ankleLeft.translation.x, ankleLeft.translation.y, ankleLeft.translation.z, boneLineWidth, boneColor);
  }

}

#define CIRCLE3D(id, pos, radius, slices, color) do \
  { \
    const Vector3<> pd((radius), 0.f, 0.f); \
    Vector3<> p1 = (pos) * pd; \
    for(unsigned int i = 0; i < (slices); ++i) \
    { \
      Vector3<> p2 = (pos) * (RotationMatrix::fromRotationZ(pi2 * (i + 1) / (slices)) * pd); \
      QUAD3D(id, p1, p2, (pos).translation, (pos).translation, color); \
      p1 = p2; \
    } \
  } while(false)

void WalkingEngine::drawShowcaseCenterOfMass(const WalkingEngineOutput& walkingEngineOutput) const
{
  RobotModel robotModel(walkingEngineOutput, theRobotDimensions, theMassCalibration);
  const Pose3D& originToAnkle = robotModel.limbs[predictedPendulumPlayer.phase.type == leftSupportPhase ? MassCalibration::footLeft : MassCalibration::footRight];

  SPHERE3D("module:WalkingEngine:com", robotModel.centerOfMass.x, robotModel.centerOfMass.y, robotModel.centerOfMass.z, 7, ColorRGBA(255, 0, 0));

  Pose3D originToCom = Pose3D(-robotModel.centerOfMass);
  Pose3D originToGround = Pose3D(originToAnkle).translate(0, 0, -theRobotDimensions.heightLeg5Joint);
  Pose3D groundToCom = originToGround.invert().conc(originToCom);
  Pose3D groundToProjectedCom = Pose3D(groundToCom.rotation, Vector3<>(groundToCom.translation.x, groundToCom.translation.y, 0.f));
  Pose3D originToProjectedCom = Pose3D(originToGround).conc(groundToProjectedCom);

  CIRCLE3D("module:WalkingEngine:com", originToProjectedCom, 7.f, 16, ColorRGBA(255, 0, 0));
  CYLINDERLINE3D("module:WalkingEngine:com", robotModel.centerOfMass, originToProjectedCom.translation, 7.f, ColorRGBA(255, 0, 0, 40));
}

const Vector3<> WalkingEngine::drawFootPoints[] =
{
  Vector3<>(0.00897f, 0.03318f, -0.04517f),
  Vector3<>(0.0577f, 0.03877f, -0.04526f),
  Vector3<>(0.07395f, 0.03729f, -0.04528f),
  Vector3<>(0.08765f, 0.03148f, -0.04531f),
  Vector3<>(0.09687f, 0.0188f, -0.04532f),
  Vector3<>(0.09942f, 0.01015f, -0.04533f),

  Vector3<>(0.09899f, -0.00869f, -0.04533f),
  Vector3<>(0.094f, -0.02418f, -0.04532f),
  Vector3<>(0.08454f, -0.0361f, -0.04531f),
  Vector3<>(0.06568f, -0.04615f, -0.04527f),
  Vector3<>(0.04991f, -0.04818f, -0.04525f),
  Vector3<>(0.00956f, -0.03881f, -0.04518f),

  Vector3<>(-0.00842f, -0.03954f, -0.04515f),
  Vector3<>(-0.02199f, -0.04758f, -0.04513f),
  Vector3<>(-0.03125f, -0.05002f, -0.04511f),
  Vector3<>(-0.04905f, -0.0376f, -0.04508f),

  Vector3<>(-0.05072f, 0.02138f, -0.04507f),
  Vector3<>(-0.04262f, 0.0306f, -0.04509f),
  Vector3<>(-0.03297f, 0.03435f, -0.0451f),
  Vector3<>(-0.00901f, 0.03272f, -0.04514f),
};
const unsigned int WalkingEngine::drawNumOfFootPoints = sizeof(drawFootPoints) / sizeof(*drawFootPoints);

void WalkingEngine::drawShowcaseContactAreaLeft() const
{
  for(unsigned int i = 0; i < drawNumOfFootPoints; ++i)
  {
    Vector3<> p1 = drawFootPoints[i] * 1000.f;
    Vector3<> p2 = drawFootPoints[(i + 1) % drawNumOfFootPoints] * 1000.f;
    p1.y = -p1.y;
    p2.y = -p2.y;
    Vector2<> p1b(p1.x, p1.y);
    Vector2<> p2b(p2.x, p2.y);
    p1b.normalize(p1b.abs() - 1.f);
    p2b.normalize(p2b.abs() - 1.f);
    LINE3D("module:WalkingEngine:contactAreaLeft", p1b.x, p1b.y, p1.z - 0.1f, p2b.x, p2b.y, p2.z - 0.1f, 2, ColorRGBA(255, 0, 0));
  }
}

void WalkingEngine::drawShowcaseContactAreaRight() const
{
  for(unsigned int i = 0; i < drawNumOfFootPoints; ++i)
  {
    Vector3<> p1 = drawFootPoints[i] * 1000.f;
    Vector3<> p2 = drawFootPoints[(i + 1) % drawNumOfFootPoints] * 1000.f;
    Vector2<> p1b(p1.x, p1.y);
    Vector2<> p2b(p2.x, p2.y);
    p1b.normalize(p1b.abs() - 1.f);
    p2b.normalize(p2b.abs() - 1.f);
    LINE3D("module:WalkingEngine:contactAreaRight", p1b.x, p1b.y, p1.z - 0.1f, p2b.x, p2b.y, p2.z - 0.1f, 2, ColorRGBA(255, 0, 0));
  }
}

void WalkingEngine::drawShowcaseSupportAreaLeft() const
{
  // compute convex hull
  bool isPointActive[drawNumOfFootPoints];
  for(int i = 0; i < (int) drawNumOfFootPoints; ++i)
    isPointActive[i] = true;
  int iBase = 0, iTest = 1, iNext = 2;
  for(;;)
  {
    const Vector3<>& basePoint = drawFootPoints[iBase];
    const Vector3<>& testPoint = drawFootPoints[iTest];
    const Vector3<>& nextPoint = drawFootPoints[iNext];
    Vector2<> dir1(testPoint.x - basePoint.x, testPoint.y - basePoint.y);
    Vector2<> dir2(nextPoint.x - basePoint.x, nextPoint.y - basePoint.y);
    if(dir1.x * dir2.y - dir1.y * dir2.x < 0)
    {
      if(iTest < iBase)
        break;
      iBase = iTest;
    }
    else
      isPointActive[iTest] = false;
    iTest = iNext;
    do
      iNext = (iNext + 1) % drawNumOfFootPoints;
    while(!isPointActive[iNext]);
  }
  const Vector3<>* convexHullPoints[drawNumOfFootPoints];
  unsigned int numOfConvexHullPoints = 0;
  for(unsigned int i = 0; i < drawNumOfFootPoints; ++i)
    if(isPointActive[i])
      convexHullPoints[numOfConvexHullPoints++] = &drawFootPoints[i];

  // draw convex hull
  for(unsigned int i = 0; i < numOfConvexHullPoints; ++i)
  {
    const Vector3<> p1 = *convexHullPoints[i] * 1000.f;
    const Vector3<> p2 = *convexHullPoints[(i + 1) % numOfConvexHullPoints] * 1000.f;
    LINE3D("module:WalkingEngine:supportAreaLeft", p1.x, -p1.y, p1.z, p2.x, -p2.y, p2.z, 2, ColorRGBA(0, 0, 0));
  }
}

void WalkingEngine::drawShowcaseSupportAreaBoth(const WalkingEngineOutput& walkingEngineOutput) const
{
  RobotModel robotModel(walkingEngineOutput, theRobotDimensions, theMassCalibration);
  Pose3D leftToRight = robotModel.limbs[MassCalibration::footLeft].invert().conc(robotModel.limbs[MassCalibration::footRight]);

  const Vector3<> footPoints[] =
  {
    Vector3<>(0.09942f, 0.01015f * -1.f, -0.04533f) * 1000.f,
    Vector3<>(0.09899f, -0.00869f * -1.f, -0.04533f) * 1000.f,
    Vector3<>(0.094f, -0.02418f * -1.f, -0.04532f) * 1000.f,
    Vector3<>(0.08454f, -0.0361f * -1.f, -0.04531f) * 1000.f,
    Vector3<>(0.06568f, -0.04615f * -1.f, -0.04527f) * 1000.f,
    Vector3<>(0.04991f, -0.04818f * -1.f, -0.04525f) * 1000.f,
    Vector3<>(0.00956f, -0.03881f * -1.f, -0.04518f) * 1000.f,
    Vector3<>(-0.00842f, -0.03954f * -1.f, -0.04515f) * 1000.f,
    Vector3<>(-0.02199f, -0.04758f * -1.f, -0.04513f) * 1000.f,
    Vector3<>(-0.03125f, -0.05002f * -1.f, -0.04511f) * 1000.f,
    Vector3<>(-0.04905f, -0.0376f * -1.f, -0.04508f) * 1000.f,
    Vector3<>(-0.05072f, 0.02138f * -1.f, -0.04507f) * 1000.f,

    leftToRight * (Vector3<>(-0.05072f, 0.02138f, -0.04507f) * 1000.f),
    leftToRight * (Vector3<>(-0.04905f, -0.0376f, -0.04508f) * 1000.f),
    leftToRight * (Vector3<>(-0.03125f, -0.05002f, -0.04511f) * 1000.f),
    leftToRight * (Vector3<>(-0.02199f, -0.04758f, -0.04513f) * 1000.f),
    leftToRight * (Vector3<>(-0.00842f, -0.03954f, -0.04515f) * 1000.f),
    leftToRight * (Vector3<>(0.00956f, -0.03881f, -0.04518f) * 1000.f),
    leftToRight * (Vector3<>(0.04991f, -0.04818f, -0.04525f) * 1000.f),
    leftToRight * (Vector3<>(0.06568f, -0.04615f, -0.04527f) * 1000.f),
    leftToRight * (Vector3<>(0.08454f, -0.0361f, -0.04531f) * 1000.f),
    leftToRight * (Vector3<>(0.094f, -0.02418f, -0.04532f) * 1000.f),
    leftToRight * (Vector3<>(0.09899f, -0.00869f, -0.04533f) * 1000.f),
    leftToRight * (Vector3<>(0.09942f, 0.01015f, -0.04533f) * 1000.f),

  };
  const int numOfFootPoints = sizeof(footPoints) / sizeof(*footPoints);

  // compute convex hull
  bool isPointActive[numOfFootPoints];
  for(int i = 0; i < numOfFootPoints; ++i)
    isPointActive[i] = true;
  int iBase = 0, iTest = 1, iNext = 2;
  for(;;)
  {
    const Vector3<>& basePoint = footPoints[iBase];
    const Vector3<>& testPoint = footPoints[iTest];
    const Vector3<>& nextPoint = footPoints[iNext];
    Vector2<> dir1(testPoint.x - basePoint.x, testPoint.y - basePoint.y);
    Vector2<> dir2(nextPoint.x - basePoint.x, nextPoint.y - basePoint.y);
    if(dir1.x * dir2.y - dir1.y * dir2.x > 0.f)
    {
      if(iTest < iBase)
        break;
      iBase = iTest;
    }
    else
      isPointActive[iTest] = false;
    iTest = iNext;
    do
      iNext = (iNext + 1) % numOfFootPoints;
    while(!isPointActive[iNext]);
  }
  const Vector3<>* convexHullPoints[numOfFootPoints];
  unsigned int numOfConvexHullPoints = 0;
  for(unsigned int i = 0; i < numOfFootPoints; ++i)
    if(isPointActive[i])
      convexHullPoints[numOfConvexHullPoints++] = &footPoints[i];

  // draw convex hull
  for(unsigned int i = 0; i < numOfConvexHullPoints; ++i)
  {
    const Vector3<> p1 = *convexHullPoints[i];
    const Vector3<> p2 = *convexHullPoints[(i + 1) % numOfConvexHullPoints];
    LINE3D("module:WalkingEngine:supportAreaBoth", p1.x, p1.y, p1.z, p2.x, p2.y, p2.z, 2, ColorRGBA(0, 0, 0));
  }
}

void WalkingEngine::drawShowcaseFloorGrid(const Pose3D& torsoMatrix) const
{
  float size = 200.f;
  unsigned int steps = 10;
  ColorRGBA gridColor(200, 200, 200);
  unsigned int gridLineSize = 1;

  Pose3D torsoMatrixInv = torsoMatrix.invert();

  float stepSize = size / steps;
  for(int x = 0; x < 10; ++x)
  {
    Vector3<> p1 = torsoMatrixInv * Vector3<>(x * stepSize, -size, 0.f);
    Vector3<> p2 = torsoMatrixInv * Vector3<>(x * stepSize, size, 0.f);
    Vector3<> p3 = torsoMatrixInv * Vector3<>(x * -stepSize, -size, 0.f);
    Vector3<> p4 = torsoMatrixInv * Vector3<>(x * -stepSize, size, 0.f);
    LINE3D("module:WalkingEngine:floorGrid", p1.x, p1.y, p1.z, p2.x, p2.y, p2.z, gridLineSize, gridColor);
    LINE3D("module:WalkingEngine:floorGrid", p3.x, p3.y, p3.z, p4.x, p4.y, p4.z, gridLineSize, gridColor);
  }

  for(int y = 0; y < 10; ++y)
  {
    Vector3<> p1 = torsoMatrixInv * Vector3<>(-size, y * stepSize, 0.f);
    Vector3<> p2 = torsoMatrixInv * Vector3<>(size, y * stepSize, 0.f);
    Vector3<> p3 = torsoMatrixInv * Vector3<>(-size, y * -stepSize, 0.f);
    Vector3<> p4 = torsoMatrixInv * Vector3<>(size, y * -stepSize, 0.f);
    LINE3D("module:WalkingEngine:floorGrid", p1.x, p1.y, p1.z, p2.x, p2.y, p2.z, gridLineSize, gridColor);
    LINE3D("module:WalkingEngine:floorGrid", p3.x, p3.y, p3.z, p4.x, p4.y, p4.z, gridLineSize, gridColor);
  }
}

void WalkingEngine::drawShowcaseIpTrajectory(const WalkingEngineOutput& walkingEngineOutput) const
{
  RobotModel robotModel(walkingEngineOutput, theRobotDimensions, theMassCalibration);
  Pose3D originToQ = robotModel.limbs[predictedPendulumPlayer.phase.type == leftSupportPhase ? MassCalibration::footLeft : MassCalibration::footRight];
  originToQ.translate(0, 0, -theRobotDimensions.heightLeg5Joint);
  originToQ.conc(predictedPendulumPlayer.phase.type == leftSupportPhase ? targetStance.leftOriginToFoot.invert() : targetStance.rightOriginToFoot.invert());

  Pose3D originToPendulum = Pose3D(originToQ).translate(predictedPendulumPlayer.phase.r.x, predictedPendulumPlayer.phase.r.y, 0.f);

  CYLINDERLINE3D("module:WalkingEngine:ipTrajectory", robotModel.centerOfMass, originToPendulum.translation, 1, ColorRGBA(0, 0, 0));
  SPHERE3D("module:WalkingEngine:ipTrajectory", robotModel.centerOfMass.x, robotModel.centerOfMass.y, robotModel.centerOfMass.z, 7, ColorRGBA(0, 0, 0));
  SPHERE3D("module:WalkingEngine:ipTrajectory", originToPendulum.translation.x, originToPendulum.translation.y, originToPendulum.translation.z, 3, ColorRGBA(0, 0, 127));

  // Q
  {
    Vector3<> px2 = originToQ* Vector3<>(200.f * 0.75f, 0.f, 0.f);
    Vector3<> p0 = originToQ* Vector3<>(0.f, 0.f, 0.f);
    Vector3<> py2 = originToQ* Vector3<>(0.f, 200.f * 0.75f, 0.f);
    Vector3<> pz2 = originToQ* Vector3<>(0.f, 0.f, 200.f * 0.75f);
    CYLINDERARROW3D("module:WalkingEngine:ipTrajectory", p0, Vector3<>(px2.x, px2.y, px2.z), 2, 20, 10, ColorRGBA(255, 0, 0));
    CYLINDERARROW3D("module:WalkingEngine:ipTrajectory", p0, Vector3<>(py2.x, py2.y, py2.z), 2, 20, 10, ColorRGBA(0, 255, 0));
    CYLINDERARROW3D("module:WalkingEngine:ipTrajectory", p0, Vector3<>(pz2.x, pz2.y, pz2.z), 2, 20, 10, ColorRGBA(0, 0, 255));
  }

  // next Q
  Pose3D originToNextQ = Pose3D(originToQ).conc(predictedPendulumPlayer.nextPhase.s.translation).rotateZ(predictedPendulumPlayer.nextPhase.s.rotation);
  {
    Vector3<> px2 = originToNextQ* Vector3<>(200.f* 0.75f , 0.f, 0.f);
    Vector3<> p0 = originToNextQ* Vector3<>(0.f, 0.f, 0.f);
    Vector3<> py2 = originToNextQ* Vector3<>(0.f, 200.f * 0.75f, 0.f);
    Vector3<> pz2 = originToNextQ* Vector3<>(0.f, 0.f, 200.f * 0.75f);
    CYLINDERARROW3D("module:WalkingEngine:ipTrajectory", p0, Vector3<>(px2.x, px2.y, px2.z), 2, 20, 10, ColorRGBA(255, 0, 0, 90));
    CYLINDERARROW3D("module:WalkingEngine:ipTrajectory", p0, Vector3<>(py2.x, py2.y, py2.z), 2, 20, 10, ColorRGBA(0, 255, 0, 90));
    CYLINDERARROW3D("module:WalkingEngine:ipTrajectory", p0, Vector3<>(pz2.x, pz2.y, pz2.z), 2, 20, 10, ColorRGBA(0, 0, 255, 90));
  }

  // next pendulum origin
  {
    Vector3<> nextR(predictedPendulumPlayer.nextPhase.r.x, predictedPendulumPlayer.nextPhase.r.y, 0.f);
    Vector3<> pz2 = originToNextQ* nextR;
    SPHERE3D("module:WalkingEngine:ipTrajectory", pz2.x, pz2.y, pz2.z, 3, ColorRGBA(0, 0, 127, 90));
  }

  // paint com trajectory
  Vector3<> lastP;
  for(float t = 0.f; t <= predictedPendulumPlayer.phase.duration; t += 0.005f)
  {
    const Vector2<>& k = predictedPendulumPlayer.phase.k;
    const Vector3<> r(predictedPendulumPlayer.phase.r.x, predictedPendulumPlayer.phase.r.y, 0.f);;

    const float tx = t + predictedPendulumPlayer.phase.tu.x;
    const float ty = t + predictedPendulumPlayer.phase.tu.y;

    Vector3<> refToCom(
      predictedPendulumPlayer.phase.x0.x * cosh(k.x * tx) + predictedPendulumPlayer.phase.xv0.x * sinh(k.x * tx) / k.x,
      predictedPendulumPlayer.phase.x0.y * cosh(k.y * ty) + predictedPendulumPlayer.phase.xv0.y * sinh(k.y * ty) / k.y,
      standComPosition.z);

    Vector3<> p = originToQ * (r + refToCom);
    if(t > 0.f)
    {
      LINE3D("module:WalkingEngine:ipTrajectory", lastP.x, lastP.y, lastP.z, p.x, p.y, p.z, 3, ColorRGBA(255, 0, 0));
    }
    lastP = p;
  }

  // paint com trajectory in next phase
  for(float t = 0.f; t <= predictedPendulumPlayer.nextPhase.duration; t += 0.005f)
  {
    const Vector2<>& k = predictedPendulumPlayer.nextPhase.k;
    const Vector3<> r(predictedPendulumPlayer.nextPhase.r.x, predictedPendulumPlayer.nextPhase.r.y, 0.f);;

    const float tx = t + predictedPendulumPlayer.nextPhase.tu.x;
    const float ty = t + predictedPendulumPlayer.nextPhase.tu.y;

    Vector3<> refToCom(
      predictedPendulumPlayer.nextPhase.x0.x * cosh(k.x * tx) + predictedPendulumPlayer.nextPhase.xv0.x * sinh(k.x * tx) / k.x,
      predictedPendulumPlayer.nextPhase.x0.y * cosh(k.y * ty) + predictedPendulumPlayer.nextPhase.xv0.y * sinh(k.y * ty) / k.y,
      standComPosition.z);

    Vector3<> p = originToNextQ * (r + refToCom);
    LINE3D("module:WalkingEngine:ipTrajectory", lastP.x, lastP.y, lastP.z, p.x, p.y, p.z, 3, ColorRGBA(255, 255, 0));
    lastP = p;
  }
}

void WalkingEngine::drawShowcaseIp(const WalkingEngineOutput& walkingEngineOutput) const
{
  //RobotModel robotModel(walkingEngineOutput, theRobotDimensions, theMassCalibration);
  //Pose3D originToQ = robotModel.limbs[predictedPendulumPlayer.phase.type == leftSupportPhase ? MassCalibration::footLeft : MassCalibration::footRight];
  //originToQ.translate(0, 0, -theRobotDimensions.heightLeg5Joint);
  //originToQ.conc(predictedPendulumPlayer.phase.type == leftSupportPhase ? targetStance.leftOriginToFoot.invert() : targetStance.rightOriginToFoot.invert());
  //
  //Pose3D originToPendulum = Pose3D(originToQ).translate(predictedPendulumPlayer.phase.r.x, predictedPendulumPlayer.phase.r.y, 0.f);

  RobotModel robotModel(walkingEngineOutput, theRobotDimensions, theMassCalibration);
  Pose3D originToFoot = robotModel.limbs[predictedPendulumPlayer.phase.type == leftSupportPhase ? MassCalibration::footLeft : MassCalibration::footRight];
  originToFoot.translate(0, 0, -theRobotDimensions.heightLeg5Joint);
  //Pose3D originToFootInv = originToFoot.invert();

  CYLINDERLINE3D("module:WalkingEngine:ip", robotModel.centerOfMass, originToFoot.translation, 1, ColorRGBA(0, 0, 0));
  SPHERE3D("module:WalkingEngine:ip", robotModel.centerOfMass.x, robotModel.centerOfMass.y, robotModel.centerOfMass.z, 7, ColorRGBA(0, 0, 0));
  //SPHERE3D("module:WalkingEngine:ip", originToFoot.translation.x, originToFoot.translation.y, originToFoot.translation.z, 3, ColorRGBA(0, 0, 127));

  //Pose3D originToPendulumInv = originToPendulum.invert();
  //Vector3<> com = originToPendulumInv* robotModel.centerOfMass;
  //Vector3<> nextToCom = RotationMatrix::fromRotationX(fromDegrees(6)) * com;
  //Vector3<> px2 = originToPendulum * nextToCom;
  //CYLINDERARROW3D("module:WalkingEngine:ip", robotModel.centerOfMass, px2, 2, 10, 6, ColorRGBA(0, 127, 127));
  //CYLINDERARROW3D("module:WalkingEngine:ip", robotModel.centerOfMass, Vector3<>(robotModel.centerOfMass.x, robotModel.centerOfMass.y, robotModel.centerOfMass.z - 100.f), 2, 10, 6, ColorRGBA(0, 127, 127));
  //SPHERE3D("module:WalkingEngine:ip", originToPendulum.translation.x, originToPendulum.translation.y, originToPendulum.translation.z, originToPendulum.invert().conc(robotModel.centerOfMass).translation.abs(), ColorRGBA(255, 80, 80, 127));

  // origin of the inverted pendulum
  {
    Vector3<> px2 = originToFoot * Vector3<>(200.f, 0.f, 0.f);
    Vector3<> p0 = originToFoot * Vector3<>(0.f, 0.f, 0.f);
    Vector3<> py2 = originToFoot * Vector3<>(0.f, 200.f, 0.f);
    Vector3<> pz2 = originToFoot * Vector3<>(0.f, 0.f, 200.f);
    CYLINDERARROW3D("module:WalkingEngine:ip", p0, Vector3<>(px2.x, px2.y, px2.z), 2, 20, 10, ColorRGBA(255, 0, 0));
    CYLINDERARROW3D("module:WalkingEngine:ip", p0, Vector3<>(py2.x, py2.y, py2.z), 2, 20, 10, ColorRGBA(0, 255, 0));
    CYLINDERARROW3D("module:WalkingEngine:ip", p0, Vector3<>(pz2.x, pz2.y, pz2.z), 2, 20, 10, ColorRGBA(0, 0, 255));
  }
}

void WalkingEngine::drawShowcaseIpSphere(const WalkingEngineOutput& walkingEngineOutput) const
{
  RobotModel robotModel(walkingEngineOutput, theRobotDimensions, theMassCalibration);
  Pose3D originToFoot = robotModel.limbs[predictedPendulumPlayer.phase.type == leftSupportPhase ? MassCalibration::footLeft : MassCalibration::footRight];
  originToFoot.translate(0, 0, -theRobotDimensions.heightLeg5Joint);

  SPHERE3D("module:WalkingEngine:ipSphere", originToFoot.translation.x, originToFoot.translation.y, originToFoot.translation.z - 125.f, originToFoot.invert().conc(robotModel.centerOfMass).translation.abs() + 100.f, ColorRGBA(255, 80, 80, 0));
  SPHERE3D("module:WalkingEngine:ipSphere", originToFoot.translation.x, originToFoot.translation.y, originToFoot.translation.z, originToFoot.invert().conc(robotModel.centerOfMass).translation.abs(), ColorRGBA(255, 80, 80, 127));
}

void WalkingEngine::drawShowcase3dlipmPlane(const WalkingEngineOutput& walkingEngineOutput) const
{
  RobotModel robotModel(walkingEngineOutput, theRobotDimensions, theMassCalibration);
  Pose3D originToFoot = robotModel.limbs[predictedPendulumPlayer.phase.type == leftSupportPhase ? MassCalibration::footLeft : MassCalibration::footRight];
  originToFoot.translate(0, 0, -theRobotDimensions.heightLeg5Joint);
  Pose3D originToFootInv = originToFoot.invert();

  // pendulum plane
  {
    Vector3<> com = Pose3D(originToFootInv).conc(theRobotModel.centerOfMass).translation;
    //Vector3<> center = originToFoot* Vector3<>(0.f, 0.f / *pendulumPlayer.supportLeg == left ? -50.f : 50.f* /, com.z);
    //CYLINDER3D("module:WalkingEngine:3dlipm", center.x, center.y, center.z, 0, 0, 0, 200, 0, ColorRGBA(80, 80, 80, 127));
    Vector3<> ltc = originToFoot * Vector3<>(200.f, 200.f, com.z);
    Vector3<> rtc = originToFoot * Vector3<>(200.f, -200.f, com.z);
    Vector3<> lbc = originToFoot * Vector3<>(-200.f, 200.f, com.z);
    Vector3<> rbc = originToFoot * Vector3<>(-200.f, -200.f, com.z);
    QUAD3D("module:WalkingEngine:3dlipmPlane", ltc, rtc, rbc, lbc, ColorRGBA(255, 80, 80, 127));
    QUAD3D("module:WalkingEngine:3dlipmPlane", lbc, rbc, rtc, ltc, ColorRGBA(255, 80, 80, 127));
  }
}

#define DASHLINE3D(id, from, to, size, dashLen, color) \
  do \
  { \
    Vector3<> dir = (to) - (from); \
    const float len = dir.abs(); \
    dir.normalize(); \
    for(float start = 0.f; start < len;) \
    { \
      float end = start + dashLen; \
      Vector3<> startP = from + dir * start; \
      Vector3<> endP = from + dir * end; \
      LINE3D(id, startP.x, startP.y, startP.z, endP.x, endP.y, endP.z, size, color); \
      start = end + dashLen; \
    } \
  } while(false)

void WalkingEngine::drawShowcaseSimpleIp(const WalkingEngineOutput& walkingEngineOutput) const
{
  Vector3<> pendulumMass(90.f, 70.f, 180.f);

  CYLINDERLINE3D("module:WalkingEngine:simpleIp", Vector3<>(), pendulumMass, 3, ColorRGBA(50, 50, 50));
  SPHERE3D("module:WalkingEngine:simpleIp", 0.f, 0.f, 0.f, 3, ColorRGBA(50, 50, 50));

  SPHERE3D("module:WalkingEngine:simpleIp", pendulumMass.x, pendulumMass.y, pendulumMass.z, 10, ColorRGBA(40, 40, 40));

  ColorRGBA dashLineColor(0, 0, 0, 80);

  //DASHLINE3D("module:WalkingEngine:simpleIp", Vector3<>(pendulumMass.x, pendulumMass.y, 0.f), pendulumMass, 2, 4.f, dashLineColor);
  //DASHLINE3D("module:WalkingEngine:simpleIp", Vector3<>(pendulumMass.x, pendulumMass.y, 0.f), Vector3<>(pendulumMass.x, 0.f, 0.f), 2, 4.f, dashLineColor);
  //DASHLINE3D("module:WalkingEngine:simpleIp", Vector3<>(pendulumMass.x, pendulumMass.y, 0.f), Vector3<>(0.f, pendulumMass.y, 0.f), 2, 4.f, dashLineColor);

  //DASHLINE3D("module:WalkingEngine:simpleIp", Vector3<>(pendulumMass.x, 0.f, pendulumMass.z), pendulumMass, 2, 4.f, dashLineColor);
  //DASHLINE3D("module:WalkingEngine:simpleIp", Vector3<>(pendulumMass.x, 0.f, pendulumMass.z), Vector3<>(0.f, 0.f, pendulumMass.z), 2, 4.f, dashLineColor);
  //DASHLINE3D("module:WalkingEngine:simpleIp", Vector3<>(pendulumMass.x, 0.f, pendulumMass.z), Vector3<>(pendulumMass.x, 0.f, 0.f), 2, 4.f,dashLineColor);

  DASHLINE3D("module:WalkingEngine:simpleIp", Vector3<>(0.f, pendulumMass.y, pendulumMass.z), pendulumMass, 2, 4.f, dashLineColor);
  DASHLINE3D("module:WalkingEngine:simpleIp", Vector3<>(0.f, pendulumMass.y, pendulumMass.z), Vector3<>(0.f, 0.f, pendulumMass.z), 2, 4.f, dashLineColor);
  DASHLINE3D("module:WalkingEngine:simpleIp", Vector3<>(0.f, pendulumMass.y, pendulumMass.z), Vector3<>(0.f, pendulumMass.y, 0.f), 2, 4.f,dashLineColor);

  // origin of the inverted pendulum
  {
    Vector3<> px2 = Vector3<>(200.f, 0.f, 0.f);
    Vector3<> p0 = Vector3<>(0.f, 0.f, 0.f);
    Vector3<> py2 = Vector3<>(0.f, 200.f, 0.f);
    Vector3<> pz2 = Vector3<>(0.f, 0.f, 220.f);
    CYLINDERARROW3D("module:WalkingEngine:simpleIp", p0, Vector3<>(px2.x, px2.y, px2.z), 2, 20, 10, ColorRGBA(255, 0, 0));
    CYLINDERARROW3D("module:WalkingEngine:simpleIp", p0, Vector3<>(py2.x, py2.y, py2.z), 2, 20, 10, ColorRGBA(0, 255, 0));
    CYLINDERARROW3D("module:WalkingEngine:simpleIp", p0, Vector3<>(pz2.x, pz2.y, pz2.z), 2, 20, 10, ColorRGBA(0, 0, 255));
  }
}

void WalkingEngine::drawShowcaseZmp(const WalkingEngineOutput& walkingEngineOutput) const
{
  // compute Q
  RobotModel robotModel(walkingEngineOutput, theRobotDimensions, theMassCalibration);
  Pose3D originToQ = robotModel.limbs[predictedPendulumPlayer.phase.type == leftSupportPhase ? MassCalibration::footLeft : MassCalibration::footRight];
  originToQ.translate(0, 0, -theRobotDimensions.heightLeg5Joint);
  originToQ.conc(predictedPendulumPlayer.phase.type == leftSupportPhase ? targetStance.leftOriginToFoot.invert() : targetStance.rightOriginToFoot.invert());

  Vector3<> ankle = robotModel.limbs[MassCalibration::footLeft].translation;
  CYLINDERARROW3D("module:WalkingEngine:zmp", Vector3<>(ankle.x + 10, ankle.y - 10, ankle.z + 30), ankle, 1, 6, 3, ColorRGBA(0, 127, 127)); // F_A
  CYLINDERARROW3D("module:WalkingEngine:zmp", ankle, Vector3<>(ankle.x - 20, ankle.y + 20, ankle.z + 40), 1, 6, 3, ColorRGBA(0x55, 0x0, 0x0));

  Vector3<> footCom = Pose3D(robotModel.limbs[MassCalibration::footLeft]).conc(theMassCalibration.masses[MassCalibration::footLeft].offset).translation;
  Vector3<> footCom2 = Pose3D(robotModel.limbs[MassCalibration::footLeft]).conc(theMassCalibration.masses[MassCalibration::footLeft].offset).translate(0.f, 0.f, -30).translation;
  CYLINDERARROW3D("module:WalkingEngine:zmp", footCom, footCom2, 1, 6, 3, ColorRGBA(0, 127, 127)); // m_s * g
  SPHERE3D("module:WalkingEngine:zmp", footCom.x, footCom.y, footCom.z, 3, ColorRGBA(255, 0, 0));

  Vector3<> p = originToQ* Vector3<>(40.f, 50.f, 0.f);
  Vector3<> mend = originToQ* Vector3<>(40.f, 50.f, -20.f);
  Vector3<> rstart = originToQ* Vector3<>(50.f, 60.f, -40.f).normalize(110.f);

  CYLINDERARROW3D("module:WalkingEngine:zmp", rstart, p, 1, 6, 3, ColorRGBA(0, 127, 127));
  CYLINDERARROW3D("module:WalkingEngine:zmp", p, mend, 1, 6, 3, ColorRGBA(0x55, 0x0, 0x0));
}

void WalkingEngine::declarePlots()
{
  drawComPlot();
}

void WalkingEngine::drawComPlot()
{
  DECLARE_PLOT("module:WalkingEngine:absComX");
  DECLARE_PLOT("module:WalkingEngine:absComY");
  DECLARE_PLOT("module:WalkingEngine:absMeaX");
  DECLARE_PLOT("module:WalkingEngine:absMeaY");
  DECLARE_PLOT("module:WalkingEngine:absRefX");
  DECLARE_PLOT("module:WalkingEngine:absRefY");

  DEBUG_RESPONSE_ONCE("module:WalkingEngine:resetComPlot",
    drawComPlotOrigin = Vector2<>();
    drawComPlotLastR = Vector2<>();
    drawComPlotState = 0;
  );

  //if(drawComPlotState > 0 || theMotionRequest.walkRequest.speed.translation.x != 0.f)
  {
    if(sgn(controlledPendulumPlayer.phase.r.y) != sgn(drawComPlotLastR.y))
    {
      drawComPlotOrigin.x += controlledPendulumPlayer.phase.s.translation.x;
      drawComPlotOrigin.y += controlledPendulumPlayer.phase.s.translation.y;
    }
    if(drawComPlotState == 0)
      drawComPlotOrigin = Vector2<>();

    Vector2<> r = drawComPlotOrigin + controlledPendulumPlayer.phase.r;
    Vector2<> comX = drawComPlotOrigin + controlledPendulumPlayer.state.p;
    Vector2<> meaX = drawComPlotOrigin + controlledPendulumPlayer.state.pMeasured;

    //if(drawComPlotState < 150)
    {
      PLOT("module:WalkingEngine:absComX", comX.x);
      PLOT("module:WalkingEngine:absComY", comX.y);
      PLOT("module:WalkingEngine:absMeaX", meaX.x);
      PLOT("module:WalkingEngine:absMeaY", meaX.y);
      PLOT("module:WalkingEngine:absRefX", r.x);
      PLOT("module:WalkingEngine:absRefY", r.y);
      //++drawComPlotState;
    }
    drawComPlotLastR = controlledPendulumPlayer.phase.r;
  }
}
*/