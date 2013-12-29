/*
* @file BallTaking.cpp
* @author Thomas Muender
*/

#include "BallTaking.h"
#include "Tools/InverseKinematic.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Math/Geometry.h"

/*
 * decide phase execution
 */
void BallTaking::update(BallTakingOutput& output)
{
  output.isLeavingPossible = false;
  ballData(output);
  output.isTakable = takable.isFilled() && takable.getAverage() == 1;

  if(theMotionSelection.ratios[MotionRequest::takeBall] > 0)
  { 
    //finished
    if(phaseLeavingPossible && phase == 5)
    {
      output.isLeavingPossible = true;
      //todo odometry offset

      side = 0;
      phase = 0;
      phaseStart = 0;
      phaseLeavingPossible = false;

      return;
    }
    else if(phaseLeavingPossible)
    {
      phase++;
      phaseStart = theFrameInfo.time;
      phaseLeavingPossible = false;
    }
     
         if(phase == 0) phaseZero(output);
    else if(phase == 1) phaseOne(output);
    else if(phase == 2) phaseTwo(output);
    else if(phase == 3) phaseThree(output);
    else if(phase == 4) phaseFour(output);
    else if(phase == 5) phaseFive(output);

    int hardness = 90;
    output.jointHardness.hardness[JointData::LHipRoll] = hardness;
    output.jointHardness.hardness[JointData::LHipPitch] = hardness;
    output.jointHardness.hardness[JointData::LKneePitch] = hardness;
    output.jointHardness.hardness[JointData::LAnklePitch] = hardness;
    output.jointHardness.hardness[JointData::LAnkleRoll] = hardness;
    output.jointHardness.hardness[JointData::RHipRoll] = hardness;
    output.jointHardness.hardness[JointData::RHipPitch] = hardness;
    output.jointHardness.hardness[JointData::RKneePitch] = hardness;
    output.jointHardness.hardness[JointData::RAnklePitch] = hardness;
    output.jointHardness.hardness[JointData::RAnkleRoll] = hardness;
    
    output.angles[JointData::LShoulderPitch] = JointData::ignore;
    output.angles[JointData::LShoulderRoll] = JointData::ignore;
    output.angles[JointData::RShoulderPitch] = JointData::ignore;
    output.angles[JointData::RShoulderRoll] = JointData::ignore;
    output.angles[JointData::LElbowRoll] = JointData::ignore;
    output.angles[JointData::LElbowYaw] = JointData::ignore;
    output.angles[JointData::RElbowRoll] = JointData::ignore;
    output.angles[JointData::RElbowYaw] = JointData::ignore;
    
    output.angles[JointData::HeadPitch] = JointData::ignore;
    output.angles[JointData::HeadYaw] = JointData::ignore;
  }
}


/*
 * calculate ball data
 */
void BallTaking::ballData(BallTakingOutput& output)
{
  //crossing y axis
  Geometry::getIntersectionOfLines(Geometry::Line(theBallModel.estimate.position, (theBallModel.estimate.velocity - theBallModel.estimate.position)), Geometry::Line(Vector2<>(0,0), Vector2<>(0,1)), crossing);
  //float ttr = theBallModel.estimate.timeForDistance((crossing - theBallModel.estimate.position).abs(), -0.4f);
  moved.add(theBallModel.estimate.velocity.abs());
  
  bool between = crossing.y < 90.f && crossing.y > -90.f && theBallModel.estimate.velocity.abs() > 1.f;
  bool notClose = theBallModel.estimate.position.abs() > 300.f;
  bool stopsBehind = theBallModel.estimate.getEndPosition(-0.4f).x < -150.f;
  bool angle = std::fabs(theBallModel.estimate.velocity.angle()) > 1.92f;
  bool moving = moved.getAverageFloat() > 10.f;
  bool wasSeen = theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) < 100.f;
  //bool time = ttr > 0.3f && ttr < 1.5f;
  
  valid = between && notClose && stopsBehind && angle && moving && wasSeen; //&& time;
  takable.add(valid ? 1 : 0);
}

/*
 * choose side 2
 */
void BallTaking::phaseZero(BallTakingOutput& output)
{
  InverseKinematic::calcLegJoints(standLeftFoot, standRightFoot, output, theRobotDimensions, 0.5f);
  
  if(valid)
  {
    side += crossing.y;
    c++;
  }
  else c--;
  if(c < 0) c = 0;
  
  //leaving possible ?
  if(c > 50)
  {
    c = 0;
    phaseLeavingPossible = true;
  }
  
  //abort
  if(theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) > 1000 || (moved.isFilled() && moved.getAverageFloat() < 1))
  {
    phase = 5;
    phaseLeavingPossible = true;
  }
}


/*
 * bank in
 */
void BallTaking::phaseOne(BallTakingOutput& output)
{
  int t = theFrameInfo.getTimeSince(phaseStart);
  float tp = (float)t / phaseDuration;

  y = sinUp(tp) * xFactor;
  rotX = sinUp(tp) * rotXFactor;
  zStand = sinUp(tp) * zStandFactor;
  rotZ = sinUp(tp) * rot;
  y2 = sinUp(tp) * y2Factor;

  if(side > 0) //left swing 
  {
    targetLeftFootPositon = Pose3D(standLeftFoot.translation.x + (y2 * 80), standLeftFoot.translation.y + y + (y2 * 80), standLeftFoot.translation.z + zStand);
    targetRightFootPositon = Pose3D(standRightFoot.translation.x, standRightFoot.translation.y + y, standRightFoot.translation.z + zStand);

    targetLeftFootPositon.rotateZ(rotZ);
    
    output.angles[JointData::RHipRoll] = -rotX;
    output.angles[JointData::LHipRoll] = rotX - y2;
  }
  else //right swing
  {
    targetLeftFootPositon = Pose3D(standLeftFoot.translation.x, standLeftFoot.translation.y - y, standLeftFoot.translation.z + zStand);
    targetRightFootPositon = Pose3D(standRightFoot.translation.x + (y2 * 80), standRightFoot.translation.y - y - (y2 * 80), standRightFoot.translation.z + zStand);

    targetRightFootPositon.rotateZ(-rotZ);
    
    output.angles[JointData::RHipRoll] = rotX - y2;
    output.angles[JointData::LHipRoll] = -rotX;
  }
  if(!InverseKinematic::calcLegJoints(targetLeftFootPositon, targetRightFootPositon, output, theRobotDimensions, 0.5f))
    OUTPUT_WARNING("not reachable 1");

  //leaving possible ?
  if(theFrameInfo.getTimeSince(phaseStart) >= phaseDuration)
  {
    leftEndPhaseOne = targetLeftFootPositon;
    rightEndPhaseOne = targetRightFootPositon;
    phaseLeavingPossible = true;
  }
}


/*
 * wait for taking
 */
void BallTaking::phaseTwo(BallTakingOutput& output)
{
  if(!InverseKinematic::calcLegJoints(leftEndPhaseOne, rightEndPhaseOne, output, theRobotDimensions, 0.5f))
    OUTPUT_WARNING("not reachable 2");

  bool time = theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) > 500;
  bool position = theBallModel.estimate.position.x < 0;
  bool moving = moved.isFilled() && moved.getAverageFloat() < 1; //todo test

  if(time || position || moving)
  {
    if(theBallModel.estimate.position.abs() > 200)
    {
      phase = 4;
      leftEndPhaseFour = leftEndPhaseOne;
      rightEndPhaseFour = rightEndPhaseOne;
    }
    phaseLeavingPossible = true;
  }
}


/*
 * move ball out 1
 */
void BallTaking::phaseThree(BallTakingOutput& output)
{
  int t = theFrameInfo.getTimeSince(phaseStart);
  float tp = sinUp((float)t / phaseDuration);

  targetLeftFootPositon = leftEndPhaseOne;
  targetRightFootPositon = rightEndPhaseOne;
  
  if(side > 0) //left swing
  {   
    targetLeftFootPositon.translation += Vector3<>(-tp * 20, -tp * 35, 0);
  }
  else //right swing
  {
    targetRightFootPositon.translation += Vector3<>(-tp * 20, tp * 35, 0);
  }
  
  if(!InverseKinematic::calcLegJoints(targetLeftFootPositon, targetRightFootPositon, output, theRobotDimensions, 0.5f))
    OUTPUT_WARNING("not reachable 3");
  
  //leaving possible ?
  if(theFrameInfo.getTimeSince(phaseStart) >= phaseDuration)
  {
    leftEndPhaseThree = targetLeftFootPositon;
    rightEndPhaseThree = targetRightFootPositon;
    phaseLeavingPossible = true;
  }
}


/*
 * move ball out 2
 */
void BallTaking::phaseFour(BallTakingOutput& output)
{
  int t = theFrameInfo.getTimeSince(phaseStart);
  float tp = sinUp((float)t / phaseDuration);

  targetLeftFootPositon = leftEndPhaseThree;
  targetRightFootPositon = rightEndPhaseThree;
  
  if(side > 0) //left swing
  {    
    targetLeftFootPositon.translation += Vector3<>(tp * 100, 0, tp * 25);
    targetRightFootPositon.translation += Vector3<>(0, 0, tp * 20);
  }
  else //right swing
  {
    targetLeftFootPositon.translation += Vector3<>(0, 0, tp * 20);
    targetRightFootPositon.translation += Vector3<>(tp * 100, 0, tp * 25);
  }
  
  if(!InverseKinematic::calcLegJoints(targetLeftFootPositon, targetRightFootPositon, output, theRobotDimensions, 0.5f))
    OUTPUT_WARNING("not reachable 4");
  
  //leaving possible ?
  if(theFrameInfo.getTimeSince(phaseStart) >= phaseDuration)
  {
    leftEndPhaseFour = targetLeftFootPositon;
    rightEndPhaseFour = targetRightFootPositon;
    phaseLeavingPossible = true;
  }
}


/*
 * get back to stand
 */
void BallTaking::phaseFive(BallTakingOutput& output)
{
  int t = theFrameInfo.getTimeSince(phaseStart);
  float tp = sinUp((float)t / phaseDuration);

  targetLeftFootPositon = leftEndPhaseFour;
  targetRightFootPositon = rightEndPhaseFour;

  targetLeftFootPositon.translation = leftEndPhaseFour.translation + (standLeftFoot.translation - leftEndPhaseFour.translation) * tp;
  targetRightFootPositon.translation = rightEndPhaseFour.translation + (standRightFoot.translation - rightEndPhaseFour.translation) * tp;

  if(side > 0) //left swing
  {
    targetLeftFootPositon.rotateZ(-rot * tp);
  }
  else //right swing
  {
    targetRightFootPositon.rotateZ(rot * tp);
  }
  
  if(!InverseKinematic::calcLegJoints(targetLeftFootPositon, targetRightFootPositon, output, theRobotDimensions, 0.5f))
    OUTPUT_WARNING("not reachable 5");
  
  //leaving possible ?
  if(theFrameInfo.getTimeSince(phaseStart) >= phaseDuration)
  {
    phaseLeavingPossible = true;
  }
}

MAKE_MODULE(BallTaking, Motion Control)