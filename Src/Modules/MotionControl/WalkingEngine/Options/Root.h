/**
 * The root option decides whether this module is active or not.
 * In the first case, UNSW data is updated, a request for the UNSW walk generator is created and then executed.
 * Finally, the output of this module is created. If this module is inactive, everything is set to default
 * values.
 */
option(Root)
{
  initial_state(inactive)
  {
    transition
    {
      if(theLegMotionSelection.ratios[MotionRequest::walk] > 0.f || theLegMotionSelection.ratios[MotionRequest::stand] > 0.f)
        goto active;
    }
    action
    {
      WalkingEngineOutput& w = const_cast<WalkingEngineOutput&>(theWalkingEngineOutput);
      w.standing = true;
      w.odometryOffset = Pose2f();
      w.speed = Pose2f();
      w.isLeavingPossible = true;
      w.maxSpeed = theWalkGenerator.maxSpeed;
      theWalkGenerator.reset();
      jointRequest.angles[Joints::firstArmJoint] = JointAngles::off;
    }
  }

  state(active)
  {
    transition
    {
      if(theLegMotionSelection.ratios[MotionRequest::walk] == 0.f && theLegMotionSelection.ratios[MotionRequest::stand] == 0.f)
        goto inactive;
    }
    action
    {
      UpdateRequest();

      if(theWalkGenerator.forceStand)
      {
        speed = Pose2f();
        walkMode = WalkGenerator::speedMode;
      }

      bool stepStarted = theWalkGenerator.t == 0;
      theWalkGenerator.calcJoints(speed, target, walkMode, getKickFootOffset);
      jointRequest = theWalkGenerator.jointRequest;
      updateOutput(stepStarted, const_cast<WalkingEngineOutput&>(theWalkingEngineOutput));
    }
  }
}
