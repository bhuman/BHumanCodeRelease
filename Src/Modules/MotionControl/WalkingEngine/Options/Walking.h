/**
 * The option fills a request for the UNSW walk generator based on the current B-Human
 * walk request.
 */
option(Walking)
{
  common_transition
  {
    if(walkRequest.mode == WalkRequest::absoluteSpeedMode)
      goto walkingWithSpeed;
    else if(walkRequest.mode == WalkRequest::relativeSpeedMode)
      goto walkingWithSpeedPercentage;
    else
      goto walkToTarget;
  }

  initial_state(walkingWithSpeed)
  {
    action
    {
      walk(walkRequest.speed);
    }
  }

  state(walkingWithSpeedPercentage)
  {
    action
    {
      walk(Pose2f(walkRequest.speed.rotation * theWalkGenerator.maxSpeed.rotation,
                  walkRequest.speed.translation.x() * theWalkGenerator.maxSpeed.translation.x(),
                  walkRequest.speed.translation.y() * theWalkGenerator.maxSpeed.translation.y()));
    }
  }

  state(walkToTarget)
  {
    action
    {
      walk(Pose2f(walkRequest.speed.rotation * theWalkGenerator.maxSpeed.rotation,
                  walkRequest.speed.translation.x() * theWalkGenerator.maxSpeed.translation.x(),
                  walkRequest.speed.translation.y() * theWalkGenerator.maxSpeed.translation.y()),
           walkRequest.target);
    }
  }
}
