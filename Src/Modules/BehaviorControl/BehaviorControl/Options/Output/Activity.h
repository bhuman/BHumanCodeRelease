/** Set the current robot activity */
option(Activity, (BehaviorStatus::Activity) activity)
{
  /* Set the activity */
  initial_state(setActivity)
  {
    action
    {
      theBehaviorStatus.activity = activity;
    }
  }
}
