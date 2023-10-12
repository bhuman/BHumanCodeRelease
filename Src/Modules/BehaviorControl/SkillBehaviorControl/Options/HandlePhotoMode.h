option(HandlePhotoMode)
{
  initial_state(notInPhotoMode)
  {
    transition
    {
      if(true) // TODO: Update condition
        goto photoMode;
    }
  }

  state(photoMode)
  {
    action
    {
      theCalibrateFootSoleSkill(); // TODO: Create setPhotoMode skill
      theLookForwardSkill();
    }
  }
}
