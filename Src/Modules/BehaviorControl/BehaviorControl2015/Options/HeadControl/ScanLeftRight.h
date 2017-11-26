/**
 * @file ScanLeftRight.h
 * Fichier pour géré le movement de tête du NAO
 * @author Benjamin "Naova2017"
 * @author Modified David Boulay "Naova2017"
 */


/**option(LookForward, (float) (0.38f) tilt, (float) (0.5f) pan, (float) (-1.0f) pan2)**/

option(ScanLeftRight)
{
	common_transition
  {
    if(theGameInfo.state != STATE_INITIAL && libCodeRelease.timeSinceBallWasSeen() > 200)
      goto lookLeft;

    switch(ScanLeftRightMode)
    {
      case ScanLeftRight::Off:
        goto off;
      case ScanLeftRight::lookLeft:
        goto lookLeft;
      case ScanLeftRight::lookRight:
        goto lookRight;
	  case ScanLeftRight::restore:
        goto restore;
      default:
        goto none;
    }
  }
	
	
  initial_state(none) {}
  
  state(off) {action SetHeadPanTilt(JointAngles::off, JointAngles::off, 0.f);}
  
  state(lookLeft)
 /** {
	transition
    {
      if(libCodeRelease.timeSinceBallWasSeen() > 500 )
        goto lookRight;
    }**/
    action
    {
      SetHeadPanTilt(0.5f, 0.38f, 2.f);
	  
    }
/**  }**/
  state(lookRight)
/**  {
	transition
    {
      if(libCodeRelease.timeSinceBallWasSeen() > 500 )
        goto lookLeft;
	  else if (libCodeRelease.timeSinceBallWasSeen() == 0 )
        goto restore;
    }**/
    action
    {
      SetHeadPanTilt(-1.0f, 0.38f, 2.f);
    }
/**  }**/
  state(restore)
  {
    action
    {
      Stand();
	  LookForward();
    }
  }

}

struct ScanLeftRight
{
  ENUM(Mode,
  {,
	none,
    Off,
    lookLeft,
    lookRight,
	restore,
  });
};

ScanLeftRight::Mode ScanLeftRightMode = ScanLeftRight::Mode::none; /**< The head control mode executed by the option HeadControl. */
