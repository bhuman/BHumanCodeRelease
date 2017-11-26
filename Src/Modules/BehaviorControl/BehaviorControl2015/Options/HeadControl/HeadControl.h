/**
 * @file HeadControl.h
 * Created Switch case to control head orientation and movement.
 * @author inital "BHuman21015
 * @author Modifications David Boulay "Naova2017"
 * @author Modifications Benjamin "Naova2017"
 */



option(ThatHeadControl)
{
  common_transition
  {
    if(/**!theGroundContactState.contact && **/theGameInfo.state != STATE_INITIAL)
	/**if(libCodeRelease.timeSinceBallWasSeen() > 500)**/
      /**goto lookForward;**/
	  goto Searching;

    switch(ThatHeadControlMode)
    {
      case ThatHeadControl::off:
        goto off;
      case ThatHeadControl::lookForward:
        goto lookForward;
      case ThatHeadControl::Searching:
        goto Searching;
      default:
        goto none;
    }
  }

  initial_state(none) {}
  
  state(off) {
			 action SetHeadPanTilt(JointAngles::off, JointAngles::off, 0.f);
			 }
			 
  state(lookForward) 
			 {
			 action LookForward();
			 }
			 
  state(Searching) 
	{
		action 
		if (libCodeRelease.timeSinceBallWasSeen() < 4000)
			{
		 SetHeadPanTilt(1.f, .5f, .75f );
			}
		if (libCodeRelease.timeSinceBallWasSeen() > 10000 and libCodeRelease.timeSinceBallWasSeen()< 15000)
			{
		 SetHeadPanTilt(-1.f, .5f, .75f );
			}
		if (libCodeRelease.timeSinceBallWasSeen() > 15000 ) 
			{
		 SetHeadPanTilt(1.f, .5f, .75f );  
			}

		ScanLeftRight::restore;
		/**SetHeadPanTilt(1.f, 1.f, 2.f);**/
	}
  
  
  
  
  
  
  
  
}

struct ThatHeadControl
{
  ENUM(Mode,
  {,
    none,
    off,
    lookForward,
	Searching,
  });
};

ThatHeadControl::Mode ThatHeadControlMode = ThatHeadControl::Mode::none; /**< The head control mode executed by the option HeadControl. */
