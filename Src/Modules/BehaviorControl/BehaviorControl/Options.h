/** All option files that belong to the current behavior have to be included by this file. */

#include "Options/Soccer.h"

#include "Options/HeadControl/HeadControl.h"

#include "Options/GameControl/HandleGameState.h"
#include "Options/GameControl/HandlePenaltyState.h"
#include "Options/GameControl/PlayingState.h"
#include "Options/GameControl/ReadyState.h"

#include "Options/HeadControl/LookForward.h"

#include "Options/Output/Activity.h"
#include "Options/Output/Annotation.h"
#include "Options/Output/HeadMotionRequest/SetHeadPanTilt.h"
#include "Options/Output/MotionRequest/InWalkKick.h"
#include "Options/Output/MotionRequest/SpecialAction.h"
#include "Options/Output/MotionRequest/Stand.h"
#include "Options/Output/MotionRequest/WalkAtAbsoluteSpeed.h"
#include "Options/Output/MotionRequest/WalkAtRelativeSpeed.h"
#include "Options/Output/MotionRequest/WalkToTarget.h"
#include "Options/Output/MotionRequest/GetUpEngine.h"
#include "Options/Output/PlaySound.h"
#include "Options/Roles/Striker.h"
#include "Options/Roles/Goaler.h"
#include "Options/Skills/GetUp.h"

#include "Options/DemoOptions/Demo.h"
#include "Options/DemoOptions/Waving.h"

#include "Options/Tools/ButtonPressedAndReleased.h"
