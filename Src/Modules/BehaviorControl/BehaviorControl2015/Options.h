/** All option files that belong to the current behavior have to be included by this file. */

#include "Options/Soccer.h"
#include "Options/GameControl/HandleGameState.h"
#include "Options/GameControl/HandlePenaltyState.h"
#include "Options/GameControl/PlayingState.h"
#include "Options/GameControl/ReadyState.h"

#include "Options/HeadControl/LookForward.h"

#include "Options/Output/ArmMotionRequest/PointAt.h"

#include "Options/Output/HeadMotionRequest/SetHeadPanTilt.h"

#include "Options/Output/MotionRequest/SpecialAction.h"
#include "Options/Output/MotionRequest/Stand.h"
#include "Options/Output/MotionRequest/WalkAtSpeedPercentage.h"
#include "Options/Output/MotionRequest/WalkToTarget.h"
#include "Options/Output/MotionRequest/InWalkKick.h"
#include "Options/Output/MotionRequest/GetUpEngine.h"
#include "Options/Output/MotionRequest/Dive.h"
#include "Options/Output/PlaySound.h"
#include "Options/Output/PlaySamples.h"

#include "Options/Skills/GetUp.h"

#include "Options/Roles/Striker.h"
#include "Options/Roles/Goaler.h"

#include "Options/DemoOptions/Demo.h"
#include "Options/DemoOptions/Waving.h"

#include "Options/Tools/ButtonPressedAndReleased.h"
