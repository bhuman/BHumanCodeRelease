/**
 * @file Options.h
 *
 * This file declares all options in the SkillBehaviorControl.
 *
 * @author Thomas Röfer
 */

// @todo Add descriptions

option(HandleAnyPlaceDemo);
option(HandleBallOutOfField);
option(HandleGameState);
option(HandleIllegalAreas);
option(HandleJoystick);
option(HandleJoystickWalk, args((const JoystickState&) joystickState, (bool) isOmnidirectionalWalk));
option(HandleJoystickHead, args((const JoystickState&) joystickState, (bool) isOmnidirectionalWalk, (bool) isLookDown, (bool) isKicking));
option(HandlePenaltyKick);
option(HandlePenaltyShootout);
option(HandlePhysicalRobot);
option(HandlePlayerState);
option(HandlePlayingOptions);
option(HandleReturnFromTouchline);
option(HandleStoppedState);
option(HandleStrikerLostBall);
option(PenaltyKeeper);
option(PenaltyTaker);
option(PlaySoccer);
