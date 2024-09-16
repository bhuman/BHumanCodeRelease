/**
 * @file SharedAutonomyRequest.h
 *
 * The file declares the messages for Joystick commands.
 *
 * @author Paul Deiß
 * @author Tatjana Thielke
 * @author Thomas Röfer
 */

#pragma once

#include "Streaming/AutoStreamable.h"

STREAMABLE(SharedAutonomyRequest,
{
  ENUM(Operations,
  {,
    walkToTarget, /**< walk button */
    dribbleInDirection, /**< Dribble in the global direction targetPose.rotation. */
    dribbleToPoint, /**< dribble button */
    passToMate, /**< pass to mate button */
    passToPoint, /**< kick button */
    zweikampf, /**< zweikampf button */
    stand, /**< initial state */
    sit, /**< sit down */
    standHigh, /**< initial state */
    walkAtRelativeSpeed, /**< Set targetPose as relative walk speed. */
    playBallDribbleInDirection, /**< Tell PlayBall to dribble in the global direction targetPose.rotation. */
    playBallDribbleToPoint, /**< Tell PlayBall to dribble to a point at the field. */
    playBallPassToMate, /**< Tell PlayBall to pass to the only teammate. */
    playBallKickToPoint, /**< Tell PlayBall to kick to the global point targetPose.translation. */
  });

  STREAMABLE(HeadRequest,
  {,
    (bool)(false) manual, /**< if the head is controlled manually */
    (Vector2a)(Vector2a::Zero()) angle, /**< the angle for the head */
  });

  void draw() const,

  (HeadRequest) headRequest,
  (bool)(false) isValid, /**< if the challenge is running */
  (Pose2f) targetPose, /**< the target pose of the robot or the ball as provided by the remote control */
  (Vector2f) arrowPosition, /**< the end of the drawn arrow that shows the new orientation of the robot */
  (bool)(false) mousePressed, /**< if the left mouse button is pressed */
  (Operations)(stand) controlOperations, /**< the selected action */
  (Operations)(walkToTarget) requestedOperations, /**< the requested action */
  (bool)(false) allowGoalKicks, /**< allows behavior to score */
  (bool)(false) teammatePlaysBall, /**< Does the teammate currently play the ball? */
});

STREAMABLE_WITH_BASE(SharedAutonomyRequest2, SharedAutonomyRequest,
{,
});
