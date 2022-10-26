#pragma once

#include "Streaming/AutoStreamable.h"
#include "Math/Pose2f.h"

STREAMABLE(GoaliePose,
{
  void draw() const;
  void verify() const,

  (Pose2f) goaliePoseField,        /**< Current walk target in absolute coordinates */
  (Pose2f) goaliePoseRel,          /**< Current walk target in relative coordinates */
  (bool)(false) isNearLeftPost,    /**< Whether the keeper is standing near to the left goal post and should thus not jump */
  (bool)(false) isNearRightPost,   /**< Whether the keeper is standing near to the right goal post and should thus not jump */
});
